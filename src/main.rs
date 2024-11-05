#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::fmt::Write;
use core::{cell::RefCell, fmt::Debug};

use arrayvec::ArrayString;
use defmt::{info, unwrap};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
use embassy_time::{Duration, Timer};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
    text::{Baseline, Text},
};
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, AdcPin, Attenuation},
    gpio::{Input, Io, Level, Output, Pin},
    i2c::I2C,
    peripherals::I2C0,
    prelude::*,
    timer::{systimer::SystemTimer, timg::TimerGroup},
    Blocking,
};
use esp_println as _;
use ringbuffer::{ConstGenericRingBuffer, RingBuffer};
use ssd1306::mode::DisplayConfig;
use ssd1306::{prelude::DisplayRotation, I2CDisplayInterface, Ssd1306};
use static_cell::StaticCell;

static I2C_BUS: StaticCell<Mutex<CriticalSectionRawMutex, RefCell<I2C<'static, I2C0, Blocking>>>> =
    StaticCell::new();

enum Event {
    Timeout { elapsed_ticks: u64, n_reads: u64 },
    Trigger { elapsed_ticks: u64, n_reads: u64 },
}

const TEXT_STYLE: MonoTextStyle<'static, BinaryColor> = MonoTextStyleBuilder::new()
    .font(&FONT_6X10)
    .text_color(BinaryColor::On)
    .build();

async fn wait_for_event<'a, ADC, PIN, B: RingBuffer<u16>>(
    adc: &mut Adc<'a, ADC>,
    adc_pin: &mut AdcPin<PIN, ADC>,
    adc_buffer: &mut B,
    timeout_ticks: u64,
    threshold: u16,
) -> Event
where
    ADC: esp_hal::analog::adc::RegisterAccess,
    PIN: esp_hal::analog::adc::AdcChannel,
{
    let t_start = SystemTimer::now();
    let t_timeout = t_start + timeout_ticks;
    let mut last_value = 0;
    let mut n_reads = 0;
    loop {
        // Abort if nothing happened for 1 second
        let t_now = SystemTimer::now();
        if t_now > t_timeout {
            return Event::Timeout {
                elapsed_ticks: t_now - t_start,
                n_reads,
            };
        }

        // Read adc non-blocking
        n_reads += 1;
        let pin_value = nb::block!(adc.read_oneshot(adc_pin))
            .expect("Failure while waiting for ADC conversion");

        // adc_buffer.push((t_now, pin_value));
        adc_buffer.push(pin_value);
        if pin_value > threshold && last_value <= threshold {
            return Event::Trigger {
                elapsed_ticks: t_now - t_start,
                n_reads,
            };
        }
        last_value = pin_value;
    }
}

fn draw_last_events<B: RingBuffer<u64>, D: DrawTarget<Color = BinaryColor>>(
    event_buffer: &B,
    display: &mut D,
) where
    <D as embedded_graphics::draw_target::DrawTarget>::Error: Debug,
{
    for (i, x) in event_buffer.iter().take(3).enumerate() {
        let t_diff_100_us = x * 10_000 / SystemTimer::ticks_per_second();

        let mut text = ArrayString::<32>::new();
        write!(
            &mut text,
            "{:>3}.{:<1}",
            t_diff_100_us / 10,
            t_diff_100_us % 10,
        )
        .unwrap();

        Text::with_baseline(
            &text,
            Point::new(128 - 6 * text.len() as i32, 64 - i as i32 * 10),
            TEXT_STYLE,
            Baseline::Bottom,
        )
        .draw(display)
        .unwrap();
    }
}

fn draw_average<B: RingBuffer<u64>, D: DrawTarget<Color = BinaryColor>>(
    event_buffer: &B,
    display: &mut D,
) where
    <D as embedded_graphics::draw_target::DrawTarget>::Error: Debug,
{
    if event_buffer.is_full() {
        // Display average
        let t_avg_ticks = event_buffer.iter().sum::<u64>() / event_buffer.len() as u64;
        let t_diff_100_us = t_avg_ticks * 10_000 / SystemTimer::ticks_per_second();

        let mut text = ArrayString::<32>::new();
        write!(
            &mut text,
            "t_avg:   {:>3}.{:<3}ms",
            t_diff_100_us / 10,
            t_diff_100_us % 10,
        )
        .unwrap();

        Text::with_baseline(&text, Point::new(0, 20), TEXT_STYLE, Baseline::Top)
            .draw(display)
            .unwrap();

        info!("  {}", text.as_str());
    }
}

fn draw_graph_border<D: DrawTarget<Color = BinaryColor>>(display: &mut D)
where
    <D as embedded_graphics::draw_target::DrawTarget>::Error: Debug,
{
    let line_style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

    // X-Axis
    Line::new(Point::new(0, 63), Point::new(4, 63))
        .into_styled(line_style)
        .draw(display)
        .unwrap();
    Line::new(Point::new(0, 63), Point::new(0, 59))
        .into_styled(line_style)
        .draw(display)
        .unwrap();
    // Y-Axis
    Line::new(Point::new(94, 32), Point::new(90, 32))
        .into_styled(line_style)
        .draw(display)
        .unwrap();
    Line::new(Point::new(94, 32), Point::new(94, 36))
        .into_styled(line_style)
        .draw(display)
        .unwrap();
}

fn show_message<D: DrawTarget<Color = BinaryColor>>(message: &str, display: &mut D)
where
    <D as embedded_graphics::draw_target::DrawTarget>::Error: Debug,
{
    Text::with_baseline(
        message,
        Point::new(46 - (message.len() as i32 * 6 / 2), 43),
        TEXT_STYLE,
        Baseline::Top,
    )
    .draw(display)
    .unwrap();
}

fn draw_graph<B: RingBuffer<u16>, D: DrawTarget<Color = BinaryColor>>(
    adc_buffer: &B,
    display: &mut D,
) where
    <D as embedded_graphics::draw_target::DrawTarget>::Error: Debug,
{
    let line_style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

    let min = *adc_buffer.iter().min().unwrap_or(&0) as i32;
    let max = *adc_buffer.iter().max().unwrap_or(&4000) as i32;
    let rescale = |x: i32| (x - min) * 30 / (max - min);

    let mut prev_y = rescale(*adc_buffer.front().unwrap_or(&0) as i32);
    for (i, &y) in adc_buffer.iter().enumerate().skip(1).take(91) {
        let i = i as i32;
        let y = rescale(y as i32);
        Line::new(Point::new(1 + i, 63 - prev_y), Point::new(2 + i, 63 - y))
            .into_styled(line_style)
            .draw(display)
            .unwrap();
        prev_y = y;
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Select internal antenna
    let _ = Output::new(io.pins.gpio14.degrade(), Level::Low);

    // I2C bus
    let sda = io.pins.gpio22;
    let scl = io.pins.gpio23;
    let i2c_bus = I2C::new(peripherals.I2C0, sda, scl, 200u32.kHz());
    let i2c_bus = I2C_BUS.init(Mutex::new(RefCell::new(i2c_bus)));

    // Create ADC instance
    let mut adc1_config = AdcConfig::new();
    let mut adc1_pin = adc1_config.enable_pin(io.pins.gpio0, Attenuation::Attenuation11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    let interface = I2CDisplayInterface::new(I2cDevice::new(i2c_bus));
    let mut display = Ssd1306::new(
        interface,
        ssd1306::size::DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    display.init().unwrap();
    display.clear(BinaryColor::Off).unwrap();
    display.flush().unwrap();

    Text::with_baseline(
        "t_event: None\nt_res:   0.0us",
        Point::zero(),
        TEXT_STYLE,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    draw_graph_border(&mut display);
    show_message("Press trigger", &mut display);
    display.flush().unwrap();

    info!("Initialized SSD1306 display");

    // Get trigger button
    let mut button = Input::new(io.pins.gpio1, esp_hal::gpio::Pull::Up);

    // Start LED blink
    let led1 = Output::new(io.pins.gpio15.degrade(), Level::High);
    unwrap!(spawner.spawn(blink(led1)));

    let mut event_buffer = ConstGenericRingBuffer::<_, 10>::new();
    let mut adc_buffer = ConstGenericRingBuffer::<_, 100>::new();
    let mut text = ArrayString::<64>::new();
    let threshold = 2700;

    loop {
        // Wait until trigger button goes low
        button.wait_for_falling_edge().await;
        let event = wait_for_event(
            &mut adc1,
            &mut adc1_pin,
            &mut adc_buffer,
            // maximum time 200ms
            SystemTimer::ticks_per_second() * 200 / 1000,
            threshold,
        )
        .await;

        display.clear(BinaryColor::Off).unwrap();
        draw_graph_border(&mut display);

        match event {
            Event::Timeout {
                elapsed_ticks,
                n_reads,
            } => {
                let t_diff_100_ns = elapsed_ticks * 10_000_000 / SystemTimer::ticks_per_second();
                let t_res_100_ns = t_diff_100_ns / n_reads;
                text.clear();
                write!(
                    &mut text,
                    "t_event: None\nt_res:   {:>3}.{:<3}us",
                    t_res_100_ns / 10,
                    t_res_100_ns % 10,
                )
                .unwrap();

                show_message("No event", &mut display);

                info!(
                    "t_event: None, t_res: {}.{}ms",
                    t_res_100_ns / 10000,
                    t_res_100_ns % 10000,
                );
            }
            Event::Trigger {
                elapsed_ticks,
                n_reads,
            } => {
                // Events quicker than 2 milliseconds are ignored in the average
                let ignore = elapsed_ticks * 1_000_000 / SystemTimer::ticks_per_second() < 2_000;
                if ignore {
                    show_message("Too short", &mut display);
                } else {
                    event_buffer.push(elapsed_ticks);
                    draw_graph(&adc_buffer, &mut display);
                }

                let t_diff_100_ns = elapsed_ticks * 10_000_000 / SystemTimer::ticks_per_second();
                let t_diff_100_us = t_diff_100_ns / 1000;
                let t_res_100_ns = t_diff_100_ns / n_reads;
                text.clear();
                write!(
                    &mut text,
                    "t_event: {:>3}.{:<3}ms\nt_res:   {:>3}.{:<3}us",
                    t_diff_100_us / 10,
                    t_diff_100_us % 10,
                    t_res_100_ns / 10,
                    t_res_100_ns % 10,
                )
                .unwrap();

                info!(
                    "t_event: {}.{}ms, t_res: {}.{}us",
                    t_diff_100_ns / 10000,
                    t_diff_100_ns % 10000,
                    t_res_100_ns / 10,
                    t_res_100_ns % 10,
                );
            }
        };

        Text::with_baseline(&text, Point::zero(), TEXT_STYLE, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        draw_average(&event_buffer, &mut display);
        draw_last_events(&event_buffer, &mut display);
        display.flush().unwrap();
        adc_buffer.clear();
    }
}

#[embassy_executor::task]
async fn blink(mut pin: Output<'static>) {
    loop {
        pin.set_low();
        Timer::after(Duration::from_millis(50)).await;
        pin.set_high();
        Timer::after(Duration::from_millis(950)).await;
    }
}
