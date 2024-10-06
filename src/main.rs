#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;
use core::fmt::Write;

use arrayvec::ArrayString;
use defmt::{info, unwrap};
use embassy_embedded_hal::shared_bus::{asynch::spi::SpiDevice, blocking::i2c::I2cDevice};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
use embassy_time::{Duration, Timer};
use embedded_devices::devices::microchip::mcp3208::{InputChannel, MCP3208};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    dma::{Dma, DmaChannel0, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{Input, Io, Level, Output, Pin},
    i2c::I2C,
    peripherals::I2C0,
    prelude::*,
    rtc_cntl::Rtc,
    spi::{master::Spi, FullDuplexMode, SpiMode},
    timer::{systimer::SystemTimer, timg::TimerGroup},
    Async, Blocking,
};
use esp_println as _;
use ssd1306::mode::DisplayConfig;
use ssd1306::{prelude::DisplayRotation, I2CDisplayInterface, Ssd1306};
use static_cell::StaticCell;
use uom::num_rational::Rational32;
use uom::si::{electric_potential::volt, rational32::ElectricPotential};

static I2C_BUS: StaticCell<Mutex<CriticalSectionRawMutex, RefCell<I2C<'static, I2C0, Blocking>>>> =
    StaticCell::new();

static SPI_BUS: StaticCell<
    embassy_sync::mutex::Mutex<
        CriticalSectionRawMutex,
        esp_hal::spi::master::SpiDmaBus<
            '_,
            esp_hal::peripherals::SPI2,
            DmaChannel0,
            FullDuplexMode,
            Async,
        >,
    >,
> = StaticCell::new();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    // let delay = Delay::new();

    // Select internal antenna
    let _ = Output::new(io.pins.gpio14.degrade(), Level::Low);

    // I2C bus
    let sda = io.pins.gpio22;
    let scl = io.pins.gpio23;
    let i2c_bus = I2C::new(peripherals.I2C0, sda, scl, 200u32.kHz());
    let i2c_bus = I2C_BUS.init(Mutex::new(RefCell::new(i2c_bus)));

    // SPI bus
    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let sck = io.pins.gpio19;
    let mosi = io.pins.gpio18;
    let miso = io.pins.gpio20;
    let spi = Spi::new(peripherals.SPI2, 1000.kHz(), SpiMode::Mode0)
        .with_mosi(mosi)
        .with_miso(miso)
        .with_sck(sck)
        .with_dma(dma_channel.configure_for_async(false, DmaPriority::Priority0))
        .with_buffers(dma_rx_buf, dma_tx_buf);
    let spi = embassy_sync::mutex::Mutex::<CriticalSectionRawMutex, _>::new(spi);
    let spi = SPI_BUS.init(spi);

    let cs = Output::new(io.pins.gpio17.degrade(), Level::Low); // dummy
    let spi_device = SpiDevice::new(spi, cs);

    let mut mcp3208 = MCP3208::new_spi(
        spi_device,
        ElectricPotential::new::<volt>(Rational32::new(3, 1)),
    );

    let rtc = Rtc::new(peripherals.LPWR);
    let systimer = SystemTimer::new(peripherals.SYSTIMER);

    let led1 = Output::new(io.pins.gpio15.degrade(), Level::High);
    unwrap!(spawner.spawn(blink(led1)));
    // unwrap!(spawner.spawn(display(
    //     I2cDevice::new(i2c_bus),
    //     io.pins.gpio0,
    //     peripherals.ADC1
    // )));
    // loop {
    //     let pin_value = mcp3208.convert_raw(InputChannel::Single0).await.unwrap();
    //     info!("val: {}", pin_value);
    //     Timer::after_millis(500).await;
    // }

    // Create ADC instance
    let mut adc1_config = AdcConfig::new();
    let mut adc1_pin = adc1_config.enable_pin(io.pins.gpio0, Attenuation::Attenuation11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    info!("Initialized display!");
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

    let mut button = Input::new(io.pins.gpio1, esp_hal::gpio::Pull::Up);

    loop {
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

        Text::with_baseline(
            "Waiting for trigger..",
            Point::new(0, 20),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();
        display.flush().unwrap();

        button.wait_for_falling_edge().await;

        let t_start = rtc.current_time().and_utc().timestamp_micros();
        let mut last_value = 0;
        let mut reads = 0;
        let mut t_end;
        let mut reached = false;
        let threshold = 3100;
        loop {
            t_end = rtc.current_time().and_utc().timestamp_micros();
            if t_end - t_start > 10_000_000 {
                break;
            }

            reads += 1;
            let pin_value = nb::block!(adc1.read_oneshot(&mut adc1_pin)).unwrap();
            // let pin_value = mcp3208.convert_raw(InputChannel::Single0).await.unwrap();
            if pin_value > threshold && last_value <= threshold {
                last_value = pin_value;
                reached = true;
                break;
            }
            last_value = pin_value;
        }

        let mut text = ArrayString::<64>::new();
        let t_diff = t_end - t_start;
        if reached {
            write!(
                &mut text,
                "Delta T: {}.{}ms\nTime res: {}us\nValue: {}",
                t_diff / 1000,
                t_diff % 1000,
                t_diff / reads,
                last_value
            )
            .unwrap();
        } else {
            write!(
                &mut text,
                "No event.\nTime resolution: {}us",
                t_diff / reads
            )
            .unwrap();
        }
        info!("{}", text.as_str());

        display.clear(BinaryColor::Off).unwrap();
        Text::with_baseline(&text, Point::zero(), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        display.flush().unwrap();
    }
}

// #[embassy_executor::task]
// async fn display(
//     i2c_device: I2cDevice<'static, CriticalSectionRawMutex, I2C<'static, I2C0, Blocking>>,
//     pin: GpioPin<0>,
//     adc1: ADC1,
// ) {
// }

#[embassy_executor::task]
async fn blink(mut pin: Output<'static>) {
    loop {
        pin.set_low();
        Timer::after(Duration::from_millis(50)).await;
        pin.set_high();
        Timer::after(Duration::from_millis(950)).await;
    }
}
