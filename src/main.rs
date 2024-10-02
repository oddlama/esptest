#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;

use defmt::{info, unwrap};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Io, Level, Output, Pin},
    i2c::I2C,
    peripherals::I2C0,
    prelude::_fugit_RateExtU32,
    timer::timg::TimerGroup,
    Blocking,
};
use esp_println as _;
use ssd1306::mode::DisplayConfig;
use ssd1306::{prelude::DisplayRotation, I2CDisplayInterface, Ssd1306};
use static_cell::StaticCell;

static I2C_BUS: StaticCell<Mutex<CriticalSectionRawMutex, RefCell<I2C<'static, I2C0, Blocking>>>> =
    StaticCell::new();

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

    let led1 = Output::new(io.pins.gpio15.degrade(), Level::High);
    unwrap!(spawner.spawn(blink(led1)));
    unwrap!(spawner.spawn(display(I2cDevice::new(i2c_bus))));
}

#[embassy_executor::task]
async fn display(
    i2c_device: I2cDevice<'static, CriticalSectionRawMutex, I2C<'static, I2C0, Blocking>>,
) {
    info!("Initialized display!");
    let interface = I2CDisplayInterface::new(i2c_device);
    let mut display = Ssd1306::new(
        interface,
        ssd1306::size::DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    display.init().unwrap();

    loop {
        display.clear(BinaryColor::Off).unwrap();
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

        Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        info!("update display!");
        display.flush().unwrap();
        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
async fn blink(mut pin: Output<'static>) {
    loop {
        info!("blink");
        pin.set_low();
        Timer::after(Duration::from_millis(50)).await;
        pin.set_high();
        Timer::after(Duration::from_millis(950)).await;
    }
}
