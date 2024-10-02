#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;

use defmt::{info, unwrap};
use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::{raw::NoopRawMutex, Mutex};
use embassy_time::{Duration, Timer};
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::{pixelcolor::Rgb666, prelude::RgbColor};
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Io, Level, Output, Pin},
    i2c::I2C,
    prelude::_fugit_RateExtU32,
    spi::{master::Spi, FullDuplexMode, SpiMode},
    timer::timg::TimerGroup,
};
use esp_println as _;
use mipidsi::models::ST7789;
use static_cell::StaticCell;

static SPI_BUS: StaticCell<
    Mutex<
        NoopRawMutex,
        RefCell<esp_hal::spi::master::Spi<'_, esp_hal::peripherals::SPI2, FullDuplexMode>>,
    >,
> = StaticCell::new();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let delay = Delay::new();

    // Select internal antenna
    let _ = Output::new(io.pins.gpio14.degrade(), Level::Low);

    // I2C bus
    let sda = io.pins.gpio21;
    let scl = io.pins.gpio22;
    let _i2c = I2C::new(peripherals.I2C0, sda, scl, 100u32.kHz());

    // SPI bus
    let sck = io.pins.gpio19;
    let mosi = io.pins.gpio18;
    let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0)
        .with_mosi(mosi)
        .with_sck(sck);
    let spi = Mutex::<NoopRawMutex, _>::new(RefCell::new(spi));
    let spi = SPI_BUS.init(spi);

    let cs = Output::new(io.pins.gpio5.degrade(), Level::Low); // dummy
    let display_device = SpiDevice::new(spi, cs);
    let dc = Output::new(io.pins.gpio2.degrade(), Level::Low);
    unwrap!(spawner.spawn(ui(delay, display_device, dc)));

    let led1 = Output::new(io.pins.gpio15.degrade(), Level::High);
    unwrap!(spawner.spawn(blink(led1)));
}

#[embassy_executor::task]
async fn ui(
    mut delay: Delay,
    display_device: SpiDevice<
        'static,
        NoopRawMutex,
        esp_hal::spi::master::Spi<'_, esp_hal::peripherals::SPI2, FullDuplexMode>,
        Output<'static>,
    >,
    dc: Output<'static>,
) {
    let di = SPIInterface::new(display_device, dc);

    let mut display = mipidsi::Builder::new(ST7789, di)
        .init(&mut delay)
        .expect("No 1");
    display.clear(Rgb666::WHITE.into()).expect("No 2");
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
