#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use esp_println as _;
use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio::{ErasedPin, Io, Level, Output, Pin},
    timer::timg::TimerGroup,
};

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    info!("defmt init");

    let led1 = Output::new(io.pins.gpio13.degrade(), Level::High);
    let led2 = Output::new(io.pins.gpio12.degrade(), Level::High);
    let led3 = Output::new(io.pins.gpio11.degrade(), Level::High);
    spawner.spawn(blink(led1)).unwrap();
    spawner.spawn(blink(led2)).unwrap();
    spawner.spawn(blink(led3)).unwrap();

    loop {
        info!("nop");
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn blink(mut pin: Output<'static, ErasedPin>) {
    loop {
        info!("blink");
        pin.set_low();
        Timer::after(Duration::from_millis(50)).await;
        pin.set_high();
        Timer::after(Duration::from_millis(950)).await;
    }
}
