#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::adc::SampleTime;
use hal::gpio::{Level, Output};
use hal::println;
use {ch32x0_hal as hal, panic_halt as _};

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());
    hal::embassy::init();

    let mut adc = hal::adc::Adc::new(p.ADC1, Default::default());

    // ADC的通道3、通道7、通道11、通道15和I2C功能不适用于批号倒数第5位为0的产品
    let mut ch = p.PB1;
    // let mut ch = p.PA2;

    // GPIO
    let mut led = Output::new(p.PB12, Level::Low);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;

        let val = adc.convert(&mut ch, SampleTime::Cycles6);
        let voltage = (val as u32) * 3300 / 4096; // 3V3 as Vref

        println!("val => {}, {}mV", val, voltage);
    }
}
