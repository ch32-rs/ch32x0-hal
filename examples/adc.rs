#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::println;
use {ch32x0_hal as hal, panic_halt as _};

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    // let p = hal::init(Default::default());
    let p = hal::init(Default::default());
    hal::embassy::init();

    /*
        // SPI1, remap 0
        let cs = p.PA4;
        let sda = p.PA7;
        let sck = p.PA5;

        let rst = p.PA1;
        let dc = p.PA2;
    */
    // let vbus_div = p.PB1;
    let mut delay = Delay;

    let mut adc = hal::adc::Adc::new(p.ADC1, &mut delay, Default::default());

    // let mut ch = hal::adc::Vref;

    // let mut ch = p.PB1;
    let mut ch = p.PA7;
    adc.configure_channel(&mut ch, 1, hal::adc::SampleTime::Cycles11);

    // GPIO
    let mut led = Output::new(p.PB12, Level::Low);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;

        let val = adc.convert(&mut ch);
        let voltage = (val as u32) * 3300 / 4096; // 3V3 as Vref

        println!("val => {}, {}mV", val, voltage);
    }
}
