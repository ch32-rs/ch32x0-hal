#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32x0_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Instant, Timer};
use hal::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use hal::{peripherals, println};

// pub struct Opa

#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(150)).await;
        led.set_low();
        Timer::after(Duration::from_millis(150)).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();

    let mut p = hal::init(Default::default());
    hal::embassy::init();

    let mut delay = Delay;

    // GPIO
    spawner.spawn(blink(p.PB12.degrade())).unwrap();

    println!("\n\nHello World from ch32x0-hal!");

    let mut opa = hal::opa::OpAmp::new(p.OPA2);
    // IN PA7, OUT PA4
    // Undocumented: PA2 is not ADC usable.
    let out = opa.buffer_non_inverting(&mut p.PA7, &mut p.PA4, hal::opa::OpAmpGain::Mul4);

    let mut adc = hal::adc::Adc::new(p.ADC1, &mut delay, Default::default());

    let mut ch = p.PA4;
    adc.configure_channel(&mut ch, 1, hal::adc::SampleTime::Cycles6);

    loop {
        //led.toggle();
        println!("inst => {:?}", Instant::now());
        // Delay.delay_ms(1000_u32); // blocking delay
        Timer::after(Duration::from_millis(1000)).await;

        let val = adc.convert(&mut ch);
        // div by 4
        let voltage = (val as u32) * 3300 / 4096 / 4;

        println!("raw => {}, val = {}mV", val, voltage);
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("\n\n\n{}", info);

    loop {}
}
