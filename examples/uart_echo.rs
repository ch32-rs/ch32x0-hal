#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::usart::{Uart, UartTx};
use hal::{println, usart};
use {ch32x0_hal as hal, panic_halt as _};

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());
    hal::embassy::init();

    // GPIO
    let mut led = Output::new(p.PA4, Level::Low);

    led.toggle();

    let mut cfg = usart::Config::default();
    //cfg.baudrate = 1000000;
    let mut uart = Uart::new(p.USART2, p.PA2, p.PA3, cfg).unwrap();

    println!("will output to serial");

    uart.blocking_write(b"Init ok\r\n");
    led.toggle();

    loop {
        // Timer::after_millis(2000).await;
        while let Ok(b) = nb::block!(uart.nb_read()) {
            if b == b'\r' {
                uart.blocking_write(b"\r\n");
            } else {
                uart.blocking_write(&[b]);
            }
        }
    }
}
