#![no_std]

pub use ch32x0::ch32x035 as pac;

pub mod rcc;

pub mod debug;
pub mod delay;

pub mod signature;

mod peripheral;
pub use peripheral::*;
pub use peripherals::Peripherals;
pub mod peripherals;

pub mod gpio;

mod critical_section;

pub fn init() -> Peripherals {
    gpio::init();

    peripherals::Peripherals::take()
}
