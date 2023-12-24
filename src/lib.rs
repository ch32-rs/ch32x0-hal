#![no_std]

pub use ch32x0::ch32x035 as pac;

pub use pac::interrupt;

pub mod rt;

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
#[cfg(feature = "embassy")]
pub mod embassy;

#[derive(Copy, Clone, Eq, PartialEq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {}

pub fn init(config: Config) -> Peripherals {
    rcc::init();

    gpio::init();

    peripherals::Peripherals::take()
}
