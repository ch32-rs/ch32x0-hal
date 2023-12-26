#![no_std]

pub use ch32x0::ch32x035 as pac;
pub use pac::interrupt;

pub mod rt;

pub mod rcc;

pub mod debug;
pub mod delay;

mod peripheral;
pub use peripheral::*;
pub use peripherals::Peripherals;
pub mod peripherals;

pub mod exti;
pub mod gpio;
pub mod pioc;
pub mod signature;
pub mod spi;
pub mod usart;

mod critical_section_impl;
#[cfg(feature = "embassy")]
pub mod embassy;

#[derive(Copy, Clone, Eq, PartialEq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {}

pub fn init(config: Config) -> Peripherals {
    rcc::init();

    gpio::init();

    ::critical_section::with(|cs| unsafe {
        exti::init(cs);
    });

    peripherals::Peripherals::take()
}
