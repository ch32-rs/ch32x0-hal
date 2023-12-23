#![no_std]

pub use ch32x0::ch32x035 as pac;

pub mod rcc;

pub mod debug;
pub mod delay;

pub mod signature;

mod peripheral;
pub use peripheral::*;
pub mod peripherals;
