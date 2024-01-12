//! SPI

/*
Supports full-duplex synchronous serial mode
Supports single-wire half-duplex mode
Supports master and slave modes, multiple slave modes
Supports 8-bit or 16-bit data structures
The highest clock frequency supports up to half of F_HCLK
Data order supports MSB or LSB first
Supports hardware or software control of NSS pin
Transmission and reception support hardware CRC check
Transmission and reception buffers support DMA transfer
Supports changing clock phase and polarity
*/

use embedded_hal::spi::{Mode, MODE_0};
use fugit::HertzU32 as Hertz;

use crate::{peripherals, Peripheral};

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    Framing,
    Crc,
    ModeFault,
    Overrun,
}

#[derive(Copy, Clone)]
pub enum BitOrder {
    LsbFirst,
    MsbFirst,
}

#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    pub mode: Mode,
    pub bit_order: BitOrder,
    pub frequency: Hertz,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: MODE_0,
            bit_order: BitOrder::MsbFirst,
            frequency: Hertz::from_raw(1_000_000),
        }
    }
}
