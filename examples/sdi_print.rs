#![no_std]
#![no_main]

//! SDI debug print

use core::arch::asm;

use embedded_hal_1::delay::DelayNs;
use hal::println;
use {ch32x0_hal as hal, panic_halt as _};

#[riscv_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();

    println!("hello world!");
    let mut delay = hal::delay::CycleDelay;

    println!("Flash size: {}kb", hal::signature::flash_size_kb());
    println!("Chip ID: {:x?}", hal::signature::unique_id());

    loop {
        println!("hello world!");

        delay.delay_ms(1000);
    }
}
