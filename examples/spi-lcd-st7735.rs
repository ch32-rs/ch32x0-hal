//! TFT LCD ST7735, 160x80
//!
//! All crate.io drivers suck!
//! Let's create our own!
//! Check how to write one in 100 lines of code.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
use core::fmt::Write;

use display_interface_spi::SPIInterface;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::{OriginDimensions, Size};
use embedded_graphics::mono_font::ascii::FONT_9X18;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::raw::ToBytes;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyle};
use embedded_graphics::text::{Alignment, Text, TextStyle};
use embedded_hal::delay::DelayNs;
use hal::dma::NoDma;
use hal::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use hal::prelude::*;
use hal::spi::Spi;
use hal::{peripherals, println};
use {ch32x0_hal as hal, panic_halt as _};

#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(1000)).await;
        led.set_low();
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Instruction {
    NOP = 0x00,
    SWRESET = 0x01,
    RDDID = 0x04,
    RDDST = 0x09,
    SLPIN = 0x10,
    SLPOUT = 0x11,
    PTLON = 0x12,
    NORON = 0x13,
    INVOFF = 0x20,
    INVON = 0x21,
    DISPOFF = 0x28,
    DISPON = 0x29,
    CASET = 0x2A,
    RASET = 0x2B,
    RAMWR = 0x2C,
    RAMRD = 0x2E,
    PTLAR = 0x30,
    COLMOD = 0x3A,
    MADCTL = 0x36,
    FRMCTR1 = 0xB1,
    FRMCTR2 = 0xB2,
    FRMCTR3 = 0xB3,
    INVCTR = 0xB4,
    DISSET5 = 0xB6,
    PWCTR1 = 0xC0,
    PWCTR2 = 0xC1,
    PWCTR3 = 0xC2,
    PWCTR4 = 0xC3,
    PWCTR5 = 0xC4,
    VMCTR1 = 0xC5,
    RDID1 = 0xDA,
    RDID2 = 0xDB,
    RDID3 = 0xDC,
    RDID4 = 0xDD,
    PWCTR6 = 0xFC,
    GMCTRP1 = 0xE0,
    GMCTRN1 = 0xE1,
}

/// 132 (H) x RGB x 162 (V) bits
pub struct ST7735<const WIDTH: u16, const HEIGHT: u16, const OFFSETX: u16, const OFFSETY: u16> {
    spi: Spi<'static, peripherals::SPI1, NoDma, NoDma>,
    dc: Output<'static, AnyPin>,
    // _marker: core::marker::PhantomData<(OFFSETX, OFFSETY)>,
}

impl<const WIDTH: u16, const HEIGHT: u16, const OFFSETX: u16, const OFFSETY: u16>
    ST7735<WIDTH, HEIGHT, OFFSETX, OFFSETY>
{
    pub fn new(spi: Spi<'static, peripherals::SPI1, NoDma, NoDma>, dc: Output<'static, AnyPin>) -> Self {
        Self {
            spi,
            dc,
            //   _marker: core::marker::PhantomData,
        }
    }

    pub fn init(&mut self) {
        self.send_command(Instruction::SWRESET);
        Delay.delay_ms(20);
        self.send_command(Instruction::SLPOUT);
        Delay.delay_ms(200);

        self.send_command_data(Instruction::FRMCTR1, &[0x01, 0x2C, 0x2D]);
        self.send_command_data(Instruction::FRMCTR2, &[0x01, 0x2C, 0x2D]);
        self.send_command_data(Instruction::FRMCTR3, &[0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D]);
        self.send_command_data(Instruction::INVCTR, &[0x07]);
        self.send_command_data(Instruction::PWCTR1, &[0xA2, 0x02, 0x84]);
        self.send_command_data(Instruction::PWCTR2, &[0xC5]);
        self.send_command_data(Instruction::PWCTR3, &[0x0A, 0x00]);
        self.send_command_data(Instruction::PWCTR4, &[0x8A, 0x2A]);
        self.send_command_data(Instruction::PWCTR5, &[0x8A, 0xEE]);
        self.send_command_data(Instruction::VMCTR1, &[0x0E]);
        self.send_command_data(Instruction::INVON, &[]);

        // BITS:
        // MY, MX, MV, ML,
        // RGB, MV, _, _
        self.send_command_data(Instruction::MADCTL, &[0b0110_1000]);

        self.send_command_data(Instruction::COLMOD, &[0x05]); // 16-bit/pixel
        self.send_command_data(Instruction::DISPON, &[]);

        Delay.delay_ms(200);
    }

    #[inline]
    fn set_update_window(&mut self, x: u16, y: u16, w: u16, h: u16) {
        let ox = OFFSETX + x;
        let oy = OFFSETY + y;

        self.send_command_data(
            Instruction::CASET,
            &[
                (ox >> 8) as u8,
                (ox & 0xFF) as u8,
                ((ox + w - 1) >> 8) as u8,
                ((ox + w - 1) & 0xFF) as u8,
            ],
        );

        self.send_command_data(
            Instruction::RASET,
            &[
                (oy >> 8) as u8,
                (oy & 0xFF) as u8,
                ((oy + h - 1) >> 8) as u8,
                ((oy + h - 1) & 0xFF) as u8,
            ],
        );
    }

    pub fn write_raw_pixel(&mut self, x: u16, y: u16, data: &[u8]) {
        self.set_update_window(x, y, 1, 1);

        self.send_command_data(Instruction::RAMWR, data);
    }

    fn send_command(&mut self, cmd: Instruction) {
        self.dc.set_low();
        self.spi.blocking_write(&[cmd as u8]).unwrap();
    }

    fn send_data(&mut self, data: &[u8]) {
        self.dc.set_high();
        self.spi.blocking_write(data).unwrap();
    }

    fn send_command_data(&mut self, cmd: Instruction, data: &[u8]) {
        self.send_command(cmd);
        self.send_data(data);
    }
}

impl<const WIDTH: u16, const HEIGHT: u16, const OFFSETX: u16, const OFFSETY: u16> OriginDimensions
    for ST7735<WIDTH, HEIGHT, OFFSETX, OFFSETY>
{
    fn size(&self) -> Size {
        Size::new(WIDTH as _, HEIGHT as _)
    }
}

impl<const WIDTH: u16, const HEIGHT: u16, const OFFSETX: u16, const OFFSETY: u16> DrawTarget
    for ST7735<WIDTH, HEIGHT, OFFSETX, OFFSETY>
{
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::prelude::Pixel<Self::Color>>,
    {
        for pixel in pixels {
            let x = pixel.0.x as u16;
            let y = pixel.0.y as u16;
            let color = pixel.1;

            self.write_raw_pixel(x, y, color.to_be_bytes().as_ref());
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.set_update_window(0, 0, WIDTH, HEIGHT);

        self.send_command(Instruction::RAMWR);
        for _ in 0..((WIDTH as u16) * (HEIGHT as u16)) {
            self.send_data(color.to_be_bytes().as_ref());
        }
        Ok(())
    }

    fn fill_contiguous<I>(
        &mut self,
        area: &embedded_graphics::primitives::Rectangle,
        colors: I,
    ) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        self.set_update_window(
            area.top_left.x as u16,
            area.top_left.y as u16,
            area.size.width as u16,
            area.size.height as u16,
        );

        self.send_command(Instruction::RAMWR);
        for color in colors {
            self.send_data(color.to_be_bytes().as_ref());
        }
        Ok(())
    }

    fn fill_solid(
        &mut self,
        area: &embedded_graphics::primitives::Rectangle,
        color: Self::Color,
    ) -> Result<(), Self::Error> {
        self.set_update_window(
            area.top_left.x as u16,
            area.top_left.y as u16,
            area.size.width as u16,
            area.size.height as u16,
        );

        self.send_command(Instruction::RAMWR);
        for _ in 0..(area.size.width * area.size.height) {
            self.send_data(color.to_be_bytes().as_ref());
        }
        Ok(())
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    // let p = hal::init(Default::default());
    let p = hal::init(Default::default());
    hal::embassy::init();

    let mut delay = Delay;

    // SPI1, remap 0
    let cs = p.PA4;
    let sda = p.PA7;
    let sck = p.PA5;

    let rst = p.PA1;
    let dc = p.PA2;

    let vbus_div = p.PB1;

    let led = p.PB12;
    let button = p.PC3;

    let mut button = Input::new(button, Pull::Down);

    let mut cs = Output::new(cs, Level::High);
    let dc = Output::new(dc.degrade(), Level::High);
    let mut rst = Output::new(rst, Level::High);

    cs.set_low();

    let mut spi_config = hal::spi::Config::default();
    spi_config.frequency = 12.MHz();

    let mut spi = Spi::new_txonly(p.SPI1, sck, sda, NoDma, NoDma, spi_config);

    rst.set_low();
    Timer::after_millis(120).await;
    rst.set_high();
    Timer::after_millis(20).await;

    // let mut display: ST7735<80, 160, 26, 0> = ST7735::new(spi, dc);
    let mut display: ST7735<160, 80, 1, 26> = ST7735::new(spi, dc);

    println!("display init ...");

    display.init();

    // GPIO, // T1C4
    spawner.spawn(blink(led.degrade())).unwrap();

    let mut i = 0;

    display.clear(Rgb565::CSS_DARK_SLATE_BLUE).unwrap();

    Line::new(Point::new(0, 0), Point::new(159, 0))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 1))
        .draw(&mut display)
        .unwrap();
    Line::new(Point::new(0, 0), Point::new(0, 79))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1))
        .draw(&mut display)
        .unwrap();
    Line::new(Point::new(159, 0), Point::new(159, 79))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLUE, 1))
        .draw(&mut display)
        .unwrap();
    Line::new(Point::new(0, 79), Point::new(159, 79))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(&mut display)
        .unwrap();

    let mut buf = heapless::String::<128>::new();
    let mut i = 0;
    loop {
        Timer::after_millis(1).await;

        // display.write_ram(40, 30, &[i, i, i]);

        i += 1;

        buf.clear();

        let val = button.is_high();
        core::write!(&mut buf, "Hi, Rust! {}\nclicked: {:05}", i, val).unwrap();

        let mut character_style = MonoTextStyle::new(&FONT_9X18, Rgb565::CSS_TOMATO);
        character_style.background_color = Some(Rgb565::CSS_DARK_SLATE_BLUE);

        Text::with_alignment(
            buf.as_str(),
            display.bounding_box().center(),
            character_style,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();
    }
}
