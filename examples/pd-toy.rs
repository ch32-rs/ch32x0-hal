#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(generic_const_exprs)]

use core::fmt::Write;
use core::future;

use ch32x0_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Delay, Duration, Ticker, Timer};
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::mono_font::ascii::{
    FONT_10X20, FONT_6X10, FONT_6X13, FONT_7X13, FONT_8X13, FONT_8X13_BOLD, FONT_9X18, FONT_9X18_BOLD,
};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::{BinaryColor, RgbColor};
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyleBuilder};
use embedded_graphics::text::{Alignment, DecorationColor, Text};
use embedded_hal::delay::DelayNs as _;
use hal::dma::NoDma;
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::prelude::*;
use hal::spi::{BitOrder, Spi};
use hal::usbpd::consts::ExtendedMessageType;
use hal::usbpd::protocol::{ExtendedHeader, Header, PowerDataObject};
use hal::usbpd::UsbPdSink;
use hal::{bind_interrupts, peripherals, println};
use memory_lcd_spi::displays::LPM013M126A;
use memory_lcd_spi::pixelcolor::Rgb111;
use memory_lcd_spi::MemoryLCD;

bind_interrupts!(struct Irqs {
    USBPD => hal::usbpd::InterruptHandler<peripherals::USBPD>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());
    hal::embassy::init();

    let mut delay = Delay;

    // GPIO
    let mut led = Output::new(p.PB12, Level::Low); // Orange LED
    let mut vbus_out_en = Output::new(p.PC1, Level::Low); // VBUS_OUT_EN

    let mut pd = UsbPdSink::new(p.USBPD, p.PC14, p.PC15);

    let cc = pd.detect_cc();
    if cc != 0 {
        println!("CC pin CC{:?}", cc);
    } else {
        println!("CC pin not connected");

        /*loop {
            led.set_high();
            Timer::after(Duration::from_millis(200)).await;
            led.set_low();
            Timer::after(Duration::from_millis(200)).await;
        }*/
    }

    vbus_out_en.set_high();

    if let Err(_) = with_timeout(Duration::from_secs(1), pd.receive_raw_packet(true)).await {
        unsafe {
            pd.hard_reset();
            println!("sending hard request");
        }
    }

    Timer::after(Duration::from_millis(100)).await;

    // println!("recv ok");
    // println!("raw: {:?}", raw);

    //println!("req pdo");

    //pd.request_pdo(6).await;

    // pd.receive_initial().await; // send per 0.18s
    let raw = pd.receive_raw_packet(true).await;

    let ret = pd.request_fixed_pdo(1).await;
    println!("Request fixed PDO: {:?}", ret);

    //  let ret = pd.request_pps_pdo(7, 10_000, 1_000).await;
    // println!("Request PPS PDO: {:?}", ret);
    match ret {
        Err(_) => {
            // pd.dump_pdo(raw);
        }
        _ => (),
    }
    Timer::after(Duration::from_millis(1000)).await;

    let ret = pd.enter_epr().await;
    println!("Enter EPR: {:?}", ret);

    let ret = pd.get_epr_src_cap().await;
    println!("EPR Src Cap: {:?}", ret);

    let caps = pd.last_capabilities();
    // println!("caps: {:?}", caps);

    //  for (i, &cap) in caps.iter().enumerate() {
    //      if cap != 0 {
    //          println!("#{}: {}", i + 1, PowerDataObject::from_u32(cap));
    //      }
    //  }
    // loop {}

    //let ret = pd.request_epr_pdo(2, 0x0002d12c).await;
    //let ret = pd.request_epr_pdo(5, 0x000641f4).await;
    let ret = pd.request_epr_pdo(4, caps[3]).await;
    println!("Request EPR PDO: {:?}", ret);

    if false {
        let ret = pd.epr_keep_alive().await;
        println!("EPR Keep Alive: {:?}", ret);

        let raw = pd.receive_raw_packet(true).await;
        println!("raw: {:?}", raw);
    }

    //println!("will enter EPR");
    //pd.enter_epr().await;

    //pd.get_cap().await;

    //loop {
    //    pd.get_cap().await;
    // }

    let mut adc = hal::adc::Adc::new(p.ADC, &mut delay, Default::default());
    let mut ch = p.PB1;
    adc.configure_channel(&mut ch, 1, hal::adc::SampleTime::Cycles6);

    // Display
    // SPI1, remap 0
    let cs = p.PA4;
    let sda = p.PA7;
    let sck = p.PA5;
    let mut spi_config = hal::spi::Config::default();
    spi_config.frequency = 4.MHz();
    // spi_config.bit_order = BitOrder::LsbFirst;
    let spi = Spi::new_txonly(p.SPI1, sck, sda, NoDma, NoDma, spi_config);

    let cs = Output::new(cs, Level::Low);
    let mut display: MemoryLCD<LPM013M126A<Rgb111>, _, _> = MemoryLCD::new(spi, cs);
    display.clear(Rgb111::BLACK);
    display.update(&mut delay);

    let mut buf = heapless::String::<128>::new();
    let mut i = 0;

    let mut style_item = MonoTextStyle::new(&FONT_8X13, Rgb111::WHITE);
    let mut style_item_selected = MonoTextStyle::new(&FONT_8X13_BOLD, Rgb111::BLACK);
    style_item_selected.background_color = Some(Rgb111::GREEN);

    let mut style_red = MonoTextStyle::new(&FONT_9X18_BOLD, Rgb111::RED);
    //  character_style.background_color = Some(Rgb111::BLACK);

    let mut style_heading = MonoTextStyle::new(&FONT_10X20, Rgb111::CYAN);
    style_heading.underline_color = DecorationColor::Custom(Rgb111::YELLOW);

    let mut style_footer = MonoTextStyle::new(&FONT_7X13, Rgb111::GREEN);

    let mut vals = [5000u32; 176];

    vbus_out_en.set_low();

    //    let mut request_voltage = 7640;

    let mut ticker = Ticker::every(hal::usbpd::timing::PPS_REQUEST_TIMEOUT);

    loop {
        //ticker.next().await;
        //let ret = pd.request_pps_pdo(6, 21_000, 1_000).await;
        //println!("Request PPS PDO(interval): {:?}", ret);

        // pd.get_pps_status().await;
        // pd.epr_keep_alive().await;
        let ret = pd.request_epr_pdo(8, caps[7]).await;

        Timer::after_millis(250).await;

        led.toggle();
        //       Timer::after(Duration::from_millis(200)).await;

        vbus_out_en.toggle();

        display.clear(Rgb111::BLACK);
        Text::with_alignment(
            "PD Toy v0.3",
            //display.bounding_box().center(),
            Point::new(80, 20),
            style_heading,
            Alignment::Center,
        )
        .draw(&mut *display)
        .unwrap();
        Text::with_alignment(
            "By @andelf with Rust",
            Point::new(80, 168),
            style_footer,
            Alignment::Center,
        )
        .draw(&mut *display)
        .unwrap();

        i += 1;

        // draw graph

        let val = adc.convert(&mut ch);
        // val as f32 / 4096.0 * 3.3 / 12.0 * (12.0 + 68.0);
        let voltage = (val as u32) * 3300 * (12 + 120) / 4096 / 12;

        vals.copy_within(1..176, 0);
        *vals.last_mut().unwrap() = voltage;

        let max = vals.iter().max().unwrap();
        let min = vals.iter().min().unwrap().min(&3000);

        let stride = max - min;

        let height = 40;
        let mut start_x = 0;
        for pairs in vals.windows(2) {
            if let &[fst, snd] = pairs {
                let y0 = (fst - min) * height / stride;
                let y1 = (snd - min) * height / stride;

                Line::new(
                    Point::new(start_x, 175 - y0 as i32),
                    Point::new(start_x + 1, 175 - y1 as i32),
                )
                .into_styled(
                    PrimitiveStyleBuilder::new()
                        .stroke_color(Rgb111::WHITE)
                        .fill_color(Rgb111::WHITE)
                        .stroke_width(1)
                        .build(),
                )
                .draw(&mut *display)
                .unwrap();
                start_x += 1;
            }
        }

        /*
                Line::new(Point::new(0, 0), Point::new(175, 175))
            .into_styled(
                PrimitiveStyleBuilder::new()
                    .stroke_color(Rgb111::WHITE)
                    .fill_color(Rgb111::WHITE)
                    .stroke_width(1)
                    .build(),
            )
            .draw(&mut *display)
            .unwrap();
        for (i, &v) in vals.iter().enumerate() {
            let h = (v - min) * 100 / stride;
            println!("v: {}, h: {}", v, 175 - (v / 200) as i32);
            Line::new(Point::new(i as i32, 175), Point::new(i as i32, 175 - (v / 200) as i32))
                .into_styled(PrimitiveStyleBuilder::new().stroke_color(Rgb111::BLACK).build())
                .draw(&mut *display)
                .unwrap();
        } */

        buf.clear();
        //        let val = button.is_high();
        core::write!(&mut buf, "Vbus: {}mV\nCC pin: CC{}", voltage, cc).unwrap();
        core::write!(&mut buf, "\n{}", PowerDataObject::from_u32(0x0088c1f4)).unwrap();

        let mut pos = 40;
        for (i, &cap) in caps.iter().enumerate() {
            if cap != 0 {
                buf.clear();
                // println!("#{}: {}", i + 1, PowerDataObject::from_u32(cap));
                if i + 1 == 8 {
                    core::write!(&mut buf, ">#{} {}", i + 1, PowerDataObject::from_u32(cap)).unwrap();
                    Text::with_alignment(buf.as_str(), Point::new(5, pos), style_item_selected, Alignment::Left)
                        .draw(&mut *display)
                        .unwrap();
                } else {
                    core::write!(&mut buf, " #{} {}", i + 1, PowerDataObject::from_u32(cap)).unwrap();
                    Text::with_alignment(buf.as_str(), Point::new(5, pos), style_item, Alignment::Left)
                        .draw(&mut *display)
                        .unwrap();
                }
                pos += 14;
            }
        }

        buf.clear();
        core::write!(&mut buf, "Vbus: {}mV", voltage).unwrap();

        Text::with_alignment(buf.as_str(), Point::new(80, 150), style_red, Alignment::Center)
            .draw(&mut *display)
            .unwrap();

        display.update(&mut delay);

        // println!("val => {}, {}mV", val, voltage);
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("\n\n\n{}", info);

    loop {}
}
