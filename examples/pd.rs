//! USB PD Sink
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use aligned::Aligned;
use bitfield::bitfield;
use ch32x0::ch32x035::Interrupt;
use ch32x0_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal::delay::DelayNs;
use hal::exti::ExtiInput;
use hal::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use hal::{pac, peripherals, println};

pub mod defs {
    // Control Message Types
    // Control Messages are short and manage the Message flow between Port Partners or for exchanging Messages that require no additional data.

    /// Send By: Source,Sink,Cable Plug
    pub const DEF_TYPE_GOODCRC: u8 = 0x01;
    /// Send By: Source
    pub const DEF_TYPE_GOTOMIN: u8 = 0x02;
    /// Send By: Source,Sink,Cable Plug
    pub const DEF_TYPE_ACCEPT: u8 = 0x03;
    /// Send By: Source,Sink,Cable Plug
    pub const DEF_TYPE_REJECT: u8 = 0x04;
    /// Send By: Source
    pub const DEF_TYPE_PING: u8 = 0x05;
    /// Send By: Source,Sink
    pub const DEF_TYPE_PS_RDY: u8 = 0x06;
    /// Send By: Sink,DRP
    pub const DEF_TYPE_GET_SRC_CAP: u8 = 0x07;
    /// Send By: Source,DRP
    pub const DEF_TYPE_GET_SNK_CAP: u8 = 0x08;
    /// Send By: Source,Sink
    pub const DEF_TYPE_DR_SWAP: u8 = 0x09;
    /// Send By: Source,Sink
    pub const DEF_TYPE_PR_SWAP: u8 = 0x0A;
    /// Send By: Source,Sink
    pub const DEF_TYPE_VCONN_SWAP: u8 = 0x0B;
    /// Send By: Source,Sink
    pub const DEF_TYPE_WAIT: u8 = 0x0C;
    /// Send By: Source,Sink
    pub const DEF_TYPE_SOFT_RESET: u8 = 0x0D;
    /// Send By: Source,Sink
    pub const DEF_TYPE_DATA_RESET: u8 = 0x0E;
    /// Send By: Source,Sink
    pub const DEF_TYPE_DATA_RESET_CMP: u8 = 0x0F;
    /// Send By: Source,Sink,Cable Plug
    pub const DEF_TYPE_NOT_SUPPORT: u8 = 0x10;
    /// Send By: Sink,DRP
    pub const DEF_TYPE_GET_SRC_CAP_EX: u8 = 0x11;
    /// Send By: Source,Sink
    pub const DEF_TYPE_GET_STATUS: u8 = 0x12;
    /// ext=1
    pub const DEF_TYPE_GET_STATUS_R: u8 = 0x02;
    /// Send By: Sink
    pub const DEF_TYPE_FR_SWAP: u8 = 0x13;
    /// Send By: Sink
    pub const DEF_TYPE_GET_PPS_STATUS: u8 = 0x14;
    /// Send By: Source,Sink
    pub const DEF_TYPE_GET_CTY_CODES: u8 = 0x15;
    /// Send By: Source,DRP
    pub const DEF_TYPE_GET_SNK_CAP_EX: u8 = 0x16;
    /// Send By: Sink,DRP
    pub const DEF_TYPE_GET_SRC_INFO: u8 = 0x17;
    /// Send By: Source,Sink
    pub const DEF_TYPE_GET_REVISION: u8 = 0x18;

    // Data Message Types
    // Data Messages exchange information between a pair of Port Partners. Data messages include exposing capabilities and negotiating power, Built-In-Self-Test (BIST), and custom messaging defined by the OEM.

    /// Send By: Source,Dual-Role Power
    pub const DEF_TYPE_SRC_CAP: u8 = 0x01;
    /// Send By: Sink
    pub const DEF_TYPE_REQUEST: u8 = 0x02;
    /// Send By: Tester,Source,Sink
    pub const DEF_TYPE_BIST: u8 = 0x03;
    /// Send By: Sink,Dual-Role Power
    pub const DEF_TYPE_SNK_CAP: u8 = 0x04;
    /// Send By: Source,Sink
    pub const DEF_TYPE_BAT_STATUS: u8 = 0x05;
    /// Send By: Source,Sink
    pub const DEF_TYPE_ALERT: u8 = 0x06;
    /// Send By: Source,Sink
    pub const DEF_TYPE_GET_CTY_INFO: u8 = 0x07;
    /// Send By: DFP
    pub const DEF_TYPE_ENTER_USB: u8 = 0x08;
    /// Send By: Sink
    pub const DEF_TYPE_EPR_REQUEST: u8 = 0x09;
    /// Send By: Source,Sink
    pub const DEF_TYPE_EPR_MODE: u8 = 0x0A;
    /// Send By: Source
    pub const DEF_TYPE_SRC_INFO: u8 = 0x0B;
    /// Send By: Source,Sink,Cable Plug
    pub const DEF_TYPE_REVISION: u8 = 0x0C;
    /// Send By: Source,Sink,Cable Plug
    pub const DEF_TYPE_VENDOR_DEFINED: u8 = 0x0F;

    // Vendor Define Message Command
    pub const DEF_VDM_DISC_IDENT: u8 = 0x01;
    pub const DEF_VDM_DISC_SVID: u8 = 0x02;
    pub const DEF_VDM_DISC_MODE: u8 = 0x03;
    pub const DEF_VDM_ENTER_MODE: u8 = 0x04;
    pub const DEF_VDM_EXIT_MODE: u8 = 0x05;
    pub const DEF_VDM_ATTENTION: u8 = 0x06;
    pub const DEF_VDM_DP_S_UPDATE: u8 = 0x10;
    pub const DEF_VDM_DP_CONFIG: u8 = 0x11;
}

const PD_SOP0: u8 = 0b01;
const PD_SOP1: u8 = 0b10; // Hard Reset
const PD_SOP2: u8 = 0b11; // Cable Reset

bitfield! {
    pub struct Header(u16);
    impl Debug;
    u16;
    pub msg_type, set_msg_type : 4, 0;
    /// Port Data Role
    pub data_role, set_data_role : 5;
    pub spec_rev, set_spec_rev : 7, 6;
    /// Port Power Role
    pub power_role, set_power_role : 8;
    pub msg_id, set_msg_id : 11, 9;
    pub num_data_objs, set_num_data_objs : 14, 12;
    pub ext, set_ext : 15;
}

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

static mut PD_RX_BUF: Aligned<aligned::A4, [u8; 34]> = Aligned([0; 34]);
static mut PD_TX_BUF: Aligned<aligned::A4, [u8; 34]> = Aligned([0; 34]);

pub fn parse_package(raw: &[u8]) -> Option<(Header, &[u32])> {
    todo!();
}

bitfield! {
    pub struct FixedPowerDataObject(u32);
    impl Debug;
    u32;

   // pub dual_role_power, _ : 29;
   // pub usb_suspend_supp, _ : 28;
   // pub unconstr_power, _ : 27;
  //  pub cap_usb_comm, _ : 26;
  //  pub dual_role_data, _ : 25;
  //  pub unchunked_extended_message_support, _ : 24;
    pub epr_mode_capable, _ : 23;
   // pub peak_current, _ : 21, 20;
    pub voltage_50mv, _ : 19, 10;
    pub max_current_10ma, _ : 9, 0;
}

bitfield! {
    pub struct AugmentedPowerDataObject(u32);
    impl Debug;
    u32;
    // 0b11
    // 0b00

    pub max_voltage_100mv, _ : 24, 17;
    pub min_voltage_100mv, _ : 15, 8;
    pub max_current_50ma, _ : 6, 0;


}

bitfield! {
    pub struct Request(u32);
    impl Debug;
    u32;
    /// Object Position
    pub positioin, set_position : 31, 28;
    pub give_back, set_give_back : 27;
    pub capability_mismatch, set_capability_mismatch : 26;
    pub usb_comm_capable, set_usb_comm_capable : 25;
    pub no_usb_suspend, set_no_usb_suspend : 24;
    pub unchunked_extended_message_support, set_unchunked_extended_message_support : 23;
    pub epr_mode, set_epr_mode : 22;
    pub operating_current_10ma, set_operating_current_10ma : 19, 10;
    pub max_operating_current_10ma, set_max_operating_current_10ma : 9, 0;
}

pub fn print_src_cap(raw: &[u8]) {
    if raw[0] & 0x1f == defs::DEF_TYPE_SRC_CAP {
        println!("Source Capabilities:");
        let nobj = (raw[1] & 0x70) >> 4;
        for i in 0..nobj {
            let power_data = u32::from_le_bytes([
                raw[2 + i as usize * 4],
                raw[2 + i as usize * 4 + 1],
                raw[2 + i as usize * 4 + 2],
                raw[2 + i as usize * 4 + 3],
            ]);
            println!("=> {:08x}", power_data);
            if power_data >> 30 == 0b00 {
                let power_data = FixedPowerDataObject(power_data);
                println!(
                    "  Fixed Supply: {}mV {}mA, EPR: {}",
                    power_data.voltage_50mv() * 50,
                    power_data.max_current_10ma() * 10,
                    power_data.epr_mode_capable()
                );
            } else if power_data >> 28 == 0b1100 {
                // Augmented Power Data Object (APDO)
                // SPR Programmable Power Supply
                let power_data = AugmentedPowerDataObject(power_data);
                println!(
                    "  SPR PPS Augmented Supply: {}mV-{}mV {}mA",
                    power_data.min_voltage_100mv() * 100,
                    power_data.max_voltage_100mv() * 100,
                    power_data.max_current_50ma() * 50
                );
            } else {
                println!("fuck => {}", power_data);
            }
        }
    }
}

#[no_mangle]
unsafe extern "C" fn USBPD() {
    let usbpd = &*pac::USBPD::PTR;

    println!("in USBPD irq");

    let status = usbpd.status().read();

    // println!("status 0x{:02x}", status.bits());
    // 接收完成中断标志
    if status.if_rx_act().bit_is_set() {
        usbpd.status().modify(|_, w| w.if_rx_act().set_bit()); // clear IF

        if status.bmc_aux().bits() == PD_SOP0 {
            let len = usbpd.bmc_byte_cnt().read().bits();
            if len >= 6 {
                // If GOODCRC, do not answer and ignore this reception
                if len != 6 || PD_RX_BUF[0] & 0x1F != defs::DEF_TYPE_GOODCRC {
                    Delay.delay_us(30); // delay 30us answer GoodCRC

                    PD_TX_BUF[0] = 0x41; // 0b10_00001
                    PD_TX_BUF[1] = PD_RX_BUF[1] & 0x0E;
                    usbpd.config().modify(|_, w| w.ie_tx_end().set_bit()); // enable tx_end irq

                    pd_phy_send_pack(&PD_TX_BUF[..2], PD_SOP0);
                }
                println!("=> {:02x?}", &PD_RX_BUF[..len as usize]);
                print_src_cap(&PD_RX_BUF[..len as usize]);
            }
        }
    }
    if status.if_tx_end().bit_is_set() {
        usbpd.port_cc1().modify(|_, w| w.cc_lve().clear_bit());
        usbpd.port_cc2().modify(|_, w| w.cc_lve().clear_bit());

        //  qingke::pfic::disable_interrupt(Interrupt::USBPD as u8);

        println!("tx end");
    }
    if status.buf_err().bit() {
        println!("buf err");
    }
    usbpd.status().modify(|r, w| unsafe { w.bits(r.bits()) });
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());
    hal::embassy::init();

    println!("embassy init ok");

    let input = Input::new(p.PA21, Pull::Up);
    let mut ei = ExtiInput::new(input, p.EXTI21);

    println!("exti init ok");

    let rcc = unsafe { &*pac::RCC::PTR };
    let afio = unsafe { &*pac::AFIO::PTR };
    let usbpd = unsafe { &*pac::USBPD::PTR };

    rcc.ahbpcenr().modify(|_, w| w.usbpd().set_bit()); // enable USBPD

    // PC14, PC15
    let cc1 = Input::new(p.PC14, Pull::None);
    let cc2 = Input::new(p.PC15, Pull::None);

    // PD 引脚 PC14/PC15 高阈值输入模式
    // PD 收发器 PHY 上拉限幅配置位: USBPD_PHY_V33
    afio.ctlr()
        .modify(|_, w| w.usbpd_in_hvt().set_bit().usbpd_phy_v33().set_bit());

    usbpd.config().write(|w| w.pd_dma_en().set_bit());

    usbpd.status().write(|w| {
        w.buf_err()
            .set_bit()
            .if_rx_bit()
            .set_bit()
            .if_rx_byte()
            .set_bit()
            .if_rx_act()
            .set_bit()
            .if_rx_reset()
            .set_bit()
            .if_tx_end()
            .set_bit()
    }); // write 1 to clear

    pd_phy_reset();

    unsafe { qingke::pfic::enable_interrupt(Interrupt::USBPD as u8) };

    pd_rx_mode();

    // init done

    /*
    // PD_PHY_reeset
    // - PD_SINK_Init
    // 0.66V

    usbpd.port_cc1().write(|w| w.cc_ce().v0_66());
    usbpd.port_cc2().write(|w| w.cc_ce().v0_66());

    // in pd rx mode

    usbpd.config().modify(|_, w| w.pd_all_clr().set_bit());
    usbpd.config().modify(|_, w| w.pd_all_clr().clear_bit());
    usbpd
        .config
        .modify(|_, w| w.ie_rx_act().set_bit().ie_rx_reset().set_bit().pd_dma_en().set_bit());

    usbpd
        .dma
        .write(|w| unsafe { w.bits(((PD_RX_BUF.as_ptr() as u32) & 0xFFFF) as u16) });

    usbpd.control().modify(|_, w| w.pd_tx_en().clear_bit());

    // #define UPD_TMR_TX_48M    (80-1)                                             /* timer value for USB PD BMC transmittal @Fsys=48MHz */
    // #define UPD_TMR_RX_48M    (120-1)                                            /* timer value for USB PD BMC receiving @Fsys=48MHz */
    // #define UPD_TMR_TX_24M    (40-1)                                             /* timer value for USB PD BMC transmittal @Fsys=24MHz */
    // #define UPD_TMR_RX_24M    (60-1)                                             /* timer value for USB PD BMC receiving @Fsys=24MHz */
    // #define UPD_TMR_TX_12M    (20-1)                                             /* timer value for USB PD BMC transmittal @Fsys=12MHz */
    // #define UPD_TMR_RX_12M    (30-1)                                             /* timer value for USB PD BMC receiving @Fsys=12MHz */
    usbpd.bmc_clk_cnt().write(|w| unsafe { w.bmc_clk_cnt().bits(120 - 1) });
    usbpd.control().modify(|_, w| w.bmc_start().set_bit());

    //    ei.wait_for_falling_edge().await;
    */

    println!("pd init  ok");

    // PD_Init ends

    // GPIO
    //  spawner.spawn(blink(p.PA4.degrade())).unwrap();
    let mut led = Output::new(p.PA4, Level::Low);

    // PD_Det_Proc
    // pull down, in SINK mode
    usbpd.port_cc1().modify(|_, w| w);
    usbpd.port_cc2().modify(|_, w| w);

    usbpd.port_cc1().modify(|_, w| w.cc_ce().v0_22());
    Delay.delay_us(2);
    // test if > 0.22V
    if usbpd.port_cc1().read().pa_cc_ai().bit_is_set() {
        // cc1 is connected
        println!("cc1 is connected");
        usbpd.config().modify(|_, w| w.cc_sel().cc1())
    } else {
        usbpd.port_cc2().modify(|_, w| w.cc_ce().v0_22());
        Delay.delay_us(2);
        if usbpd.port_cc2().read().pa_cc_ai().bit_is_set() {
            // cc2 is connected
            println!("cc2 is connected");
            usbpd.config().modify(|_, w| w.cc_sel().cc2())
        } else {
            // no cc is connected
            println!("no cc is connected");
        }
    }
    /*     println!(
           "cc1 pd {} cc2 pd {}",
           usbpd.port_cc1().read().cc_pd().bit(),
           usbpd.port_cc2().read().cc_pd().bit()
       );
    */
    // detect ok
    // PD_Main_Proc

    // STA_SRC_CONNECT
    pd_phy_reset();
    pd_rx_mode();

    let mut header = Header(0);
    header.set_msg_type(defs::DEF_TYPE_REQUEST as _);
    header.set_spec_rev(0b10);
    header.set_num_data_objs(1);
    header.set_ext(true);
    header.set_msg_id(2);

    println!("=> h => {:02x?}", &u16::to_le_bytes(header.0));

    let mut req = Request(0);
    req.set_position(2); // request 9V
    req.set_operating_current_10ma(30);
    req.set_max_operating_current_10ma(50);
    req.set_no_usb_suspend(true);

    println!("=> r => {:02x?}", &u32::to_le_bytes(req.0));

    unsafe {
        PD_TX_BUF[0..2].clone_from_slice(&u16::to_le_bytes(header.0));
        PD_TX_BUF[2..6].clone_from_slice(&u32::to_le_bytes(req.0));

        Timer::after(Duration::from_millis(5000)).await;
        //  pd_phy_send_pack(&PD_TX_BUF[..6], PD_SOP0);
        //        pd_rx_mode();
        const TX_SEL_HARD_RESET: u8 = 0b10_10_10_01;
        pd_phy_send_empty(TX_SEL_HARD_RESET);
    }

    loop {
        Timer::after(Duration::from_millis(5000)).await;
        println!("tick");

        pd_rx_mode();

        led.toggle();

        //        pd_phy_reset();
        //      pd_rx_mode();
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("\n\n\n{}", info);

    loop {}
}

fn pd_phy_reset() {
    let usbpd = unsafe { &*pac::USBPD::PTR };

    usbpd.port_cc1().write(|w| w.cc_ce().v0_66());
    usbpd.port_cc2().write(|w| w.cc_ce().v0_66());
}

// rx mode
fn pd_rx_mode() {
    let usbpd = unsafe { &*pac::USBPD::PTR };

    usbpd.config().modify(|_, w| w.pd_all_clr().set_bit());
    usbpd.config().modify(|_, w| w.pd_all_clr().clear_bit());
    usbpd
        .config()
        .modify(|_, w| w.ie_rx_act().set_bit().ie_rx_reset().set_bit().pd_dma_en().set_bit());

    //    println!("=> {:08x}", usbpd.config().read().bits());
    // println!("dma {:p}", unsafe { PD_RX_BUF.as_mut_ptr() });
    usbpd
        .dma()
        .write(|w| unsafe { w.bits(((PD_RX_BUF.as_mut_ptr() as u32) & 0xFFFF) as u16) });

    usbpd.control().modify(|_, w| w.pd_tx_en().clear_bit()); // rx_en

    // #define UPD_TMR_TX_48M    (80-1)                                             /* timer value for USB PD BMC transmittal @Fsys=48MHz */
    // #define UPD_TMR_RX_48M    (120-1)                                            /* timer value for USB PD BMC receiving @Fsys=48MHz */
    // #define UPD_TMR_TX_24M    (40-1)                                             /* timer value for USB PD BMC transmittal @Fsys=24MHz */
    // #define UPD_TMR_RX_24M    (60-1)                                             /* timer value for USB PD BMC receiving @Fsys=24MHz */
    // #define UPD_TMR_TX_12M    (20-1)                                             /* timer value for USB PD BMC transmittal @Fsys=12MHz */
    // #define UPD_TMR_RX_12M    (30-1)                                             /* timer value for USB PD BMC receiving @Fsys=12MHz */
    const UPD_TMR_RX_48M: u16 = 120 - 1;
    usbpd
        .bmc_clk_cnt()
        .write(|w| unsafe { w.bmc_clk_cnt().bits(UPD_TMR_RX_48M) });
    usbpd.control().modify(|_, w| w.bmc_start().set_bit());

    unsafe { qingke::pfic::enable_interrupt(Interrupt::USBPD as u8) };
}

fn pd_phy_send_pack(buf: &[u8], sop: u8) {
    let usbpd = unsafe { &*pac::USBPD::PTR };

    if usbpd.config().read().cc_sel().is_cc1() {
        usbpd.port_cc1().modify(|_, w| w.cc_lve().set_bit());
    } else {
        usbpd.port_cc2().modify(|_, w| w.cc_lve().set_bit());
    }
    const UPD_TMR_TX_48M: u16 = 80 - 1;

    usbpd.bmc_clk_cnt().write(|w| w.bmc_clk_cnt().variant(UPD_TMR_TX_48M));

    let dma_addr = buf.as_ptr() as u32 & 0xFFFF;

    usbpd.dma().write(|w| unsafe { w.bits(dma_addr as u16) });

    usbpd.tx_sel().write(|w| unsafe { w.bits(sop) });

    usbpd
        .bmc_tx_sz()
        .write(|w| unsafe { w.bmc_tx_sz().bits(buf.len() as _) });

    usbpd.control().modify(|_, w| w.pd_tx_en().set_bit()); // tx

    usbpd.status().modify(|_, w| unsafe { w.bmc_aux().bits(0) }); // clear bmc aux status

    // NO wait
}

fn pd_phy_blocking_send_pack(buf: &[u8], sop: u8) {
    let usbpd = unsafe { &*pac::USBPD::PTR };

    if usbpd.config().read().cc_sel().is_cc1() {
        usbpd.port_cc1().modify(|_, w| w.cc_lve().set_bit());
    } else {
        usbpd.port_cc2().modify(|_, w| w.cc_lve().set_bit());
    }
    const UPD_TMR_TX_48M: u16 = 80 - 1;

    usbpd.bmc_clk_cnt().write(|w| w.bmc_clk_cnt().variant(UPD_TMR_TX_48M));

    let dma_addr = buf.as_ptr() as u32 & 0xFFFF;

    usbpd.dma().write(|w| unsafe { w.bits(dma_addr as u16) });

    usbpd.tx_sel().write(|w| unsafe { w.bits(sop) });

    usbpd
        .bmc_tx_sz()
        .write(|w| unsafe { w.bmc_tx_sz().bits(buf.len() as _) });

    usbpd.control().modify(|_, w| w.pd_tx_en().set_bit()); // tx

    usbpd.status().modify(|_, w| unsafe { w.bmc_aux().bits(0) }); // clear bmc aux status

    // wait
}

pub fn pd_phy_send_empty(sop: u8) {
    let usbpd = unsafe { &*pac::USBPD::PTR };

    if usbpd.config().read().cc_sel().is_cc1() {
        usbpd.port_cc1().modify(|_, w| w.cc_lve().set_bit());
    } else {
        usbpd.port_cc2().modify(|_, w| w.cc_lve().set_bit());
    }
    const UPD_TMR_TX_48M: u16 = 80 - 1;

    usbpd.bmc_clk_cnt().write(|w| w.bmc_clk_cnt().variant(UPD_TMR_TX_48M));

    usbpd.dma().write(|w| unsafe { w.bits(0) });

    usbpd.tx_sel().write(|w| unsafe { w.bits(sop) });

    usbpd.bmc_tx_sz().write(|w| unsafe { w.bmc_tx_sz().bits(0) }); // len = 0
    usbpd.control().modify(|_, w| w.pd_tx_en().set_bit()); // tx mode

    usbpd.status().write(|w| unsafe { w.bits(0b11111100) }); // clear bmc aux status
                                                             // rb.config.modify(|_, w| w.pd_all_clr().set_bit());
                                                             //rb.config.modify(|_, w| w.pd_all_clr().clear_bit());

    usbpd.control().modify(|_, w| w.bmc_start().set_bit());

    usbpd.config().modify(|_, w| w.ie_tx_end().set_bit()); // enable tx_end irq
}
