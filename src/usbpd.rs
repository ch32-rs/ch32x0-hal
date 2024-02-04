use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, AtomicU8, Ordering};
use core::task::Poll;

use aligned::Aligned;
use ch32x0::ch32x035::adc::iofr1::W;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::Delay;
use embedded_hal::delay::DelayNs;
use qingke_rt::highcode;

use crate::delay::SystickDelay;
use crate::gpio::Pull;
use crate::pac::Interrupt;
use crate::usbpd::protocol::{
    AugmentedPowerDataObject, BatterySupplyPowerDataObject, FixedPowerDataObject, Header, Request,
};
use crate::{into_ref, pac, peripherals, println, Peripheral, PeripheralRef};

mod consts;
pub mod protocol;

const NEW_AW: AtomicWaker = AtomicWaker::new();
static TXEND_WAKER: AtomicWaker = NEW_AW;
static RX_WAKER: AtomicWaker = NEW_AW;

static mut PD_RX_BUF: Aligned<aligned::A4, [u8; 34]> = Aligned([0; 34]);
static mut PD_TX_BUF: Aligned<aligned::A4, [u8; 34]> = Aligned([0; 34]);
static mut PD_ACK_BUF: Aligned<aligned::A4, [u8; 2]> = Aligned([0; 2]);

static MSG_ID: AtomicU8 = AtomicU8::new(0);

// for bmc aux
const PD_SOP0: u8 = 0b01;
const PD_SOP1: u8 = 0b10; // Hard Reset
const PD_SOP2: u8 = 0b11; // Cable Reset

#[no_mangle]
#[highcode]
unsafe extern "C" fn USBPD() {
    let usbpd = &*pac::USBPD::PTR;

    let status = usbpd.status().read();

    // println!("IRQ {:08x}", status.bits());
    // 接收完成中断标志
    if usbpd.status().read().if_rx_act().bit_is_set() {
        usbpd.status().modify(|_, w| w.if_rx_act().set_bit()); // clear IF_RX_ACT

        if usbpd.status().read().bmc_aux().bits() == PD_SOP0 {
            let len = usbpd.bmc_byte_cnt().read().bits();

            // crate::println!("len = {}", len);
            if len >= 6 {
                // qingke::pfic::disable_interrupt(Interrupt::USBPD as u8);

                // If GOODCRC, do not answer and ignore this reception
                if len != 6 || PD_RX_BUF[0] & 0x1F != consts::DEF_TYPE_GOODCRC {
                    // usbpd.config().modify(|_, w| w.ie_rx_act().clear_bit()); // as flag

                    SystickDelay.delay_us(25); // delay 30us to send GoodCRC // max 195us
                                               //crate::delay::SystickDelay.delay_ms(30); // delay 30us to send GoodCRC // max 195us

                    PD_ACK_BUF[0] = 0x41; // 0b10_00001
                    PD_ACK_BUF[1] = PD_RX_BUF[1] & 0x0E;

                    //let header = protocol::Header(u16::from_le_bytes([PD_ACK_BUF[0], PD_ACK_BUF[1]]));
                    //println!("header: {:?}", header);

                    usbpd.config().modify(|_, w| w.ie_tx_end().set_bit()); // enable tx_end irq
                    UsbPdSink::<'static, peripherals::USBPD>::send_phy(consts::UPD_SOP0, &PD_ACK_BUF[..2]);
                    //println!("send good crc");

                    // RX_WAKER.wake();
                    // GoodCRC is handled in receive_raw
                }
            }
        }
    }

    // sending done
    if usbpd.status().read().if_tx_end().bit_is_set() {
        usbpd.port_cc1().modify(|_, w| w.cc_lve().clear_bit());
        usbpd.port_cc2().modify(|_, w| w.cc_lve().clear_bit());

        qingke::pfic::disable_interrupt(Interrupt::USBPD as u8);
        //  println!("tx end");

        usbpd.status().modify(|_, w| w.if_tx_end().set_bit()); // clear IF_TX_END
        usbpd.config().modify(|_, w| w.ie_tx_end().clear_bit()); // set flag, triggered

        TXEND_WAKER.wake();
        RX_WAKER.wake();
    }

    if usbpd.status().read().if_rx_reset().bit_is_set() {
        usbpd.status().modify(|_, w| w.if_rx_reset().set_bit()); // clear IF_RX_RESET

        // reset
        // usbpd.set_rx_mode();

        // TODO: handle reset
        crate::println!("TODO: reset");
    }
    //  if status.buf_err().bit_is_set() {
    //     usbpd.status().modify(|_, w| w.buf_err().set_bit()); // clear BUF_ERR
    // }

    //  usbpd.status().modify(|r, w| unsafe { w.bits(r.bits()) });
}

pub struct UsbPdSink<'d, T: Instance> {
    _marker: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> UsbPdSink<'d, T> {
    /// Create a new SPI driver.
    pub fn new(
        _peri: impl Peripheral<P = T> + 'd,
        cc1: impl Peripheral<P = impl Cc1Pin<T>> + 'd,
        cc2: impl Peripheral<P = impl Cc2Pin<T>> + 'd,
    ) -> Self {
        into_ref!(cc1, cc2);

        let afio = unsafe { &*pac::AFIO::PTR };

        T::enable_and_reset();

        cc1.set_as_input(Pull::None);
        cc2.set_as_input(Pull::None);

        // PD 引脚 PC14/PC15 高阈值输入模式
        // PD 收发器 PHY 上拉限幅配置位: USBPD_PHY_V33
        afio.ctlr()
            .modify(|_, w| w.usbpd_in_hvt().set_bit().usbpd_phy_v33().set_bit());

        T::regs().config().write(|w| w.pd_dma_en().set_bit());
        T::regs().status().write(|w| {
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

        // pd_phy_reset
        T::regs().port_cc1().write(|w| w.cc_ce().v0_66());
        T::regs().port_cc2().write(|w| w.cc_ce().v0_66());

        let mut this = Self { _marker: PhantomData };
        // this.set_rx_mode();
        this
    }

    fn set_rx_mode(&mut self) {
        let usbpd = T::regs();

        usbpd.config().modify(|_, w| w.pd_all_clr().set_bit());
        usbpd.config().modify(|_, w| w.pd_all_clr().clear_bit());
        // usbpd
        //    .config()
        //    .modify(|_, w| w.ie_rx_act().set_bit().ie_rx_reset().set_bit().pd_dma_en().set_bit());

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

        // unsafe { qingke::pfic::enable_interrupt(Interrupt::USBPD as u8) };
    }

    // 0, 1, 2
    pub fn detect_cc(&mut self) -> u8 {
        let usbpd = T::regs();

        usbpd.port_cc1().modify(|_, w| w);
        usbpd.port_cc2().modify(|_, w| w);

        usbpd.port_cc1().modify(|_, w| w.cc_ce().v0_22());
        SystickDelay.delay_us(2);
        // test if > 0.22V
        if usbpd.port_cc1().read().pa_cc_ai().bit_is_set() {
            // cc1 is connected
            usbpd.config().modify(|_, w| w.cc_sel().cc1());
            return 1;
        } else {
            usbpd.port_cc2().modify(|_, w| w.cc_ce().v0_22());
            SystickDelay.delay_us(2);
            if usbpd.port_cc2().read().pa_cc_ai().bit_is_set() {
                // cc2 is connected
                usbpd.config().modify(|_, w| w.cc_sel().cc2());
                return 2;
            } else {
                // no cc is connected
                return 0;
            }
        }
    }

    pub async fn receive_initial(&mut self) {
        let raw = self.receive_raw().await;

        let header = Header(u16::from_le_bytes([raw[0], raw[1]]));
        println!("header: {:?}", header);

        if header.msg_type() == consts::DEF_TYPE_SRC_CAP as _ {
            println!("Source Capabilities:");
            let nobj = header.num_data_objs();
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
                } else if power_data >> 30 == 0b01 {
                    let power_data = BatterySupplyPowerDataObject(power_data);
                    println!(
                        "  Battery Supply: {}mV-{}mV {}mW",
                        power_data.min_voltage_50mv() * 50,
                        power_data.max_voltage_50mv() * 50,
                        power_data.max_power_250mw() * 250
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
                    println!("unknown => {}", power_data);
                }
            }
        }
    }

    pub async fn request_pdo(&mut self) {
        let mut header = Header(0);
        header.set_msg_type(consts::DEF_TYPE_REQUEST as _);
        header.set_spec_rev(0b10);
        header.set_num_data_objs(1);
        header.set_ext(false);
        header.set_power_role(false);
        header.set_data_role(false); // PD role
        header.set_msg_id(0);

        // 0002d12c
        let mut req = Request(0x0);
        req.set_usb_comm_capable(false);
        req.set_max_operating_current_10ma(100);
        req.set_operating_current_10ma(100);
        req.set_epr_mode(false);
        req.set_unchunked_extended_message_support(false);
        req.set_capability_mismatch(false);
        req.set_no_usb_suspend(false);
        // 2 9V
        // 3 12V
        // 4 15V
        req.set_position(5);

        for _ in 0..2 {
            unsafe {
                PD_TX_BUF[0..2].clone_from_slice(&u16::to_le_bytes(header.0));
                PD_TX_BUF[2..6].clone_from_slice(&u32::to_le_bytes(req.0));
            }

            let usbpd = T::regs();

            usbpd
                .config()
                .modify(|_, w| w.ie_tx_end().set_bit().ie_rx_act().clear_bit()); // enable tx_end irq
            unsafe { qingke::pfic::disable_interrupt(Interrupt::USBPD as u8) };
            unsafe {
                self.blocking_send(consts::UPD_SOP0, &PD_TX_BUF[..6]);
            }
            // recv timeout 750us

            let raw = self.receive_raw().await;

            let header = Header(u16::from_le_bytes([raw[0], raw[1]]));
            println!("header: {:?}", header);
            println!("raw: {:?}", raw);
        }
    }

    #[inline]
    async fn wait_for_tx_end(&mut self) {
        compiler_fence(Ordering::SeqCst);

        // wait for tx_end
        poll_fn(|cx| {
            TXEND_WAKER.register(cx.waker());

            if T::regs().config().read().ie_tx_end().bit_is_clear() {
                // usbpd.status().modify(|_, w| w.if_tx_end().set_bit()); // clear IF_TX_END
                return Poll::Ready(());
            }
            Poll::Pending
        })
        .await;

        compiler_fence(Ordering::SeqCst);
    }

    pub async fn receive_raw(&mut self) -> &'static [u8] {
        let usbpd = T::regs();

        unsafe {
            PD_RX_BUF.fill(0);
        }

        self.set_rx_mode();
        usbpd
            .config()
            .modify(|_, w| w.ie_rx_act().set_bit().ie_rx_reset().set_bit().ie_tx_end().set_bit());
        unsafe { qingke::pfic::enable_interrupt(Interrupt::USBPD as u8) };

        compiler_fence(Ordering::SeqCst);

        poll_fn(|cx| {
            RX_WAKER.register(cx.waker());

            if usbpd.config().read().ie_tx_end().bit_is_clear() {
                return Poll::Ready(());
            }
            Poll::Pending
        })
        .await;

        // let len = usbpd.bmc_byte_cnt().read().bits() as usize;

        /*
            // check if need to send GoodCRC
            crate::delay::SystickDelay.delay_us(20);
            unsafe {
                PD_TX_BUF[0] = 0x41; // 0b10_00001
                PD_TX_BUF[1] = PD_RX_BUF[1] & 0x0E;
            }

            usbpd
                .config()
                .modify(|_, w| w.ie_tx_end().set_bit().ie_rx_act().clear_bit()); // enable tx_end irq
            unsafe { qingke::pfic::enable_interrupt(Interrupt::USBPD as u8) };
            unsafe {
                Self::send_phy(consts::UPD_SOP0, &PD_TX_BUF[..2]);
            }
            compiler_fence(Ordering::SeqCst);

            // wait for tx_end
            poll_fn(|cx| {
                TXEND_WAKER.register(cx.waker());

                if usbpd.config().read().ie_tx_end().bit_is_clear() {
                    return Poll::Ready(());
                }
                Poll::Pending
            })
            .await;
        */

        //self.set_rx_mode(); 之后,  BUF 会改变
        // println!("good crc send");

        compiler_fence(Ordering::SeqCst);

        let header = unsafe { Header(u16::from_le_bytes([PD_RX_BUF[0], PD_RX_BUF[1]])) };
        let len = 2 + header.num_data_objs() * 4;
        unsafe { &PD_RX_BUF[..len as usize] }
    }

    pub async fn send(&mut self) {
        let usbpd = T::regs();

        compiler_fence(Ordering::SeqCst);

        poll_fn(|cx| {
            TXEND_WAKER.register(cx.waker());

            if usbpd.status().read().if_tx_end().bit_is_set() {
                usbpd.status().modify(|_, w| w.if_tx_end().set_bit()); // clear IF_TX_END

                unsafe { qingke::pfic::enable_interrupt(Interrupt::USBPD as u8) };

                return Poll::Ready(());
            }
            Poll::Pending
        })
        .await;

        compiler_fence(Ordering::SeqCst);
    }

    /// Do USB PD hard reset. If the MCU is powering from VBUS, it will be powered off.
    pub unsafe fn hard_reset(&mut self) {
        const TX_SEL_HARD_RESET: u8 = 0b10_10_10_01;
        self.send_phy_empty_playload(TX_SEL_HARD_RESET);

        // pd_phy_reset
        T::regs().port_cc1().write(|w| w.cc_ce().v0_66());
        T::regs().port_cc2().write(|w| w.cc_ce().v0_66());
    }

    fn blocking_send(&mut self, sop: u8, buf: &[u8]) {
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

        // usbpd.status().modify(|_, w| unsafe { w.bmc_aux().bits(0) }); // clear bmc aux status
        usbpd.control().modify(|_, w| w.pd_tx_en().set_bit()); // tx
        usbpd.status().write(|w| unsafe { w.bits(0) });

        usbpd.control().modify(|_, w| w.bmc_start().set_bit());

        // await
        while usbpd.status().read().if_tx_end().bit_is_clear() {}
        usbpd.status().modify(|_, w| w.if_tx_end().set_bit()); // clear IF_TX_END

        if usbpd.config().read().cc_sel().is_cc1() {
            usbpd.port_cc1().modify(|_, w| w.cc_lve().clear_bit());
        } else {
            usbpd.port_cc2().modify(|_, w| w.cc_lve().clear_bit());
        }

        self.set_rx_mode();
    }

    fn send_phy(sop: u8, buf: &[u8]) {
        let usbpd = unsafe { &*pac::USBPD::PTR };

        if usbpd.config().read().cc_sel().is_cc1() {
            usbpd.port_cc1().modify(|_, w| w.cc_lve().set_bit());
        } else {
            usbpd.port_cc2().modify(|_, w| w.cc_lve().set_bit());
        }

        const UPD_TMR_TX_48M: u16 = 80 - 1;
        usbpd.bmc_clk_cnt().write(|w| w.bmc_clk_cnt().variant(UPD_TMR_TX_48M));

        let dma_addr = (buf.as_ptr() as u32 & 0xFFFF) as u16;

        usbpd.dma().write(|w| unsafe { w.bits(dma_addr) });
        usbpd.tx_sel().write(|w| unsafe { w.bits(sop) });

        usbpd
            .bmc_tx_sz()
            .write(|w| unsafe { w.bmc_tx_sz().bits(buf.len() as _) });

        // usbpd.status().modify(|_, w| unsafe { w.bmc_aux().bits(0) }); // clear bmc aux status
        usbpd.control().modify(|_, w| w.pd_tx_en().set_bit()); // tx
        usbpd.status().write(|w| unsafe { w.bits(0) }); // clear bmc aux status
        println!("after clean {}", usbpd.status().read().bits()); // need sleep here

        usbpd.control().modify(|_, w| w.bmc_start().set_bit());

        // usbpd.config().modify(|_, w| w.ie_tx_end().set_bit()); // enable tx_end irq
    }

    fn send_phy_empty_playload(&mut self, sop: u8) {
        let usbpd = T::regs();

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
    }
}

pub(crate) mod sealed {
    use super::*;

    pub trait Instance {
        #[inline(always)]
        fn regs() -> &'static pac::usbpd::RegisterBlock {
            unsafe { &*pac::USBPD::PTR }
        }

        fn enable_and_reset() {
            let rcc = unsafe { &*pac::RCC::ptr() };
            rcc.ahbpcenr().modify(|_, w| w.usbpd().set_bit());
            rcc.ahbrstr().modify(|_, w| w.usbpdrst().set_bit());
            rcc.ahbrstr().modify(|_, w| w.usbpdrst().clear_bit());
        }
    }
}

pub trait Instance: Peripheral<P = Self> + sealed::Instance {}

impl sealed::Instance for peripherals::USBPD {}
impl Instance for peripherals::USBPD {}

pub trait Cc1Pin<T: Instance>: crate::gpio::Pin {}
pub trait Cc2Pin<T: Instance>: crate::gpio::Pin {}

macro_rules! pin_trait_impl {
    (crate::$mod:ident::$trait:ident, $instance:ident, $pin:ident) => {
        impl crate::$mod::$trait<crate::peripherals::$instance> for crate::peripherals::$pin {}
    };
}

pin_trait_impl!(crate::usbpd::Cc1Pin, USBPD, PC14);
pin_trait_impl!(crate::usbpd::Cc2Pin, USBPD, PC15);
