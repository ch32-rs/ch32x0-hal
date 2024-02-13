use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, AtomicBool, AtomicU8, Ordering};
use core::task::Poll;

use aligned::Aligned;
use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal::delay::DelayNs;

use self::consts::{ExtendedControlType, ExtendedMessageType};
use self::protocol::{EPRModeDataObject, PPSRequest};
use crate::delay::SystickDelay;
use crate::gpio::Pull;
use crate::pac::Interrupt;
use crate::usbpd::protocol::{
    BatterySupplyPowerDataObject, ExtendedHeader, FixedPowerDataObject, FixedRequest, Header,
    VariableSupplyPowerDataObject, EPR_APDO, PPS_APDO,
};
use crate::{interrupt, into_ref, pac, peripherals, println, Peripheral};

pub mod consts;
pub mod protocol;
pub mod timing;

static mut PD_RX_BUF: Aligned<aligned::A4, [u8; 54]> = Aligned([0; 54]);
static mut PD_TX_BUF: Aligned<aligned::A4, [u8; 34]> = Aligned([0; 34]);
static mut PD_ACK_BUF: Aligned<aligned::A4, [u8; 2]> = Aligned([0; 2]);

// field def for bmc_aux
const PD_SOP0: u8 = 0b01;
const PD_SOP1: u8 = 0b10; // Hard Reset, 2
const PD_SOP2: u8 = 0b11; // Cable Reset

#[derive(Debug)]
pub enum Error {
    Rejected,
    Timeout,
    CCNotConnected,
    NotSupported,
    /// Unexpeted message type
    Protocol(u8),
    MaxRetry,
}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let usbpd = T::regs();

        if usbpd.status().read().if_rx_act().bit_is_set() {
            usbpd.status().modify(|_, w| w.if_rx_act().set_bit()); // clear IF_RX_ACT

            match usbpd.status().read().bmc_aux().bits() {
                PD_SOP0 => {
                    let len = usbpd.bmc_byte_cnt().read().bits();

                    if len >= 6 {
                        let header = Header(u16::from_le_bytes([PD_RX_BUF[0], PD_RX_BUF[1]]));
                        // 2 byte header + 4 byte CRC32
                        if len == 6 && header.msg_type() == consts::DEF_TYPE_GOODCRC {
                            usbpd.config().modify(|_, w| w.ie_rx_act().clear_bit());

                            qingke::pfic::disable_interrupt(Interrupt::USBPD as u8);
                            //                            T::state().msg_id.store(header.msg_id(), Ordering::Relaxed);
                            println!(">");
                            T::state().acked.store(true, Ordering::Relaxed);
                            T::state().ack_waker.wake(); // notifiy GoodCRC received
                        } else {
                            usbpd.config().modify(|_, w| w.ie_rx_act().clear_bit());

                            qingke::pfic::disable_interrupt(Interrupt::USBPD as u8);

                            T::state().rx_waker.wake(); // notifiy receive
                            println!("!");
                        }
                    }
                }
                PD_SOP1 => {
                    // hard reset
                    println!("!! hard reset");
                    T::state().msg_id.store(0, Ordering::Relaxed);
                }
                PD_SOP2 => {
                    // cable reset
                    println!("!! cable reset");
                }
                _ => (),
            }
        }

        // sending done
        if usbpd.status().read().if_tx_end().bit_is_set() {
            usbpd.port_cc1().modify(|_, w| w.cc_lve().clear_bit());
            usbpd.port_cc2().modify(|_, w| w.cc_lve().clear_bit());

            qingke::pfic::disable_interrupt(Interrupt::USBPD as u8);

            usbpd.status().modify(|_, w| w.if_tx_end().set_bit()); // clear IF_TX_END
            usbpd.config().modify(|_, w| w.ie_tx_end().clear_bit()); // set flag, triggered

            T::state().tx_waker.wake();
        }

        if usbpd.status().read().if_rx_reset().bit_is_set() {
            usbpd.status().modify(|_, w| w.if_rx_reset().set_bit()); // clear IF_RX_RESET

            // reset
            // usbpd.set_rx_mode();
            // TODO: handle reset
            crate::println!("TODO: reset");
            T::state().msg_id.store(0, Ordering::Relaxed);
        }
    }
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
        this.detect_cc();

        this
    }

    fn set_rx_mode(&mut self) {
        let usbpd = T::regs();

        usbpd.config().modify(|_, w| w.pd_all_clr().set_bit());
        usbpd.config().modify(|_, w| w.pd_all_clr().clear_bit());

        usbpd
            .dma()
            .write(|w| unsafe { w.bits(((PD_RX_BUF.as_mut_ptr() as u32) & 0xFFFF) as u16) });

        usbpd.control().modify(|_, w| w.pd_tx_en().clear_bit()); // rx_en

        usbpd
            .bmc_clk_cnt()
            .write(|w| unsafe { w.bmc_clk_cnt().bits(get_bmc_clk_for_rx()) });
        usbpd.control().modify(|_, w| w.bmc_start().set_bit());
    }

    // 0, 1, 2
    pub fn detect_cc(&mut self) -> u8 {
        let usbpd = T::regs();

        // CH32X035 has no internal CC pull down support
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

    // Dump Source Capabilities
    pub fn dump_pdo(&self, raw: &[u8]) {
        let header = Header(u16::from_le_bytes([raw[0], raw[1]]));

        if header.msg_type() == consts::DEF_TYPE_SRC_CAP as _ {
            println!("header: {:?}", header);

            println!("Source Capabilities:");
            let nobj = header.num_data_objs() as usize;
            self.dump_raw_pdo(&raw[2..nobj * 4]);
        }
    }

    /// Dump raw Power Data Objects
    pub fn dump_raw_pdo(&self, raw: &[u8]) {
        println!("Source Capabilities:");
        let nobj = raw.len() / 4;
        for i in 0..nobj {
            let power_data = u32::from_le_bytes([
                raw[i as usize * 4],
                raw[i as usize * 4 + 1],
                raw[i as usize * 4 + 2],
                raw[i as usize * 4 + 3],
            ]);
            println!("{} => 0x{:08x}", i + 1, power_data);
            if power_data == 0 {
                println!("  Unused");
            } else if power_data >> 30 == 0b00 {
                let fixed_pdo = FixedPowerDataObject(power_data);
                println!(
                    "  Fixed Supply: {}mV {}mA {}",
                    fixed_pdo.voltage_50mv() * 50,
                    fixed_pdo.max_current_10ma() * 10,
                    if fixed_pdo.epr_mode_capable() { "(EPR)" } else { "" },
                    // fixed_pdo.unchunked_extended_messages_supported(),
                );
            } else if power_data >> 30 == 0b01 {
                let bat_pdo = BatterySupplyPowerDataObject(power_data);
                println!(
                    "  Battery Supply: {}mV-{}mV {}mW",
                    bat_pdo.min_voltage_50mv() * 50,
                    bat_pdo.max_voltage_50mv() * 50,
                    bat_pdo.max_power_250mw() * 250
                );
            } else if power_data >> 30 == 0b10 {
                // Variable Supply (non-Battery)
                let var_pdo = VariableSupplyPowerDataObject(power_data);
                println!(
                    "  Variable Supply: {}mV-{}mV {}mA",
                    var_pdo.min_voltage_50mv() * 50,
                    var_pdo.max_voltage_50mv() * 50,
                    var_pdo.max_current_10ma() * 10
                );
            } else if power_data >> 28 == 0b1100 {
                // Augmented Power Data Object (APDO)
                // SPR Programmable Power Supply
                let pps_apdo = PPS_APDO(power_data);
                println!(
                    "  SPR PPS Augmented Supply: {}mV-{}mV {}mA",
                    pps_apdo.min_voltage_100mv() * 100,
                    pps_apdo.max_voltage_100mv() * 100,
                    pps_apdo.max_current_50ma() * 50
                );
            } else if power_data >> 28 == 0b1101 {
                // EPR Adjustable Voltage Supply
                let epr_apdo = EPR_APDO(power_data);
                println!(
                    "  EPR Adjustable Supply: {}mV-{}mV {}W",
                    epr_apdo.min_voltage_100mv() * 100,
                    epr_apdo.max_voltage_100mv() * 100,
                    epr_apdo.pdp_1w()
                );
            } else {
                println!("unknown => {}", power_data);
            }
        }
    }

    pub async fn receive_initial(&mut self) {
        let raw = self.receive_raw_packet(true).await;

        let header = Header(u16::from_le_bytes([raw[0], raw[1]]));
        println!("header: {:?}", header);

        self.dump_pdo(raw);
    }

    //    pub async fn get_

    /// EPR_Mode
    pub async fn enter_epr(&mut self) -> Result<(), Error> {
        let mut header = Header(0);
        header.set_msg_type(consts::DEF_TYPE_EPR_MODE);

        header.set_spec_rev(0b10);
        header.set_num_data_objs(1);
        header.set_extended(false);
        header.set_power_role(false);
        header.set_data_role(false); // PD role
        header.set_msg_id(T::state().msg_id.load(Ordering::Relaxed));

        let mut eprmdo = EPRModeDataObject(0);
        eprmdo.set_action(EPRModeDataObject::ACTION_ENTER);
        eprmdo.set_data(120); // 120W

        unsafe {
            PD_TX_BUF[0..2].clone_from_slice(&u16::to_le_bytes(header.0));
            PD_TX_BUF[2..6].clone_from_slice(&u32::to_le_bytes(eprmdo.0));

            self.send_raw_packet(consts::UPD_SOP0, &PD_TX_BUF[..6]).await;
        }

        let raw = self.receive_raw_packet(false).await;
        let header = Header(u16::from_le_bytes([raw[0], raw[1]]));
        let raw2 = self.receive_raw_packet(true).await; // recv as soon as possible
        let header2 = Header(u16::from_le_bytes([raw2[0], raw2[1]]));
        // FIXME: protocol is too fast to handle, actually raw and raw2 overlap
        if header.msg_type() == consts::DEF_TYPE_EPR_MODE && header2.msg_type() == consts::DEF_TYPE_EPR_MODE {
            let eprmdo = EPRModeDataObject(u32::from_le_bytes([raw[2], raw[3], raw[4], raw[5]]));
            if eprmdo.action() == EPRModeDataObject::ACTION_ENTER_SUCCEEDED {
                return Ok(());
            }
        }
        return Err(Error::Rejected);
    }

    // Handle EPR Src Capabilities
    pub async fn get_epr_src_cap(&mut self) -> Result<(), Error> {
        let mut buf: heapless::Vec<u8, 128> = Default::default();

        let raw = self.receive_raw_packet(true).await;
        // buf.extend_from_slice(&raw[4..]);
        let header = Header(u16::from_le_bytes([raw[0], raw[1]]));
        let ext_header = ExtendedHeader(u16::from_le_bytes([raw[2], raw[3]]));

        // 2 byte ext header, 2 byte padding
        buf.extend_from_slice(&raw[4..4 + header.num_data_objs() as usize * 4 - 2])
            .unwrap();
        // ref: PESinkWaitForHandleEPRChunk, PESinkHandleEPRChunk
        if header.extended() && ext_header.chunked() {
            let chunk_number = ext_header.chunk_number();

            // request next chunk
            let mut header = Header(0);
            header.set_spec_rev(0b10);
            header.set_num_data_objs(1);
            header.set_extended(true);
            header.set_msg_type(ExtendedMessageType::EPRSourceCapabilities as u8);
            header.set_msg_id(T::state().msg_id.load(Ordering::Relaxed));

            let mut ext_header = ExtendedHeader(0);
            ext_header.set_chunk_number(chunk_number + 1);
            ext_header.set_request_chunk(true);
            ext_header.set_chunked(true);

            unsafe {
                PD_TX_BUF[0..2].clone_from_slice(&u16::to_le_bytes(header.0));
                PD_TX_BUF[2..4].clone_from_slice(&u16::to_le_bytes(ext_header.0));
                PD_TX_BUF[4] = 0;
                PD_TX_BUF[5] = 0; // padding
                self.send_raw_packet(consts::UPD_SOP0, &PD_TX_BUF[..6]).await;
            }

            let raw = self.receive_raw_packet(true).await;

            let header = Header(u16::from_le_bytes([raw[0], raw[1]]));
            //let ext_header = ExtendedHeader(u16::from_le_bytes([raw[2], raw[3]]));

            buf.extend_from_slice(&raw[4..4 + header.num_data_objs() as usize * 4 - 2])
                .unwrap();

            // println!("headerXX: {:?}", header);
            //println!("ext_headerXX: {:?}", ext_header);
            //println!("raw => {:02x?}", raw);

            self.dump_raw_pdo(&buf);

            Ok(())
        } else {
            Err(Error::Protocol(header.msg_type()))
        }
    }

    // Not impled
    pub async fn epr_keep_alive(&mut self) -> Result<(), Error> {
        let mut header = Header(0);
        header.set_spec_rev(0b10);
        header.set_num_data_objs(1);
        header.set_extended(true);
        header.set_msg_type(ExtendedMessageType::ExtendedControl as u8);
        header.set_msg_id(T::state().msg_id.load(Ordering::Relaxed));

        let mut ext_header = ExtendedHeader(0);
        ext_header.set_chunked(false);
        ext_header.set_data_size(2);

        unsafe {
            PD_TX_BUF[0..2].clone_from_slice(&u16::to_le_bytes(header.0));
            PD_TX_BUF[2..4].clone_from_slice(&u16::to_le_bytes(ext_header.0));
            // ECDB
            PD_TX_BUF[4] = ExtendedControlType::EPR_KeepAlive as u8; // type
            PD_TX_BUF[5] = 0; // data
            self.send_raw_packet(consts::UPD_SOP0, &PD_TX_BUF[..6]).await;
        }

        let raw = self.receive_raw_packet(false).await;

        let header = Header(u16::from_le_bytes([raw[0], raw[1]]));
        // let ext_header = ExtendedHeader(u16::from_le_bytes([raw[2], raw[3]]));
        println!("rawXX: {:?}\n{:?}", raw, header);

        if header.extended()
            && header.msg_type() == ExtendedMessageType::ExtendedControl as u8
            && raw[4] == ExtendedControlType::EPR_KeepAlive_Ack as u8
        {
            Ok(())
        } else {
            Err(Error::Protocol(header.msg_type()))
        }
    }

    pub async fn request_epr_pdo(&mut self, index: u8, pdo: u32) -> Result<(), Error> {
        let mut header = Header(0);
        header.set_msg_type(consts::DEF_TYPE_EPR_REQUEST);
        header.set_num_data_objs(2); // 1 RDO 1 PDO copy
        header.set_spec_rev(0b10);
        header.set_msg_id(T::state().msg_id.load(Ordering::Relaxed));

        let mut rdo = FixedRequest(0);
        rdo.set_position(index);
        rdo.set_no_usb_suspend(false);
        rdo.set_epr_mode_capable(true);
        rdo.set_max_operating_current_10ma(200);
        rdo.set_operating_current_10ma(100);

        // let pdo: u32 = 0x0088c1f4;
        //let pdo = 0x0002d12c;
        unsafe {
            PD_TX_BUF[0..2].clone_from_slice(&u16::to_le_bytes(header.0));
            PD_TX_BUF[2..6].clone_from_slice(&u32::to_le_bytes(rdo.0));
            PD_TX_BUF[6..10].clone_from_slice(&u32::to_le_bytes(pdo));
            self.send_raw_packet(consts::UPD_SOP0, &PD_TX_BUF[..10]).await;
        }

        let raw = self.receive_raw_packet(false).await;
        let header0 = Header(u16::from_le_bytes([raw[0], raw[1]]));
        if header0.msg_type() == consts::DEF_TYPE_ACCEPT {
            // println!("Request Accepted");
        } else if header0.msg_type() == consts::DEF_TYPE_REJECT {
            return Err(Error::Rejected);
        } else if header0.msg_type() == consts::DEF_TYPE_WAIT {
            return Err(Error::Rejected); // TODO ? wait
        } else {
            return Err(Error::Protocol(header0.msg_type()));
        }

        let raw = self.receive_raw_packet(true).await;
        let header = Header(u16::from_le_bytes([raw[0], raw[1]]));

        if header.msg_type() == consts::DEF_TYPE_PS_RDY {
            Ok(())
        } else {
            Err(Error::Protocol(header.msg_type()))
        }
    }

    // not supported?
    pub async fn get_pps_status(&mut self) {
        let mut header = Header(0);
        header.set_msg_type(consts::DEF_TYPE_GET_PPS_STATUS);

        header.set_spec_rev(0b10);
        header.set_num_data_objs(0);
        header.set_extended(false);
        header.set_power_role(false);
        header.set_data_role(false); // PD role
        header.set_msg_id(T::state().msg_id.load(Ordering::Relaxed));

        unsafe {
            PD_TX_BUF[0..2].clone_from_slice(&u16::to_le_bytes(header.0));
        }

        unsafe {
            self.send_raw_packet(consts::UPD_SOP0, &PD_TX_BUF[..2]).await;
        }

        let raw = self.receive_raw_packet(false).await;

        let header0 = Header(u16::from_le_bytes([raw[0], raw[1]]));
        let extheader = ExtendedHeader(u16::from_le_bytes([raw[2], raw[3]]));
        println!("header get_pps status: {:?} {:?}", header, extheader);
        println!("raw 0: {:?}", raw);
    }

    pub async fn get_cap(&mut self) {
        let mut header = Header(0);
        // header.set_msg_type(consts::DEF_TYPE_GET_SRC_CAP as _);
        // header.set_msg_type(consts::DEF_TYPE_PING as _); => not supported
        // DEF_TYPE_GET_STATUS => not

        header.set_msg_type(consts::DEF_TYPE_GET_CTY_CODES);

        header.set_spec_rev(0b10);
        header.set_num_data_objs(0);
        header.set_extended(false);
        header.set_power_role(false);
        header.set_data_role(false); // PD role
        header.set_msg_id(T::state().msg_id.load(Ordering::Relaxed));

        unsafe {
            PD_TX_BUF[0..2].clone_from_slice(&u16::to_le_bytes(header.0));
        }

        let usbpd = T::regs();

        usbpd
            .config()
            .modify(|_, w| w.ie_tx_end().set_bit().ie_rx_act().clear_bit()); // enable tx_end irq
        unsafe { qingke::pfic::disable_interrupt(Interrupt::USBPD as u8) };
        unsafe {
            self.blocking_send(consts::UPD_SOP0, &PD_TX_BUF[..2]);
        }
        println!("send get cap");

        let raw = self.receive_raw_packet(true).await;

        let header = Header(u16::from_le_bytes([raw[0], raw[1]]));
        println!("header: {:?}", header);
        println!("raw: {:?}", raw);

        self.dump_pdo(&raw);

        let raw = self.receive_raw_packet(true).await;
        let header = Header(u16::from_le_bytes([raw[0], raw[1]]));
        println!("header: {:?}", header);
        println!("raw: {:?}", raw);
    }

    // 6
    pub async fn request_pps_pdo(&mut self, index: u8, mv: u32, ma: u32) -> Result<(), Error> {
        let mut header = Header(0);
        header.set_msg_type(consts::DEF_TYPE_REQUEST as _);
        header.set_spec_rev(0b10);
        header.set_num_data_objs(1);
        header.set_extended(false);
        header.set_power_role(false);
        header.set_data_role(false); // PD role

        // header.set_msg_id(T::state().msg_id.load(Ordering::Relaxed));

        let mut req = PPSRequest(0x0);
        req.set_usb_comm_capable(false);
        req.set_epr_mode_capable(true);
        req.set_unchunked_extended_message_support(false);
        req.set_capability_mismatch(false);
        req.set_no_usb_suspend(false);

        req.set_output_voltage_20mv((mv / 20).max(1) as _);
        req.set_operating_current_50ma((ma / 50).max(1) as _);
        req.set_position(index);

        self.request_pdo(header, req.0).await
    }

    /// 0 illegal
    /// 1 5V
    /// 2 9V
    /// 3 12V
    /// 4 15V
    /// 5 20V
    pub async fn request_fixed_pdo(&mut self, index: u8) -> Result<(), Error> {
        let mut header = Header(0);
        header.set_msg_type(consts::DEF_TYPE_REQUEST as _);
        header.set_spec_rev(0b10);
        header.set_num_data_objs(1);

        let mut req = FixedRequest(0x0);
        req.set_max_operating_current_10ma(100);
        req.set_operating_current_10ma(100);
        req.set_usb_comm_capable(false);
        req.set_epr_mode_capable(true);
        req.set_unchunked_extended_message_support(false);
        req.set_capability_mismatch(false);
        req.set_no_usb_suspend(false);

        req.set_position(index);

        self.request_pdo(header, req.0).await
    }

    // PDO, fixed or PPS
    async fn request_pdo(&mut self, mut header: Header, pdo: u32) -> Result<(), Error> {
        let mut retries = 0;
        loop {
            header.set_msg_id(T::state().msg_id.load(Ordering::Relaxed));

            unsafe {
                PD_TX_BUF[0..2].clone_from_slice(&u16::to_le_bytes(header.0));
                PD_TX_BUF[2..6].clone_from_slice(&u32::to_le_bytes(pdo));

                self.send_raw_packet(consts::UPD_SOP0, &PD_TX_BUF[..6]).await;
            }

            let raw = self.receive_raw_packet(false).await;

            let header0 = Header(u16::from_le_bytes([raw[0], raw[1]]));
            // println!("header0: {:?}", header);

            if header0.msg_type() == consts::DEF_TYPE_ACCEPT {
                // println!("Request Accepted");
                break;
            } else if header0.msg_type() == consts::DEF_TYPE_REJECT {
                return Err(Error::Rejected);
            } else {
                retries += 1;
                if retries > 3 {
                    return Err(Error::MaxRetry);
                }
                continue;
            }
        }

        let raw = self.receive_raw_packet(true).await;
        let header = Header(u16::from_le_bytes([raw[0], raw[1]]));
        //println!("header: {:?}", header);
        // println!("raw: {:?}", raw);

        if header.msg_id() as u8 != T::state().msg_id.load(Ordering::Relaxed) {
            T::state().msg_id.store(header.msg_id(), Ordering::Relaxed);
        }

        if header.msg_type() == consts::DEF_TYPE_PS_RDY {
            //           println!("PS_RDY");
            Ok(())
        } else {
            Err(Error::Protocol(header.msg_type()))
        }
    }

    /// Receive raw packet and handle GoodCRC ack, ends with tx mode
    // Header + Data + CRC32
    pub async fn receive_raw_packet(&mut self, set_mode: bool) -> &'static [u8] {
        unsafe { PD_RX_BUF.fill(0) };

        let usbpd = T::regs();

        if set_mode {
            self.set_rx_mode();
        }
        usbpd
            .config()
            .modify(|_, w| w.ie_rx_act().set_bit().ie_rx_reset().set_bit().ie_tx_end().clear_bit());
        unsafe { qingke::pfic::enable_interrupt(Interrupt::USBPD as u8) };
        compiler_fence(Ordering::SeqCst);

        poll_fn(|cx| {
            T::state().rx_waker.register(cx.waker());

            if usbpd.config().read().ie_rx_act().bit_is_clear() {
                return Poll::Ready(());
            }
            Poll::Pending
        })
        .await;

        let header = unsafe { Header(u16::from_le_bytes([PD_RX_BUF[0], PD_RX_BUF[1]])) };
        let len = usbpd.bmc_byte_cnt().read().bits();

        self.ack_goodcrc(header.msg_id()).await;
        // T::state().fetch_update_msg_id();
        // println!("ack {}", header.msg_id());
        T::state().msg_id.store(header.msg_id(), Ordering::Relaxed);

        unsafe { &PD_RX_BUF[..len as usize] }
    }

    /// Send raw packet, and wait for GoodCRC ack, ends with rx mode
    async fn send_raw_packet(&mut self, sop: u8, buf: &'static [u8]) {
        let usbpd = T::regs();

        self.blocking_send(sop, buf);

        // wait for googd crc
        usbpd.config().modify(|_, w| w.ie_rx_act().set_bit());
        T::state().acked.store(false, Ordering::Relaxed);
        self.set_rx_mode();
        unsafe { qingke::pfic::enable_interrupt(Interrupt::USBPD as u8) };
        compiler_fence(Ordering::SeqCst);
        // wait for GoodCRC ack
        poll_fn(|cx| {
            T::state().ack_waker.register(cx.waker());
            //println!("pool");
            //if usbpd.config().read().ie_rx_act().bit_is_clear() {
            if T::state().acked.load(Ordering::Relaxed) {
                return Poll::Ready(());
            }
            Poll::Pending
        })
        .await;

        let header = unsafe { Header(u16::from_le_bytes([PD_RX_BUF[0], PD_RX_BUF[1]])) };
        T::state().msg_id.store(header.msg_id(), Ordering::Relaxed);
    }

    /// Do USB PD hard reset. If the MCU is powering from VBUS, it will be powered off.
    pub unsafe fn hard_reset(&mut self) {
        const TX_SEL_HARD_RESET: u8 = 0b10_10_10_01;
        self.send_phy_empty_playload(TX_SEL_HARD_RESET);

        // pd_phy_reset
        T::regs().port_cc1().write(|w| w.cc_ce().v0_66());
        T::regs().port_cc2().write(|w| w.cc_ce().v0_66());
    }

    /// Blocking send packet
    fn blocking_send(&mut self, sop: u8, buf: &[u8]) {
        let usbpd = unsafe { &*pac::USBPD::PTR };

        unsafe { qingke::pfic::disable_interrupt(Interrupt::USBPD as u8) };
        usbpd
            .config()
            .modify(|_, w| w.ie_tx_end().set_bit().ie_rx_act().clear_bit()); // enable tx_end irq

        if usbpd.config().read().cc_sel().is_cc1() {
            usbpd.port_cc1().modify(|_, w| w.cc_lve().set_bit());
        } else {
            usbpd.port_cc2().modify(|_, w| w.cc_lve().set_bit());
        }

        usbpd
            .bmc_clk_cnt()
            .write(|w| w.bmc_clk_cnt().variant(get_bmc_clk_for_tx()));

        let dma_addr = buf.as_ptr() as u32 & 0xFFFF;

        usbpd.dma().write(|w| unsafe { w.bits(dma_addr as u16) });
        usbpd.tx_sel().write(|w| unsafe { w.bits(sop) });

        usbpd
            .bmc_tx_sz()
            .write(|w| unsafe { w.bmc_tx_sz().bits(buf.len() as _) });

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
    }

    fn send_phy(sop: u8, buf: &[u8]) {
        let usbpd = unsafe { &*pac::USBPD::PTR };

        if usbpd.config().read().cc_sel().is_cc1() {
            usbpd.port_cc1().modify(|_, w| w.cc_lve().set_bit());
        } else {
            usbpd.port_cc2().modify(|_, w| w.cc_lve().set_bit());
        }

        usbpd
            .bmc_clk_cnt()
            .write(|w| w.bmc_clk_cnt().variant(get_bmc_clk_for_tx()));

        let dma_addr = (buf.as_ptr() as u32 & 0xFFFF) as u16;

        usbpd.dma().write(|w| unsafe { w.bits(dma_addr) });
        usbpd.tx_sel().write(|w| unsafe { w.bits(sop) });

        usbpd
            .bmc_tx_sz()
            .write(|w| unsafe { w.bmc_tx_sz().bits(buf.len() as _) });

        usbpd.control().modify(|_, w| w.pd_tx_en().set_bit()); // tx
        usbpd.status().write(|w| unsafe { w.bits(0) }); // clear bmc aux status

        // println!("fuck why {}", usbpd.status().read().bits());
        SystickDelay.delay_us(300); // required for timing. no idea why

        usbpd.control().modify(|_, w| w.bmc_start().set_bit());
    }

    fn send_phy_empty_playload(&mut self, sop: u8) {
        let usbpd = T::regs();

        if usbpd.config().read().cc_sel().is_cc1() {
            usbpd.port_cc1().modify(|_, w| w.cc_lve().set_bit());
        } else {
            usbpd.port_cc2().modify(|_, w| w.cc_lve().set_bit());
        }

        usbpd
            .bmc_clk_cnt()
            .write(|w| w.bmc_clk_cnt().variant(get_bmc_clk_for_tx()));

        usbpd.dma().write(|w| unsafe { w.bits(0) });

        usbpd.tx_sel().write(|w| unsafe { w.bits(sop) });

        usbpd.bmc_tx_sz().write(|w| unsafe { w.bmc_tx_sz().bits(0) }); // len = 0
        usbpd.control().modify(|_, w| w.pd_tx_en().set_bit()); // tx mode

        usbpd.status().write(|w| unsafe { w.bits(0b11111100) }); // clear bmc aux status
                                                                 // rb.config.modify(|_, w| w.pd_all_clr().set_bit());
                                                                 //rb.config.modify(|_, w| w.pd_all_clr().clear_bit());

        usbpd.control().modify(|_, w| w.bmc_start().set_bit());
    }

    async fn ack_goodcrc(&mut self, msg_id: u8) {
        let usbpd = T::regs();

        let mut header = Header(0);
        header.set_msg_type(consts::DEF_TYPE_GOODCRC);
        header.set_data_role(false);
        header.set_power_role(false);
        header.set_spec_rev(0b10);
        header.set_msg_id(msg_id);
        // CRCGOOD handling
        unsafe {
            PD_ACK_BUF[..2].copy_from_slice(&u16::to_le_bytes(header.0));
        }

        usbpd
            .config()
            .modify(|_, w| w.ie_tx_end().set_bit().ie_rx_act().clear_bit()); // enable tx_end irq
        unsafe { qingke::pfic::enable_interrupt(Interrupt::USBPD as u8) };
        compiler_fence(Ordering::SeqCst);
        unsafe {
            Self::send_phy(consts::UPD_SOP0, &PD_ACK_BUF[..2]);
        }

        // wait for tx_end
        poll_fn(|cx| {
            T::state().tx_waker.register(cx.waker());

            if usbpd.config().read().ie_tx_end().bit_is_clear() {
                return Poll::Ready(());
            }
            Poll::Pending
        })
        .await;
    }
}

pub(crate) mod sealed {
    use super::*;

    pub struct State {
        pub rx_waker: AtomicWaker,
        // tx end
        pub tx_waker: AtomicWaker,
        /// ack received
        pub ack_waker: AtomicWaker,
        // sent and acked
        pub acked: AtomicBool,
        // 0 to 7 counter
        pub msg_id: AtomicU8,
    }

    impl State {
        pub const fn new() -> Self {
            Self {
                rx_waker: AtomicWaker::new(),
                tx_waker: AtomicWaker::new(),
                ack_waker: AtomicWaker::new(),
                msg_id: AtomicU8::new(0),
                acked: AtomicBool::new(false),
            }
        }

        pub fn fetch_update_msg_id(&self) -> u8 {
            self.msg_id
                .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |x| Some((x + 1) & 0b111))
                .unwrap()
        }
    }

    pub trait Instance {
        type Interrupt: interrupt::Interrupt;

        #[inline(always)]
        fn regs() -> &'static pac::usbpd::RegisterBlock {
            unsafe { &*pac::USBPD::PTR }
        }

        fn state() -> &'static State;

        fn enable_and_reset() {
            let rcc = unsafe { &*pac::RCC::ptr() };
            rcc.ahbpcenr().modify(|_, w| w.usbpd().set_bit());
            rcc.ahbrstr().modify(|_, w| w.usbpdrst().set_bit());
            rcc.ahbrstr().modify(|_, w| w.usbpdrst().clear_bit());
        }
    }
}

pub trait Instance: Peripheral<P = Self> + sealed::Instance {}

impl sealed::Instance for peripherals::USBPD {
    type Interrupt = interrupt::USBPD;

    fn state() -> &'static crate::usbpd::sealed::State {
        static STATE: crate::usbpd::sealed::State = crate::usbpd::sealed::State::new();
        &STATE
    }
}
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

/*
const UPD_TMR_TX_48M: u8 = (80 - 1); // timer value for USB PD BMC transmittal @Fsys=48MHz
const UPD_TMR_TX_24M: u8 = (40 - 1); // timer value for USB PD BMC transmittal @Fsys=24MHz
const UPD_TMR_TX_12M: u8 = (20 - 1); // timer value for USB PD BMC transmittal @Fsys=12MHz
const UPD_TMR_RX_48M: u8 = (120 - 1); // timer value for USB PD BMC receiving @Fsys=48MHz
const UPD_TMR_RX_24M: u8 = (60 - 1); // timer value for USB PD BMC receiving @Fsys=24MHz
const UPD_TMR_RX_12M: u8 = (30 - 1); // timer value for USB PD BMC receiving @Fsys=12MHz
*/

#[inline]
fn get_bmc_clk_for_tx() -> u16 {
    (crate::rcc::clocks().hclk.to_MHz() * 80 / 48 - 1) as u16
}

#[inline]
fn get_bmc_clk_for_rx() -> u16 {
    (crate::rcc::clocks().hclk.to_MHz() * 120 / 48 - 1) as u16
}
