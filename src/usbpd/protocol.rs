#![allow(non_camel_case_types)]
use bitfield::bitfield;

// Table 6.1 “Message Header”
bitfield! {
    pub struct Header(u16);
    impl Debug;
    u8;

    // 0 for Control Message or Data Message
    // 1 for Extended Message
    pub extended, set_extended : 15;
    // Number of data objects
    pub num_data_objs, set_num_data_objs : 14, 12;
    pub msg_id, set_msg_id : 11, 9;
    /// Port Power Role
    /// 0 for Sink
    /// 1 for Source
    pub power_role, set_power_role : 8;
    /// Specification Revision
    /// 00b –Revision 1.0
    /// 01b –Revision 2.0
    /// 10b – Revision 3.0
    pub spec_rev, set_spec_rev : 7, 6;
    /// Port Data Role
    pub data_role, set_data_role : 5;
    pub msg_type, set_msg_type : 4, 0;
}

// Table 6.3 “Extended Message Header”
bitfield! {
    pub struct ExtendedHeader(u16);
    impl Debug;
    u8;

    pub chunked, set_chunked : 15;
    pub chunk_number, set_chunk_number : 14, 11;
    pub request_chunk, set_request_chunk : 10;
    pub u16, data_size, set_data_size : 9, 0;
}

#[derive(Debug, Clone, Copy)]
pub enum PowerDataObject {
    FixedSupply(FixedSupplyPDO),
    VariableSupply(VariableSupplyPDO),
    BatterySupply(BatterySupplyPDO),
    PPS(PPS_APDO),
    AVS(AVS_APDO),
    // for supplying 15V up to 48V.
    EPR_AVS(EPR_AVS_APDO),
}

impl PowerDataObject {
    pub fn from_u32(raw: u32) -> Self {
        match raw >> 30 {
            0b00 => PowerDataObject::FixedSupply(FixedSupplyPDO(raw)),
            0b01 => PowerDataObject::BatterySupply(BatterySupplyPDO(raw)),
            0b10 => PowerDataObject::VariableSupply(VariableSupplyPDO(raw)),
            0b11 => match raw >> 28 {
                0b1100 => PowerDataObject::PPS(PPS_APDO(raw)),
                0b1101 => PowerDataObject::EPR_AVS(EPR_AVS_APDO(raw)),
                0b1110 => PowerDataObject::AVS(AVS_APDO(raw)),
                _ => PowerDataObject::FixedSupply(FixedSupplyPDO(raw)),
            },
            _ => PowerDataObject::FixedSupply(FixedSupplyPDO(raw)),
        }
    }
}

fn format_mv(f: &mut core::fmt::Formatter<'_>, mv: u32) -> core::fmt::Result {
    if mv % 1000 == 0 {
        write!(f, "{}V", mv / 1000)
    } else if mv % 100 == 0 {
        write!(f, "{}.{}V", mv / 1000, mv % 1000 / 100)
    } else {
        write!(f, "{}mV", mv)
    }
}
fn format_ma(f: &mut core::fmt::Formatter<'_>, ma: u32) -> core::fmt::Result {
    if ma % 1000 == 0 {
        write!(f, "{}A", ma / 1000)
    } else {
        write!(f, "{}mA", ma)
    }
}

fn format_mw(f: &mut core::fmt::Formatter<'_>, mw: u32) -> core::fmt::Result {
    if mw % 1000 == 0 {
        write!(f, "{}W", mw / 1000)
    } else {
        write!(f, "{}mW", mw)
    }
}

impl core::fmt::Display for PowerDataObject {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            PowerDataObject::FixedSupply(pdo) => {
                if pdo.0 == 0 {
                    write!(f, "Unused")
                } else {
                    write!(f, "Fixed(",)?;
                    format_mv(f, pdo.voltage_50mv() * 50)?;
                    write!(f, ",")?;
                    format_ma(f, pdo.max_current_10ma() * 10)?;

                    if pdo.epr_mode_capable() {
                        write!(f, ",EPR)")
                    } else {
                        write!(f, ")")
                    }
                }
            }
            PowerDataObject::VariableSupply(pdo) => {
                write!(f, "Variable(")?;
                format_mv(f, pdo.min_voltage_50mv() * 50)?;
                write!(f, "-")?;
                format_mv(f, pdo.max_voltage_50mv() * 50)?;
                write!(f, ",")?;
                format_ma(f, pdo.max_current_10ma() * 10)?;
                write!(f, ")")
            }
            PowerDataObject::BatterySupply(pdo) => {
                write!(f, "Battery(")?;
                format_mv(f, pdo.min_voltage_50mv() * 50)?;
                write!(f, "-")?;
                format_mv(f, pdo.max_voltage_50mv() * 50)?;
                write!(f, ",")?;
                format_mw(f, pdo.max_power_250mw() * 250)
            }
            PowerDataObject::PPS(pdo) => {
                write!(f, "PPS(")?;
                format_mv(f, pdo.min_voltage_100mv() * 100)?;
                write!(f, "-")?;
                format_mv(f, pdo.max_voltage_100mv() * 100)?;
                write!(f, ",")?;
                format_ma(f, pdo.max_current_50ma() * 50)?;
                write!(f, ")")
            }
            PowerDataObject::AVS(pdo) => {
                write!(f, "AVS({}mA @ 15V", pdo.max_current_10ma_15v() * 10)?;
                if pdo.max_current_10ma_20v() != 0 {
                    write!(f, ", {}mA @ 20V)", pdo.max_current_10ma_20v() * 10)
                } else {
                    write!(f, ")")
                }
            }
            PowerDataObject::EPR_AVS(pdo) => write!(
                f,
                "EPR_AVS({}mV-{}mV,{}W)",
                pdo.min_voltage_100mv() * 100,
                pdo.max_voltage_100mv() * 100,
                pdo.pdp_1w()
            ),
        }
    }
}

bitfield! {
    /// Table 6.9 “Fixed Supply PDO – Source”
    #[derive(Copy, Clone)]
    pub struct FixedSupplyPDO(u32);
    impl Debug;
    u32;

    // 0b00
    // supports PR_Swap message? equal in SrcCap, SinkCap
    pub dual_role_power, _ : 29;
    pub usb_suspend_supported, _ : 28;
    pub unconstrained_power, _ : 27;
    // only for D+/D- or SS Tx/Rx
    pub usb_communications_capable, _ : 26;
    // supports DR_Swap message?
    pub dual_role_data, _ : 25;
    // Set to 1 for unchunked extended messages
    pub unchunked_extended_messages_supported, _ : 24;
    // if the Source is designed to supply more than 100W and operate in EPR Mode.
    pub epr_mode_capable, _ : 23;
    // peak current overload capability
    pub peak_current, _ : 21, 20;
    pub voltage_50mv, _ : 19, 10;
    pub max_current_10ma, _ : 9, 0;
}

// Table 6.11 “Variable Supply (non-Battery) PDO – Source”
bitfield! {
    #[derive(Copy, Clone)]
    pub struct VariableSupplyPDO(u32);
    impl Debug;
    u32;

    // 0b10
    pub max_voltage_50mv, _ : 29, 20;
    pub min_voltage_50mv, _ : 19, 10;
    pub max_current_10ma, _ : 9, 0;
}

//  Battery PDO uses power instead of current.
bitfield! {
    /// Table 6.12 “Battery Supply PDO – Source”
    #[derive(Copy, Clone)]
    pub struct BatterySupplyPDO(u32);
    impl Debug;
    u32;

    // 0b01
    pub max_voltage_50mv, _ : 29, 20;
    pub min_voltage_50mv, _ : 19, 10;
    pub max_power_250mw, _ : 9, 0;
}

// Table 6.8 “Augmented Power Data Object”, APDO
bitfield! {
    /// Table 6.13 “SPR Programmable Power Supply APDO – Source”
    /// SPR PPS APDO
    #[derive(Copy, Clone)]
    pub struct PPS_APDO(u32);
    impl Debug;
    u32;
    // 0b11b00
    pub max_voltage_100mv, _ : 24, 17;
    pub min_voltage_100mv, _ : 15, 8;
    pub max_current_50ma, _ : 6, 0;
}

bitfield! {
    /// Table 6.14 “EPR Adjustable Voltage Supply APDO – Source “
    #[derive(Copy, Clone)]
    pub struct EPR_AVS_APDO(u32);
    impl Debug;
    u32;
    // 0b1101
    pub peak_current, _ : 27, 26;
    pub max_voltage_100mv, _ : 25, 17;
    pub min_voltage_100mv, _ : 15, 8;
    pub pdp_1w, _ : 7, 0;
}

bitfield! {
       /// Table 6.16 “SPR Adjustable Voltage Supply APDO – Source”
       /// 9V up to 20V
    #[derive(Copy, Clone)]
    pub struct AVS_APDO(u32);
    impl Debug;
    u32;
    // 0b1110
    pub peak_current, _ : 27, 26;
    pub max_current_10ma_15v, _ : 19, 10;
    pub max_current_10ma_20v, _ : 9, 0;
}

bitfield! {
    /// Table 6.51 “EPR Mode Data Object (EPRMDO)”
    pub struct EPRModeDataObject(u32);
    impl Debug;
    u8;

    pub action, set_action : 31, 24;
    // EPR Sink Operational PDP or Fail cause
    // PDP when action = ACTION_ENTER
    pub data, set_data: 23, 16;
}

impl EPRModeDataObject {
    // action field
    pub const ACTION_ENTER: u8 = 0x01;
    pub const ACTION_ENTER_ACK: u8 = 0x02;
    pub const ACTION_ENTER_SUCCEEDED: u8 = 0x03;
    pub const ACTION_ENTER_FAILED: u8 = 0x04;
    pub const ACTION_EXIT: u8 = 0x05;
}

// Sink Capabilities

// Request Data Object (RDO)

// Fixed and Variable Request Data Object
bitfield! {
    pub struct FixedRequest(u32);
    impl Debug;
    u8;
    /// Object Position
    pub positioin, set_position : 31, 28;

    pub give_back, set_give_back : 27; // = 0
    pub capability_mismatch, set_capability_mismatch : 26;
    pub usb_comm_capable, set_usb_comm_capable : 25;
    pub no_usb_suspend, set_no_usb_suspend : 24;
    pub unchunked_extended_message_support, set_unchunked_extended_message_support : 23;
    pub epr_mode_capable, set_epr_mode_capable : 22;
    pub u16, operating_current_10ma, set_operating_current_10ma : 19, 10;
    pub u16, max_operating_current_10ma, set_max_operating_current_10ma : 9, 0;
}

// TODO: Battery Request Data Object

// Table 6.26 “PPS Request Data Object”
bitfield! {
    pub struct PPSRequest(u32);
    impl Debug;
    u8;
    /// Object Position
    pub positioin, set_position : 31, 28;

    pub capability_mismatch, set_capability_mismatch : 26;
    pub usb_comm_capable, set_usb_comm_capable : 25;
    pub no_usb_suspend, set_no_usb_suspend : 24;
    pub unchunked_extended_message_support, set_unchunked_extended_message_support : 23;
    pub epr_mode_capable, set_epr_mode_capable : 22;
    pub u16, output_voltage_20mv, set_output_voltage_20mv : 20, 9;
    pub operating_current_50ma, set_operating_current_50ma : 6, 0;
}
