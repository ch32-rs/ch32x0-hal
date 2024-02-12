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

bitfield! {
    pub struct FixedPowerDataObject(u32);
    impl Debug;
    u32;

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

//  Battery PDO uses power instead of current.
bitfield! {
    pub struct BatterySupplyPowerDataObject(u32);
    impl Debug;
    u32;

    // 0b01
    pub max_voltage_50mv, _ : 29, 20;
    pub min_voltage_50mv, _ : 19, 10;
    pub max_power_250mw, _ : 9, 0;
}

// Table 6.11 “Variable Supply (non-Battery) PDO – Source”
bitfield! {
    pub struct VariableSupplyPowerDataObject(u32);
    impl Debug;
    u32;

    // 0b10
    pub max_voltage_50mv, _ : 29, 20;
    pub min_voltage_50mv, _ : 19, 10;
    pub max_current_10ma, _ : 9, 0;
}

bitfield! {
    /// Table 6.8 “Augmented Power Data Object”, APDO
    /// Table 6.13 “SPR Programmable Power Supply APDO – Source”
    /// PPS APDO
    pub struct PPS_APDO(u32);
    impl Debug;
    u32;
    // 0b11
    // 0b00
    pub max_voltage_100mv, _ : 24, 17;
    pub min_voltage_100mv, _ : 15, 8;
    pub max_current_50ma, _ : 6, 0;
}

bitfield! {
    /// Table 6.14 “EPR Adjustable Voltage Supply APDO – Source “
    pub struct EPR_APDO(u32);
    impl Debug;
    u32;
    // 0b11
    // 0b01
    pub peak_current, _ : 27, 26;
    pub max_voltage_100mv, _ : 25, 17;
    pub min_voltage_100mv, _ : 15, 8;
    pub pdp_1w, _ : 7, 0;
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
