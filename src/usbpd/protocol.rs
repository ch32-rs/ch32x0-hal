use bitfield::bitfield;

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

bitfield! {
    /// Table 6.8 “Augmented Power Data Object”, APDO
    pub struct AugmentedPowerDataObject(u32);
    impl Debug;
    u32;
    // 0b11
    // 0b00
    pub max_voltage_100mv, _ : 24, 17;
    pub min_voltage_100mv, _ : 15, 8;
    pub max_current_50ma, _ : 6, 0;


}
