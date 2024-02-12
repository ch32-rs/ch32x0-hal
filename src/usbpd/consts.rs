// Control Message Types
// Control Messages are short and manage the Message flow between Port Partners or for exchanging Messages that require no additional data.

pub const PD_MAX_EXT_MSG_LEGACY_LEN: usize = 26;

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

// USBPD->TX_SEL
pub const TX_SEL1: u8 = 0 << 0;
pub const TX_SEL1_SYNC1: u8 = 0 << 0; // 0-SYNC1
pub const TX_SEL1_RST1: u8 = 1 << 0; // 1-RST1
pub const TX_SEL2_MASK: u8 = 3 << 2;
pub const TX_SEL2_SYNC1: u8 = 0 << 2; // 00-SYNC1
pub const TX_SEL2_SYNC3: u8 = 1 << 2; // 01-SYNC3
pub const TX_SEL2_RST1: u8 = 2 << 2; // 1x-RST1
pub const TX_SEL3_MASK: u8 = 3 << 4;
pub const TX_SEL3_SYNC1: u8 = 0 << 4; // 00-SYNC1
pub const TX_SEL3_SYNC3: u8 = 1 << 4; // 01-SYNC3
pub const TX_SEL3_RST1: u8 = 2 << 4; // 1x-RST1
pub const TX_SEL4_MASK: u8 = 3 << 6;
pub const TX_SEL4_SYNC2: u8 = 0 << 6; // 00-SYNC2
pub const TX_SEL4_SYNC3: u8 = 1 << 6; // 01-SYNC3
pub const TX_SEL4_RST2: u8 = 2 << 6; // 1x-RST2

// Start of Packet Sequences
/// SOP
pub const UPD_SOP0: u8 = TX_SEL1_SYNC1 | TX_SEL2_SYNC1 | TX_SEL3_SYNC1 | TX_SEL4_SYNC2; // SOP1
/// SOP'
pub const UPD_SOP1: u8 = TX_SEL1_SYNC1 | TX_SEL2_SYNC1 | TX_SEL3_SYNC3 | TX_SEL4_SYNC3; // SOP2
/// SOP''
pub const UPD_SOP2: u8 = TX_SEL1_SYNC1 | TX_SEL2_SYNC3 | TX_SEL3_SYNC1 | TX_SEL4_SYNC3; // SOP3

pub const UPD_HARD_RESET: u8 = TX_SEL1_RST1 | TX_SEL2_RST1 | TX_SEL3_RST1 | TX_SEL4_RST2; // Hard Reset
pub const UPD_CABLE_RESET: u8 = TX_SEL1_RST1 | TX_SEL2_SYNC1 | TX_SEL3_RST1 | TX_SEL4_SYNC3; // Cable Reset

/* PD Revision */
pub const DEF_PD_REVISION_10: u8 = 0x00;
pub const DEF_PD_REVISION_20: u8 = 0x01;
pub const DEF_PD_REVISION_30: u8 = 0x02;

// Table 6.54 “Extended Message Types”
// when Header.ext=true
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum ExtendedMessageType {
    SourceCapabilitiesExtended = 0b0_0001,
    Status = 0b0_0010,
    GetBatteryCap = 0b0_0011,
    GetBatteryStatus = 0b0_0100,
    BatteryCapabilities = 0b0_0101,
    GetManufacturerInfo = 0b0_0110,
    ManufacturerInfo = 0b0_0111,
    SecurityRequest = 0b0_1000,
    SecurityResponse = 0b0_1001,
    FirmwareUpdateRequest = 0b0_1010,
    FirmwareUpdateResponse = 0b0_1011,
    PPSStatus = 0b0_1100,
    CountryInfo = 0b0_1101,
    CountryCodes = 0b0_1110,
    SinkCapabilitiesExtended = 0b0_1111,
    ExtendedControl = 0b1_0000,
    // 17, 0x11
    EPRSourceCapabilities = 0b1_0001,
    EPRSinkCapabilities = 0b1_0010,

    VendorDefinedExtended = 0b1_1111,
}

impl ExtendedMessageType {
    pub fn from_u8(raw: u8) -> Self {
        unsafe { core::mem::transmute(raw) }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum ExtendedControlType {
    EPR_Get_Source_Cap = 0x01,
    EPR_Get_Sink_Cap = 0x02,
    EPR_KeepAlive = 0x03,
    EPR_KeepAlive_Ack = 0x04,
}
