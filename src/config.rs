#![allow(dead_code)]
pub const COMMAND_REGISTER: u8 = 0xfd;
pub const COMMAND_WRITE_LOCK_REGISTER: u8 = 0xfe;
pub const COMMAND_WRITE_UNLOCK: u8 = 0xc5;
pub const INTERRUPT_MASK_REGISTER: u8 = 0xf0;
pub const INTERRUPT_STATUS_REGISTER: u8 = 0xf1;

pub struct PagedRegister {
    pub page: u8,
    pub register: u8,
}

pub const LED_CONTROL_REGISTER_BASE: PagedRegister = PagedRegister {
    page: 0x00,
    register: 0x00,
};
pub const LED_OPEN_REGISTER_BASE: PagedRegister = PagedRegister {
    page: 0x00,
    register: 0x18,
};
pub const LED_SHORT_REGISTER_BASE: PagedRegister = PagedRegister {
    page: 0x00,
    register: 0x30,
};

pub const PWM_REGISTER_BASE: PagedRegister = PagedRegister {
    page: 0x01,
    register: 0x00,
};

pub const AUTO_BREATH_MODE_REGISTER_BASE: PagedRegister = PagedRegister {
    page: 0x02,
    register: 0x00,
};

pub const CONFIGURATION_REGISTER: PagedRegister = PagedRegister {
    page: 0x03,
    register: 0x00,
};
pub const GCC_REGISTER: PagedRegister = PagedRegister {
    page: 0x03,
    register: 0x01,
};
pub const RESET_REGISTER: PagedRegister = PagedRegister {
    page: 0x03,
    register: 0x11,
};

pub const CONFIGURATION_SYNC_HIGH_IMPEDANCE: u8 = 0b0000_0000;
pub const CONFIGURATION_SYNC_HIGH_IMPEDANCE_ALTERNATE: u8 = 0b0110_0000;
pub const CONFIGURATION_SYNC_MASTER: u8 = 0b0010_0000;
pub const CONFIGURATION_SYNC_SLAVE: u8 = 0b0100_0000;
pub const CONFIGURATION_OSD_ENABLE: u8 = 0b0000_0100;
pub const CONFIGURATION_AUTO_BREATH_MODE_ENABLE: u8 = 0b0000_0010;
pub const CONFIGURATION_SOFTWARE_SHUTDOWN_DISABLE: u8 = 0b0000_0001;

pub const TOTAL_LED_COUNT: usize = 192;

