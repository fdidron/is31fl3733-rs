use crate::config::*;

pub struct State {
    pub page: u8,
    pub configuration_register: u8,
    pub leds: [u8; TOTAL_LED_COUNT / 8],
    pub brightness: [u8; TOTAL_LED_COUNT],
    pub global_current_control: u8,
}

impl Default for State {
    // This reflect the presumed default state after a reset
    fn default() -> Self {
        Self {
            page: 0,
            configuration_register: 0,
            leds: [0; 24],
            brightness: [0; 192],
            global_current_control: 0,
        }
    }
}
