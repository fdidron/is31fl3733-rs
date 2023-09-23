#![no_std]

use embedded_hal::i2c::I2c;

pub struct IS31FL3731 {
    i2c: I2c,
    pwm: [u8; 0xc0],
    leds: [u8; 0x18],
    current_page: u8,
    address: u8,
}

const COMMAND_REGISTER: u8 = 0xfd;
const COMMAND_WRITE_LOCK_REGISTER: u8 = 0xfe;
const COMMAND_WRITE_UNLOCK: u8 = 0xc5;
const INTERRUPT_MASK_REGISTER: u8 = 0xf0;
const INTERRUPT_STATUS_REGISTER: u8 = 0xf1;

struct PagedRegister {
    page: u8,
    register: u8,
}

const LED_CONTROL_REGISTER_BASE: PagedRegister = PagedRegister { page: 0x00, register: 0x00 };
const LED_OPEN_REGISTER_BASE: PagedRegister = PagedRegister { page: 0x00, register: 0x18 };
const LED_SHORT_REGISTER_BASE: PagedRegister = PagedRegister { page: 0x00, register: 0x30 };

const PWM_REGISTER_BASE: PagedRegister = PagedRegister { page: 0x01, register: 0x00 };

const AUTO_BREATH_MODE_REGISTER_BASE: PagedRegister = PagedRegister { page: 0x02, register: 0x00 };

const CONFIGURATION_REGISTER: PagedRegister = PagedRegister { page: 0x03, register: 0x00 };
const RESET_REGISTER: PagedRegister = PagedRegister { page: 0x03, register: 0x11 };


impl IS31FL3731 {
    pub fn new(i2c: I2c, address: u8) -> Self {
        Self {
            i2c,
            pwm: [0; 0xc0],
            leds: [0; 0x18],
            current_page: 0,
            address,
        }
    }

    pub fn write_register(&mut self, address: u8, value: u8) -> Result<(), ()> {
        self.i2c.write(self.address, &[address, value])?;
    }

    pub fn read_register(&mut self, address: u8) -> u8 {
        let mut buffer = [0; 1];
        self.i2c.write_read(self.address, &[address], &mut buffer).unwrap();
        buffer[0]
    }

    pub fn unlock(&mut self) {
        self.write_register(COMMAND_WRITE_LOCK_REGISTER, COMMAND_WRITE_UNLOCK);
    }

    pub fn set_page(&mut self, page: u8) {
        if self.current_page != page {
            self.unlock();
            self.write_register(COMMAND_REGISTER, page);
            self.current_page = page;
        }
    }
}

mod tests {
    use super::*;

    #[test]
    fn it_works() {
    }
}
