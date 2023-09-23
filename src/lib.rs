#![no_std]

use embedded_hal::i2c::I2c;
use embedded_hal::i2c::{Error, ErrorType, ErrorKind};

#[derive(Debug)]
pub enum IS31FL3731Error {
    I2CError,
}

impl Error for IS31FL3731Error {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}


impl<BUS: I2c> ErrorType for IS31FL3731<BUS> {
    type Error = IS31FL3731Error;
}

pub struct IS31FL3731<BUS: I2c> {
    i2c: BUS,
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


impl<BUS: I2c> IS31FL3731<BUS> {
    pub fn new(i2c: BUS, address: u8) -> Self {
        Self {
            i2c,
            current_page: 0,
            address,
        }
    }

    pub fn write_paged(&mut self, page: u8, address: u8, value: u8) -> Result<(), IS31FL3731Error> {
        self.set_page(page)?;
        self.write_register(address, value)?;
        Ok(())
    }

    pub fn write_register(&mut self, address: u8, value: u8) -> Result<(), IS31FL3731Error> {
        self.i2c.write(self.address, &[address, value])
            .map_err(|e| IS31FL3731Error::I2CError)?;
        Ok(())
    }

    pub fn read_register(&mut self, address: u8) -> Result<u8, IS31FL3731Error> {
        let mut buffer = [0; 1];
        self.i2c.write_read(self.address, &[address], &mut buffer)
            .map_err(|e| IS31FL3731Error::I2CError)?;
        Ok(buffer[0])
    }

    pub fn unlock(&mut self) -> Result<(), IS31FL3731Error> {
        self.write_register(COMMAND_WRITE_LOCK_REGISTER, COMMAND_WRITE_UNLOCK)?;
        Ok(())
    }

    pub fn set_page(&mut self, page: u8) -> Result<(), IS31FL3731Error> {
        if self.current_page != page {
            self.unlock()?;
            self.write_register(COMMAND_REGISTER, page)?;
            self.current_page = page;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug)]
    enum FakeI2cError {
        Error,
    }
    impl Error for FakeI2cError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    struct FakeI2cBus<const N: usize> {
        pub data: heapless::Vec<u8, N>,
    }

    impl<const N: usize> ErrorType for FakeI2cBus<N> {
        type Error = FakeI2cError;
    }

    impl<const N: usize>  FakeI2cBus<N> {
        pub fn new() -> Self {
            Self {
                data: heapless::Vec::new(),
            }
        }
    }

    impl<const N: usize> embedded_hal::i2c::I2c for FakeI2cBus<N> {
        fn transaction(
            &mut self,
            address: embedded_hal::i2c::SevenBitAddress,
            operations: &mut [embedded_hal::i2c::Operation],
        ) -> Result<(), Self::Error> {
            for operation in operations {
                match operation {
                    embedded_hal::i2c::Operation::Write(write) => {
                        self.data.extend_from_slice(write)
                            .map_err(|_| FakeI2cError::Error)?;
                    },
                    embedded_hal::i2c::Operation::Read(read) => {
                        panic!("Read not implemented");
                    },
                }
            }
            Ok(())
        }
    }

    #[test]
    fn page_set_test() {
        let mut bus = FakeI2cBus::<128>::new();
        let mut is31fl3731 = IS31FL3731::new(&mut bus, 0x60);
        is31fl3731.set_page(0x01).unwrap();
        is31fl3731.set_page(0x01).unwrap();
        is31fl3731.set_page(0x02).unwrap();
        assert_eq!(bus.data, [0xfe, 0xc5, 0xfd, 0x01, 0xfe, 0xc5, 0xfd, 0x02]);
    }
}
