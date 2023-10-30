#![no_std]
use core::cell::RefCell;
use diff_in_place::DiffInPlace;
use embedded_hal::i2c::{Error, ErrorKind, ErrorType};

mod config;
use config::*;

#[derive(Debug)]
pub enum IS31FL3733Error {
    I2CError(ErrorKind),
    StateError,
    DeviceError,
    OutOfSpaceError,
}

impl Error for IS31FL3733Error {
    fn kind(&self) -> ErrorKind {
        match self {
            IS31FL3733Error::I2CError(kind) => *kind,
            _ => ErrorKind::Other,
        }
    }
}

impl<BUS: embedded_hal::i2c::I2c> ErrorType for IS31FL3733<BUS> {
    type Error = IS31FL3733Error;
}

pub struct IS31FL3733<BUS: embedded_hal::i2c::I2c> {
    i2c: RefCell<BUS>,
    state: State,
    address: u8,
}

struct State {
    page: u8,
    configuration_register: u8,
    leds: [u8; TOTAL_LED_COUNT / 8],
    brightness: [u8; TOTAL_LED_COUNT],
    global_current_control: u8,
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

impl<BUS: embedded_hal::i2c::I2c> IS31FL3733<BUS> {
    /// Create a new IS31FL3733 driver
    /// # Arguments
    /// * `i2c` - The I2C bus to use
    /// * `address` - The I2C address of the device
    ///
    /// # Returns
    /// A new IS31FL3733 driver
    pub fn new(i2c: BUS, address: u8) -> Self {
        Self {
            i2c: RefCell::new(i2c),
            address,
            state: State::default(),
        }
    }

    /// Initialize the device
    ///
    /// # Returns
    /// * Ok(()) if the device was initialized successfully
    /// * Err(IS31FL3733Error::DeviceError) if the device did not respond as expected
    /// * Err(IS31FL3733Error::I2CError) if there was an I2C error
    pub fn initialize(&mut self) -> Result<(), IS31FL3733Error> {
        self.set_page(RESET_REGISTER.page)?;
        let reset_result = self.read_register(RESET_REGISTER.register)?;

        if reset_result != 0x00 {
            return Err(IS31FL3733Error::DeviceError);
        }
        self.set_configuration(CONFIGURATION_SOFTWARE_SHUTDOWN_DISABLE)?;
        self.set_page(self.state.page)?;

        Ok(())
    }

    /// Set the global current control
    ///
    /// # Arguments
    /// * `gcc` - The global current control value
    ///
    /// # Returns
    /// * Ok(()) if the global current control was set successfully
    /// * Err(IS31FL3733Error::I2CError) if there was an I2C error
    pub fn set_global_current_control(
        &mut self,
        gcc: u8,
    ) -> Result<(), IS31FL3733Error> {
        if self.state.global_current_control != gcc {
            self.set_page(GCC_REGISTER.page)?;
            self.write_register(GCC_REGISTER.register, gcc)?;
            self.state.global_current_control = gcc;
        }
        Ok(())
    }

    /// Set the configuration register
    ///
    /// # Arguments
    /// * `configuration` - The configuration register value
    ///
    /// # Returns
    /// * Ok(()) if the configuration register was set successfully
    /// * Err(IS31FL3733Error::I2CError) if there was an I2C error
    pub fn set_configuration(
        &mut self,
        configuration: u8,
    ) -> Result<(), IS31FL3733Error> {
        if self.state.configuration_register != configuration {
            self.set_page(CONFIGURATION_REGISTER.page)?;
            self.write_register(
                CONFIGURATION_REGISTER.register,
                configuration,
            )?;
            self.state.configuration_register = configuration;
        }
        Ok(())
    }

    /// Unlock the page register
    ///
    /// # Returns
    /// * Ok(()) if the command register was unlocked successfully
    /// * Err(IS31FL3733Error::I2CError) if there was an I2C error
    fn unlock(&mut self) -> Result<(), IS31FL3733Error> {
        self.write_register(COMMAND_WRITE_LOCK_REGISTER, COMMAND_WRITE_UNLOCK)?;
        Ok(())
    }

    /// Set the page register, must be unlocked first using `unlock()`
    ///
    /// # Arguments
    /// * `page` - The page to set
    ///
    /// # Returns
    /// * Ok(()) if the page register was set successfully
    /// * Err(IS31FL3733Error::I2CError) if there was an I2C error
    fn set_page(&mut self, page: u8) -> Result<(), IS31FL3733Error> {
        if page != self.state.page {
            self.unlock()?;
            self.write_register(COMMAND_REGISTER, page)?;

            self.state.page = page;
        }
        Ok(())
    }

    /// Set the LEDs state on the device. Each bit in the array represents an LED.
    /// Only the delta between the current state and the new state is written to the device.
    ///
    /// # Arguments
    /// * `leds` - The new state of the LEDs
    ///
    /// # Returns
    /// * Ok(()) if the LEDs were set successfully
    /// * Err(IS31FL3733Error::I2CError) if there was an I2C error
    pub fn set_leds(
        &mut self,
        leds: &[u8; TOTAL_LED_COUNT / 8],
    ) -> Result<(), IS31FL3733Error> {
        self.set_page(LED_CONTROL_REGISTER_BASE.page)?;

        self.state.leds.try_diff_in_place(
            leds,
            |idx, data| -> Result<(), IS31FL3733Error> {
                self.write_buffer::<{ TOTAL_LED_COUNT / 8 }>(
                    LED_CONTROL_REGISTER_BASE.register + idx as u8,
                    data,
                )?;
                Ok(())
            },
        )?;

        Ok(())
    }

    /// Set the brightness of the LEDs on the device.
    /// Each byte in the array represents the brightness of a LED.
    /// Only the delta between the current state and the new state is written to the device.
    ///
    /// # Arguments
    /// * `brightness` - The new brightness of the LEDs
    ///
    /// # Returns
    /// * Ok(()) if the brightness was set successfully
    /// * Err(IS31FL3733Error::I2CError) if there was an I2C error
    pub fn set_brightness(
        &mut self,
        brightness: &[u8; TOTAL_LED_COUNT],
    ) -> Result<(), IS31FL3733Error> {
        self.set_page(PWM_REGISTER_BASE.page)?;

        // TODO - is is possible to avoid this copy?
        // perhaps with lifetime annotations?
        self.state.brightness.try_diff_in_place(
            brightness,
            |idx, data| -> Result<(), IS31FL3733Error> {
                self.write_buffer::<TOTAL_LED_COUNT>(
                    PWM_REGISTER_BASE.register + idx as u8,
                    data,
                )?;
                Ok(())
            },
        )?;

        Ok(())
    }

    fn write_buffer<const N: usize>(
        &self,
        address: u8,
        data: &[u8],
    ) -> Result<(), IS31FL3733Error> {
        let mut buffer: heapless::Vec<u8, N> = heapless::Vec::new();

        // Push address
        buffer
            .push(address)
            .map_err(|_| IS31FL3733Error::OutOfSpaceError)?;

        // and then data
        buffer
            .extend_from_slice(data)
            .map_err(|_| IS31FL3733Error::OutOfSpaceError)?;

        self.i2c
            .borrow_mut()
            .write(self.address, buffer.as_slice())
            .map_err(|e| IS31FL3733Error::I2CError(e.kind()))?;

        Ok(())
    }

    fn write_register(
        &mut self,
        address: u8,
        value: u8,
    ) -> Result<(), IS31FL3733Error> {
        self.i2c
            .borrow_mut()
            .write(self.address, &[address, value])
            .map_err(|e| IS31FL3733Error::I2CError(e.kind()))?;
        Ok(())
    }

    fn read_register(&mut self, address: u8) -> Result<u8, IS31FL3733Error> {
        let mut buffer = [0; 1];

        self.i2c
            .borrow_mut()
            .write_read(self.address, &[address], &mut buffer)
            .map_err(|e| IS31FL3733Error::I2CError(e.kind()))?;
        Ok(buffer[0])
    }
}

#[cfg(test)]
mod test_utils;

#[cfg(test)]
mod tests {
    use super::*;
    use test_utils::*;

    impl<const N: usize, const M: usize> embedded_hal::i2c::I2c
        for FakeI2cBus<N, M>
    {
        fn transaction(
            &mut self,
            _address: embedded_hal::i2c::SevenBitAddress,
            operations: &mut [embedded_hal::i2c::Operation],
        ) -> Result<(), Self::Error> {
            for operation in operations {
                match operation {
                    embedded_hal::i2c::Operation::Write(write) => {
                        self.write_data
                            .extend_from_slice(write)
                            .map_err(|_| FakeI2cError::Error)?;
                    }
                    embedded_hal::i2c::Operation::Read(read) => {
                        // Copy read.len() bytes from read_data to read and remove those bytes
                        for i in 0..read.len() {
                            read[i] = self.read_data.remove(i);
                        }
                    }
                }
            }
            Ok(())
        }
    }

    #[test]
    fn init_test() {
        const EXPECTED_WRITE_DATA: [u8; 7] =
            [0xfe, 0xc5, 0xfd, 0x03, 0x11, 0x00, 0x01];
        const EXPECTED_READ_DATA: [u8; 1] = [0];

        let mut bus =
            FakeI2cBus::<32, 32>::new_with_read_data(&EXPECTED_READ_DATA);

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);
        is31fl3733.initialize().unwrap();

        assert_eq!(bus.write_data_as_ref(), EXPECTED_WRITE_DATA);
    }

    #[test]
    fn configuration_test() {
        const EXPECTED_WRITE_DATA: &[u8] =
            &[0xfe, 0xc5, 0xfd, 0x03, 0x00, 0xaa, 0x00, 0xab];

        let mut bus = FakeI2cBus::<32, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);
        is31fl3733.set_configuration(0xaa).unwrap();
        is31fl3733.set_configuration(0xaa).unwrap();
        is31fl3733.set_configuration(0xab).unwrap();

        assert_eq!(bus.write_data_as_ref(), EXPECTED_WRITE_DATA);
    }

    #[test]
    fn global_current_control_test() {
        const EXPECTED_WRITE_DATA: &[u8] =
            &[0xfe, 0xc5, 0xfd, 0x03, 0x01, 0xaa, 0x01, 0xab];

        let mut bus = FakeI2cBus::<32, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

        is31fl3733.set_global_current_control(0xaa).unwrap();
        is31fl3733.set_global_current_control(0xaa).unwrap();
        is31fl3733.set_global_current_control(0xab).unwrap();

        assert_eq!(bus.write_data_as_ref(), EXPECTED_WRITE_DATA);
    }

    #[test]
    fn state_update_test() {
        #[rustfmt::skip]
        const EXPECTED_WRITE_DATA: &[u8] = &[
            254, 197,  // Write unlock
            253, 1,   // Set page to 1
            20, 255, 255, // set brightness leds 20 to 21 to 0xff
            188, 240, 240, 240, 240, // set brightness leds 188 to 192 to 0xff
            254, 197, // Write unlock
            253, 0,  // Set page to 0
            10, 255, 255, // set leds 10 to 11 to 0xff
        ];

        let mut bus = FakeI2cBus::<32, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

        let mut new_brightness = [0; 192];

        new_brightness[20..22].fill(0xff);
        new_brightness[188..].fill(0xf0);

        is31fl3733.set_brightness(&new_brightness).unwrap();

        let mut new_leds = [0; 24];

        new_leds[10..12].fill(0xff);

        is31fl3733.set_leds(&new_leds).unwrap();

        assert_eq!(bus.write_data_as_ref(), EXPECTED_WRITE_DATA);
    }
}
