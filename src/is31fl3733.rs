use crate::config::*;
use crate::state::State;

use diff_in_place::DiffInPlace;

#[derive(Debug)]
pub enum IS31FL3733Error {
    StateError,
    DeviceError,
    OutOfSpaceError,
}

pub trait Mode {}

#[derive(Debug)]
pub struct Async;
#[derive(Debug)]
pub struct Blocking;

impl Mode for Async {}
impl Mode for Blocking {}

pub struct IS31FL3733<BUS, M: Mode> {
    bus: BUS,
    address: u8,
    state: State,
    _phantom: core::marker::PhantomData<M>,
}

// General implementation
impl<BUS, M: Mode> IS31FL3733<BUS, M> {
    /// Create a new IS31FL3733 driver
    /// # Arguments
    /// * `i2c` - The I2C bus to use
    /// * `address` - The I2C address of the device
    ///
    /// # Returns
    /// A new IS31FL3733 driver
    pub fn new(bus: BUS, address: u8) -> Self {
        Self {
            bus,
            address,
            state: State::default(),
            _phantom: core::marker::PhantomData,
        }
    }

    pub fn into_inner(self) -> BUS {
        self.bus
    }

    pub fn inner(&self) -> &BUS {
        &self.bus
    }

    pub fn inner_mut(&mut self) -> &mut BUS {
        &mut self.bus
    }
}

impl<BUS: embedded_hal::i2c::I2c> IS31FL3733<BUS, Blocking> {
    pub fn new_blocking(bus: BUS, address: u8) -> Self {
        Self {
            bus,
            address,
            state: State::default(),
            _phantom: core::marker::PhantomData,
        }
    }

    fn write(&mut self, address: u8, data: &[u8]) -> Result<(), BUS::Error> {
        self.bus.transaction(
            self.address,
            &mut [
                embedded_hal::i2c::Operation::Write(&[address]),
                embedded_hal::i2c::Operation::Write(data),
            ],
        )?;

        Ok(())
    }

    fn read(&mut self, address: u8, data: &mut [u8]) -> Result<(), BUS::Error> {
        self.bus.transaction(
            self.address,
            &mut [
                embedded_hal::i2c::Operation::Write(&[address]),
                embedded_hal::i2c::Operation::Read(data),
            ],
        )?;

        Ok(())
    }

    fn write_register(
        &mut self,
        register: u8,
        data: u8,
    ) -> Result<(), BUS::Error> {
        self.write(register, &[data])
    }

    fn read_register(&mut self, register: u8) -> Result<u8, BUS::Error> {
        let mut data = [0];
        self.read(register, &mut data)?;
        Ok(data[0])
    }

    /// Initialize the device
    ///
    /// # Returns
    /// * Ok(()) if the device was initialized successfully
    /// * Err(IS31FL3733Error::DeviceError) if the device did not respond as expected
    pub fn initialize(&mut self) -> Result<(), IS31FL3733Error> {
        self.set_page(RESET_REGISTER.page)?;
        let reset_result = self
            .read_register(RESET_REGISTER.register)
            .map_err(|_| IS31FL3733Error::DeviceError)?;

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
    pub fn set_global_current_control(
        &mut self,
        gcc: u8,
    ) -> Result<(), IS31FL3733Error> {
        if self.state.global_current_control != gcc {
            self.set_page(GCC_REGISTER.page)?;
            self.write_register(GCC_REGISTER.register, gcc)
                .map_err(|_| IS31FL3733Error::DeviceError)?;
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
    pub fn set_configuration(
        &mut self,
        configuration: u8,
    ) -> Result<(), IS31FL3733Error> {
        if self.state.configuration_register != configuration {
            self.set_page(CONFIGURATION_REGISTER.page)?;
            self.write_register(CONFIGURATION_REGISTER.register, configuration)
                .map_err(|_| IS31FL3733Error::DeviceError)?;
            self.state.configuration_register = configuration;
        }
        Ok(())
    }

    /// Unlock the page register
    ///
    /// # Returns
    /// * Ok(()) if the command register was unlocked successfully
    fn unlock(&mut self) -> Result<(), IS31FL3733Error> {
        self.write_register(COMMAND_WRITE_LOCK_REGISTER, COMMAND_WRITE_UNLOCK)
            .map_err(|_| IS31FL3733Error::DeviceError)?;
        Ok(())
    }

    /// Set the page register, must be unlocked first using `unlock()`
    ///
    /// # Arguments
    /// * `page` - The page to set
    ///
    /// # Returns
    /// * Ok(()) if the page register was set successfully
    fn set_page(&mut self, page: u8) -> Result<(), IS31FL3733Error> {
        if page != self.state.page {
            self.unlock()?;
            self.write_register(COMMAND_REGISTER, page)
                .map_err(|_| IS31FL3733Error::DeviceError)?;

            self.state.page = page;
        }
        Ok(())
    }

    /// Set the LEDs state on the device. Each bit in the array represents a LED.
    /// Only the delta between the current state and the new state is written to the device.
    ///
    /// # Arguments
    /// * `leds` - The new state of the LEDs
    ///

    /// * Ok(()) if the LEDs were set successfully
    pub fn update_leds(
        &mut self,
        leds: &[u8; TOTAL_LED_COUNT / 8],
    ) -> Result<(), IS31FL3733Error> {
        let current = self.state.leds;

        current.try_diff_in_place(
            leds,
            |index, leds| -> Result<(), IS31FL3733Error> {
                self.write_leds(index, leds)?;
                Ok(())
            },
        )?;

        Ok(())
    }
    /// Sets the state of the LEDs on the device, starting from a
    /// specific index.
    ///
    /// # Arguments
    /// * `index` - The index of the first LED array to update
    /// * `leds` - The new brightness of the LEDs, one bit per LED
    ///
    /// # Returns
    /// * Ok(()) if the brightness was set successfully
    pub fn write_leds(
        &mut self,
        index: usize,
        leds: &[u8],
    ) -> Result<(), IS31FL3733Error> {
        core::assert!(index + leds.len() <= TOTAL_LED_COUNT / 8);

        self.set_page(LED_CONTROL_REGISTER_BASE.page)?;
        self.write(LED_CONTROL_REGISTER_BASE.register + index as u8, leds)
            .map_err(|_| IS31FL3733Error::DeviceError)?;

        self.state.leds[index..index + leds.len()].copy_from_slice(leds);
        Ok(())
    }

    /// Set the LEDs brightness on the device. Each byte in the array represents a LED.
    /// Only the delta between the current state and the new state is written to the device.
    ///
    /// # Arguments
    /// * `brightness` - The new state of the LEDs
    ///
    /// * Ok(()) if the LEDs were set successfully
    pub fn update_brightness(
        &mut self,
        brightness: &[u8; TOTAL_LED_COUNT],
    ) -> Result<(), IS31FL3733Error> {
        let current = self.state.brightness;

        current.try_diff_in_place(
            brightness,
            |index, brightness| -> Result<(), IS31FL3733Error> {
                self.write_brightness(index, brightness)?;
                Ok(())
            },
        )?;

        Ok(())
    }

    /// Sets the brightness of the LEDs on the device, starting from a
    /// specific index.
    ///
    /// # Arguments
    /// * `index` - The index of the first LED to update
    /// * `brightness` - The new brightness of the LEDs, one byte per LED
    ///
    /// # Returns
    /// * Ok(()) if the brightness was set successfully
    pub fn write_brightness(
        &mut self,
        index: usize,
        brightness: &[u8],
    ) -> Result<(), IS31FL3733Error> {
        core::assert!(index + brightness.len() <= TOTAL_LED_COUNT);

        self.set_page(PWM_REGISTER_BASE.page)?;

        self.write(PWM_REGISTER_BASE.register + index as u8, brightness)
            .map_err(|_| IS31FL3733Error::DeviceError)?;

        self.state.brightness[index..index + brightness.len()]
            .copy_from_slice(brightness);

        Ok(())
    }
}

impl<BUS: embedded_hal_async::i2c::I2c> IS31FL3733<BUS, Async> {
    pub fn new_async(bus: BUS, address: u8) -> Self {
        Self {
            bus,
            address,
            state: State::default(),
            _phantom: core::marker::PhantomData,
        }
    }

    async fn write(
        &mut self,
        address: u8,
        data: &[u8],
    ) -> Result<(), BUS::Error> {
        self.bus
            .transaction(
                self.address,
                &mut [
                    embedded_hal_async::i2c::Operation::Write(&[address]),
                    embedded_hal_async::i2c::Operation::Write(data),
                ],
            )
            .await?;

        Ok(())
    }

    async fn read(
        &mut self,
        address: u8,
        data: &mut [u8],
    ) -> Result<(), BUS::Error> {
        self.bus
            .transaction(
                self.address,
                &mut [
                    embedded_hal_async::i2c::Operation::Write(&[address]),
                    embedded_hal_async::i2c::Operation::Read(data),
                ],
            )
            .await?;

        Ok(())
    }

    async fn write_register(
        &mut self,
        register: u8,
        data: u8,
    ) -> Result<(), BUS::Error> {
        self.write(register, &[data]).await
    }

    async fn read_register(&mut self, register: u8) -> Result<u8, BUS::Error> {
        let mut data = [0];
        self.read(register, &mut data).await?;
        Ok(data[0])
    }

    /// Initialize the device
    ///
    /// # Returns
    /// * Ok(()) if the device was initialized successfully
    /// * Err(IS31FL3733Error::DeviceError) if the device did not respond as expected
    pub async fn initialize(&mut self) -> Result<(), IS31FL3733Error> {
        self.set_page(RESET_REGISTER.page).await?;
        let reset_result = self
            .read_register(RESET_REGISTER.register)
            .await
            .map_err(|_| IS31FL3733Error::DeviceError)?;

        if reset_result != 0x00 {
            return Err(IS31FL3733Error::DeviceError);
        }
        self.set_configuration(CONFIGURATION_SOFTWARE_SHUTDOWN_DISABLE)
            .await?;
        self.set_page(self.state.page).await?;

        Ok(())
    }

    /// Set the global current control
    ///
    /// # Arguments
    /// * `gcc` - The global current control value
    ///
    /// # Returns
    /// * Ok(()) if the global current control was set successfully
    pub async fn set_global_current_control(
        &mut self,
        gcc: u8,
    ) -> Result<(), IS31FL3733Error> {
        if self.state.global_current_control != gcc {
            self.set_page(GCC_REGISTER.page).await?;
            self.write_register(GCC_REGISTER.register, gcc)
                .await
                .map_err(|_| IS31FL3733Error::DeviceError)?;
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
    pub async fn set_configuration(
        &mut self,
        configuration: u8,
    ) -> Result<(), IS31FL3733Error> {
        if self.state.configuration_register != configuration {
            self.set_page(CONFIGURATION_REGISTER.page).await?;
            self.write_register(CONFIGURATION_REGISTER.register, configuration)
                .await
                .map_err(|_| IS31FL3733Error::DeviceError)?;
            self.state.configuration_register = configuration;
        }
        Ok(())
    }

    /// Unlock the page register
    ///
    /// # Returns
    /// * Ok(()) if the command register was unlocked successfully
    async fn unlock(&mut self) -> Result<(), IS31FL3733Error> {
        self.write_register(COMMAND_WRITE_LOCK_REGISTER, COMMAND_WRITE_UNLOCK)
            .await
            .map_err(|_| IS31FL3733Error::DeviceError)?;
        Ok(())
    }

    /// Set the page register, must be unlocked first using `unlock()`
    ///
    /// # Arguments
    /// * `page` - The page to set
    ///
    /// # Returns
    /// * Ok(()) if the page register was set successfully
    async fn set_page(&mut self, page: u8) -> Result<(), IS31FL3733Error> {
        if page != self.state.page {
            self.unlock().await?;
            self.write_register(COMMAND_REGISTER, page)
                .await
                .map_err(|_| IS31FL3733Error::DeviceError)?;

            self.state.page = page;
        }
        Ok(())
    }

    /// Set the LEDs state on the device. Each bit in the array represents a LED.
    /// Only the delta between the current state and the new state is written to the device.
    ///
    /// # Arguments
    /// * `leds` - The new state of the LEDs
    ///
    /// # Returns
    /// * Ok(()) if the LEDs were set successfully
    pub async fn update_leds(
        &mut self,
        leds: &[u8; TOTAL_LED_COUNT / 8],
    ) -> Result<(), IS31FL3733Error> {
        // note: I couldn't figure out a way to make diff_in_place work with async without having
        // to use boxed futures, which is not no_std compatible, so I'm reluctantly inlining here
        let current = self.state.leds;

        let byte_for_byte = current.iter().zip(leds.iter());
        let mut run_state = DiffState::Same;
        for (current, (left, right)) in byte_for_byte.enumerate() {
            match (run_state, left == right) {
                (DiffState::Same, false) => {
                    // We are starting an unequal run, preserve the current index
                    run_state = DiffState::Different(current);
                }
                (DiffState::Different(run_start), true) => {
                    // We are ending an unequal run, call the diff function
                    self.write_leds(run_start, &leds[run_start..current])
                        .await?;
                    run_state = DiffState::Same;
                }
                _ => {
                    // Run state is unchanged
                }
            }
        }

        // If we are still in a different run, call the diff function
        if let DiffState::Different(run_start) = run_state {
            self.write_leds(run_start, &leds[run_start..]).await?;
        }
        Ok(())
    }
    /// Sets the state of the LEDs on the device, starting from a
    /// specific index.
    ///
    /// # Arguments
    /// * `index` - The index of the first LED array to update
    /// * `leds` - The new brightness of the LEDs, one bit per LED
    ///
    /// # Returns
    /// * Ok(()) if the brightness was set successfully
    pub async fn write_leds(
        &mut self,
        index: usize,
        leds: &[u8],
    ) -> Result<(), IS31FL3733Error> {
        core::assert!(index + leds.len() <= TOTAL_LED_COUNT / 8);

        self.set_page(LED_CONTROL_REGISTER_BASE.page).await?;
        self.write(LED_CONTROL_REGISTER_BASE.register + index as u8, leds)
            .await
            .map_err(|_| IS31FL3733Error::DeviceError)?;

        self.state.leds[index..index + leds.len()].copy_from_slice(leds);
        Ok(())
    }

    /// Set the LEDs brightness on the device. Each byte in the array represents a LED.
    /// Only the delta between the current state and the new state is written to the device.
    ///
    /// # Arguments
    /// * `brightness` - The new state of the LEDs
    ///
    /// * Ok(()) if the LEDs were set successfully
    pub async fn update_brightness(
        &mut self,
        brightness: &[u8; TOTAL_LED_COUNT],
    ) -> Result<(), IS31FL3733Error> {
        let current = self.state.brightness;

        let byte_for_byte = current.iter().zip(brightness.iter());
        let mut run_state = DiffState::Same;
        for (current, (left, right)) in byte_for_byte.enumerate() {
            match (run_state, left == right) {
                (DiffState::Same, false) => {
                    // We are starting an unequal run, preserve the current index
                    run_state = DiffState::Different(current);
                }
                (DiffState::Different(run_start), true) => {
                    // We are ending an unequal run, call the diff function
                    self.write_brightness(
                        run_start,
                        &brightness[run_start..current],
                    )
                    .await?;
                    run_state = DiffState::Same;
                }
                _ => {
                    // Run state is unchanged
                }
            }
        }

        // If we are still in a different run, call the diff function
        if let DiffState::Different(run_start) = run_state {
            self.write_brightness(run_start, &brightness[run_start..])
                .await?;
        }

        Ok(())
    }

    /// Sets the brightness of the LEDs on the device, starting from a
    /// specific index.
    ///
    /// # Arguments
    /// * `index` - The index of the first LED to update
    /// * `brightness` - The new brightness of the LEDs, one byte per LED
    ///
    /// # Returns
    /// * Ok(()) if the brightness was set successfully
    pub async fn write_brightness(
        &mut self,
        index: usize,
        brightness: &[u8],
    ) -> Result<(), IS31FL3733Error> {
        core::assert!(index + brightness.len() <= TOTAL_LED_COUNT);

        self.set_page(PWM_REGISTER_BASE.page).await?;

        self.write(PWM_REGISTER_BASE.register + index as u8, brightness)
            .await
            .map_err(|_| IS31FL3733Error::DeviceError)?;

        self.state.brightness[index..index + brightness.len()]
            .copy_from_slice(brightness);

        Ok(())
    }
}

#[derive(Copy, Clone)]
enum DiffState {
    Same,
    Different(usize),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_utils::*;

    #[test]
    fn init_test() {
        const EXPECTED_WRITE_DATA: [u8; 7] =
            [0xfe, 0xc5, 0xfd, 0x03, 0x11, 0x00, 0x01];
        const EXPECTED_READ_DATA: [u8; 1] = [0];

        let mut bus = FakeI2cBus::<32, 32, Blocking>::new_with_read_data(
            &EXPECTED_READ_DATA,
        );

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);
        is31fl3733.initialize().unwrap();

        assert_eq!(bus.write_data_as_ref(), EXPECTED_WRITE_DATA);
    }

    #[test]
    fn configuration_test() {
        const EXPECTED_WRITE_DATA: &[u8] =
            &[0xfe, 0xc5, 0xfd, 0x03, 0x00, 0xaa, 0x00, 0xab];

        let mut bus = FakeI2cBus::<32, 32, _>::new_blocking();

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

        let mut bus = FakeI2cBus::<32, 32, _>::new_blocking();

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

        let mut bus = FakeI2cBus::<32, 32, _>::new_blocking();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

        let mut new_brightness = [0; 192];

        new_brightness[20..22].fill(0xff);
        new_brightness[188..].fill(0xf0);

        is31fl3733.update_brightness(&new_brightness).unwrap();

        let mut new_leds = [0; 24];

        new_leds[10..12].fill(0xff);

        is31fl3733.update_leds(&new_leds).unwrap();

        assert_eq!(bus.write_data_as_ref(), EXPECTED_WRITE_DATA);
    }

    mod _async {
        use super::*;

        use lite_async_test::async_test;

        #[async_test]
        async fn init_test() {
            const EXPECTED_WRITE_DATA: [u8; 7] =
                [0xfe, 0xc5, 0xfd, 0x03, 0x11, 0x00, 0x01];
            const EXPECTED_READ_DATA: [u8; 1] = [0];

            let mut bus = FakeI2cBus::<32, 32, Async>::new_with_read_data(
                &EXPECTED_READ_DATA,
            );

            let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);
            is31fl3733.initialize().await.unwrap();

            assert_eq!(bus.write_data_as_ref(), EXPECTED_WRITE_DATA);
        }

        #[async_test]
        async fn configuration_test() {
            const EXPECTED_WRITE_DATA: &[u8] =
                &[0xfe, 0xc5, 0xfd, 0x03, 0x00, 0xaa, 0x00, 0xab];

            let mut bus = FakeI2cBus::<32, 32, Async>::new();

            let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

            is31fl3733.set_configuration(0xaa).await.unwrap();
            is31fl3733.set_configuration(0xaa).await.unwrap();
            is31fl3733.set_configuration(0xab).await.unwrap();

            assert_eq!(bus.write_data_as_ref(), EXPECTED_WRITE_DATA);
        }

        #[async_test]
        async fn global_current_control_test() {
            const EXPECTED_WRITE_DATA: &[u8] =
                &[0xfe, 0xc5, 0xfd, 0x03, 0x01, 0xaa, 0x01, 0xab];

            let mut bus = FakeI2cBus::<32, 32, Async>::new();

            let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

            is31fl3733.set_global_current_control(0xaa).await.unwrap();
            is31fl3733.set_global_current_control(0xaa).await.unwrap();
            is31fl3733.set_global_current_control(0xab).await.unwrap();

            assert_eq!(bus.write_data_as_ref(), EXPECTED_WRITE_DATA);
        }

        #[async_test]
        async fn state_update_test() {
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

            let mut bus = FakeI2cBus::<32, 32, _>::new_async();

            let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

            let mut new_brightness = [0; 192];

            new_brightness[20..22].fill(0xff);
            new_brightness[188..].fill(0xf0);

            is31fl3733.update_brightness(&new_brightness).await.unwrap();

            let mut new_leds = [0; 24];

            new_leds[10..12].fill(0xff);

            is31fl3733.update_leds(&new_leds).await.unwrap();

            assert_eq!(bus.write_data_as_ref(), EXPECTED_WRITE_DATA);
        }
    }
}
