use crate::device::RawDevice;
use crate::is31fl3733::IS31FL3733;

use embedded_hal_async::i2c::I2c;

impl<BUS: I2c> IS31FL3733<I2cAdapter<BUS>> {
    /// Create a new IS31FL3733 driver
    /// # Arguments
    /// * `i2c` - The I2C bus to use
    /// * `address` - The I2C address of the device
    ///
    /// # Returns
    /// A new IS31FL3733 driver
    pub fn new_with_i2c_bus(i2c: BUS, address: u8) -> Self {
        Self::new(I2cAdapter::new(i2c, address))
    }
}

pub struct I2cAdapter<BUS: I2c> {
    i2c: BUS,
    address: u8,
}

impl<BUS: I2c> I2cAdapter<BUS> {
    pub fn new(i2c: BUS, address: u8) -> Self {
        Self { i2c, address }
    }

    #[cfg(test)]
    pub(crate) fn into_inner(self) -> BUS {
        self.i2c
    }
}

impl<BUS: I2c> RawDevice for I2cAdapter<BUS> {
    type Error = BUS::Error;

    async fn write(
        &mut self,
        address: u8,
        data: &[u8],
    ) -> Result<(), BUS::Error> {
        self.i2c
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
        self.i2c
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
}
