pub trait RawDevice {
    type Error;

    fn write(&mut self, address: u8, data: &[u8]) -> Result<(), Self::Error>;

    fn read(&mut self, address: u8, data: &mut [u8])
        -> Result<(), Self::Error>;

    fn write_register(
        &mut self,
        address: u8,
        value: u8,
    ) -> Result<(), Self::Error> {
        self.write(address, &[value])?;

        Ok(())
    }

    fn read_register(&mut self, address: u8) -> Result<u8, Self::Error> {
        let mut buffer = [0; 1];

        self.read(address, &mut buffer)?;

        Ok(buffer[0])
    }
}

impl<T> RawDevice for &mut T
where
    T: RawDevice,
{
    type Error = T::Error;

    fn write(&mut self, address: u8, data: &[u8]) -> Result<(), Self::Error> {
        T::write(self, address, data)
    }

    fn read(
        &mut self,
        address: u8,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        T::read(self, address, data)
    }
}
