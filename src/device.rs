pub trait RawDevice {
    type Error;

    async fn write(
        &mut self,
        address: u8,
        data: &[u8],
    ) -> Result<(), Self::Error>;

    async fn read(
        &mut self,
        address: u8,
        data: &mut [u8],
    ) -> Result<(), Self::Error>;

    async fn write_register(
        &mut self,
        address: u8,
        value: u8,
    ) -> Result<(), Self::Error> {
        self.write(address, &[value]).await?;

        Ok(())
    }

    async fn read_register(&mut self, address: u8) -> Result<u8, Self::Error> {
        let mut buffer = [0; 1];

        self.read(address, &mut buffer).await?;

        Ok(buffer[0])
    }
}

impl<T> RawDevice for &mut T
where
    T: RawDevice,
{
    type Error = T::Error;

    async fn write(
        &mut self,
        address: u8,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        T::write(self, address, data).await
    }

    async fn read(
        &mut self,
        address: u8,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        T::read(self, address, data).await
    }
}
