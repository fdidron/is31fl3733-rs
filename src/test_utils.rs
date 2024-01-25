use crate::is31fl3733::{Async, Blocking, Mode};

#[derive(Debug)]
pub enum FakeI2cError<M: Mode> {
    Error,
    #[allow(non_camel_case_types)]
    _phantom(core::marker::PhantomData<M>),
}

impl embedded_hal_async::i2c::Error for FakeI2cError<Async> {
    fn kind(&self) -> embedded_hal_async::i2c::ErrorKind {
        embedded_hal_async::i2c::ErrorKind::Other
    }
}

impl embedded_hal::i2c::Error for FakeI2cError<Blocking> {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

impl<const N: usize, const M: usize> embedded_hal::i2c::I2c
    for FakeI2cBus<N, M, Blocking>
{
    fn transaction(
        &mut self,
        _address: embedded_hal_async::i2c::SevenBitAddress,
        operations: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        for operation in operations {
            match operation {
                embedded_hal_async::i2c::Operation::Write(write) => {
                    self.write_data
                        .extend_from_slice(write)
                        .map_err(|_| FakeI2cError::Error)?;
                }
                embedded_hal_async::i2c::Operation::Read(read) => {
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

impl<const N: usize, const M: usize> embedded_hal_async::i2c::I2c
    for FakeI2cBus<N, M, Async>
{
    async fn transaction(
        &mut self,
        _address: embedded_hal_async::i2c::SevenBitAddress,
        operations: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        for operation in operations {
            match operation {
                embedded_hal_async::i2c::Operation::Write(write) => {
                    self.write_data
                        .extend_from_slice(write)
                        .map_err(|_| FakeI2cError::Error)?;
                }
                embedded_hal_async::i2c::Operation::Read(read) => {
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

pub struct FakeI2cBus<const N: usize, const M: usize, MODE: Mode> {
    pub write_data: heapless::Vec<u8, N>,
    pub read_data: heapless::Vec<u8, M>,
    _phantom: core::marker::PhantomData<MODE>,
}

impl<const N: usize, const M: usize> embedded_hal_async::i2c::ErrorType
    for FakeI2cBus<N, M, Blocking>
{
    type Error = FakeI2cError<Blocking>;
}

impl<const N: usize, const M: usize> embedded_hal_async::i2c::ErrorType
    for FakeI2cBus<N, M, Async>
{
    type Error = FakeI2cError<Async>;
}

impl<const N: usize, const M: usize> FakeI2cBus<N, M, Blocking> {
    pub fn new_blocking() -> Self {
        Self {
            write_data: heapless::Vec::new(),
            read_data: heapless::Vec::new(),
            _phantom: core::marker::PhantomData,
        }
    }
}
impl<const N: usize, const M: usize> FakeI2cBus<N, M, Async> {
    pub fn new_async() -> Self {
        Self {
            write_data: heapless::Vec::new(),
            read_data: heapless::Vec::new(),
            _phantom: core::marker::PhantomData,
        }
    }
}
impl<const N: usize, const M: usize, MODE: Mode> FakeI2cBus<N, M, MODE> {
    pub fn new() -> Self {
        Self {
            write_data: heapless::Vec::new(),
            read_data: heapless::Vec::new(),
            _phantom: core::marker::PhantomData,
        }
    }

    pub fn new_with_read_data(read_data: &[u8]) -> Self {
        Self {
            write_data: heapless::Vec::new(),
            read_data: heapless::Vec::from_slice(read_data).unwrap(),
            _phantom: core::marker::PhantomData,
        }
    }

    pub fn write_data_as_ref(&self) -> &[u8] {
        self.write_data.as_slice()
    }

    #[allow(dead_code)]
    pub fn read_data_as_ref(&self) -> &[u8] {
        self.read_data.as_slice()
    }
}
