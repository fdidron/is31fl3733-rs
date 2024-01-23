use embedded_hal_async::i2c::{Error, ErrorKind, ErrorType};

#[derive(Debug)]
pub enum FakeI2cError {
    Error,
}
impl Error for FakeI2cError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

pub struct FakeI2cBus<const N: usize, const M: usize> {
    pub write_data: heapless::Vec<u8, N>,
    pub read_data: heapless::Vec<u8, M>,
}

impl<const N: usize, const M: usize> ErrorType for FakeI2cBus<N, M> {
    type Error = FakeI2cError;
}

impl<const N: usize, const M: usize> FakeI2cBus<N, M> {
    pub fn new() -> Self {
        Self {
            write_data: heapless::Vec::new(),
            read_data: heapless::Vec::new(),
        }
    }

    pub fn new_with_read_data(read_data: &[u8]) -> Self {
        Self {
            write_data: heapless::Vec::new(),
            read_data: heapless::Vec::from_slice(read_data).unwrap(),
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
