use crate::DecodeError;

pub(crate) struct Decoder<'a> {
    input: &'a [u8],
    offset: usize,
}

impl<'a> Decoder<'a> {
    pub(crate) fn new(input: &'a [u8]) -> Self {
        Self { input, offset: 0 }
    }

    pub(crate) fn read_u8(&mut self) -> Result<u8, DecodeError> {
        Ok(self.read_array::<1>()?[0])
    }

    pub(crate) fn read_i8(&mut self) -> Result<i8, DecodeError> {
        Ok(self.read_u8()? as i8)
    }

    pub(crate) fn read_u32(&mut self) -> Result<u32, DecodeError> {
        Ok(u32::from_le_bytes(self.read_array()?))
    }

    pub(crate) fn read_i32(&mut self) -> Result<i32, DecodeError> {
        Ok(i32::from_le_bytes(self.read_array()?))
    }

    pub(crate) fn read_u64(&mut self) -> Result<u64, DecodeError> {
        Ok(u64::from_le_bytes(self.read_array()?))
    }

    pub(crate) fn read_f32(&mut self) -> Result<f32, DecodeError> {
        Ok(f32::from_le_bytes(self.read_array()?))
    }

    pub(crate) fn read_f64(&mut self) -> Result<f64, DecodeError> {
        Ok(f64::from_le_bytes(self.read_array()?))
    }

    pub(crate) fn read_string_limited(
        &mut self,
        maximum_length: usize,
    ) -> Result<String, DecodeError> {
        let bytes = self.read_vec_limited(maximum_length)?;
        String::from_utf8(bytes).map_err(|_| DecodeError::InvalidUtf8)
    }

    pub(crate) fn read_vec_limited(
        &mut self,
        maximum_length: usize,
    ) -> Result<Vec<u8>, DecodeError> {
        let raw_length = self.read_u32()?;
        let length =
            usize::try_from(raw_length).map_err(|_| DecodeError::InvalidLength(raw_length))?;
        if length > maximum_length {
            return Err(DecodeError::InvalidLength(raw_length));
        }
        Ok(self.take(length)?.to_vec())
    }

    pub(crate) fn finish(self) -> Result<(), DecodeError> {
        let trailing = self.input.len() - self.offset;
        if trailing == 0 {
            Ok(())
        } else {
            Err(DecodeError::TrailingBytes(trailing))
        }
    }

    pub(crate) fn remaining(&self) -> usize {
        self.input.len() - self.offset
    }

    fn read_array<const N: usize>(&mut self) -> Result<[u8; N], DecodeError> {
        let mut output = [0_u8; N];
        output.copy_from_slice(self.take(N)?);
        Ok(output)
    }

    fn take(&mut self, length: usize) -> Result<&'a [u8], DecodeError> {
        let remaining = self.input.len() - self.offset;
        if remaining < length {
            return Err(DecodeError::UnexpectedEnd {
                needed: length,
                remaining,
            });
        }
        let start = self.offset;
        self.offset += length;
        Ok(&self.input[start..self.offset])
    }
}

#[cfg(feature = "ros1")]
pub(crate) fn write_u32(output: &mut Vec<u8>, value: u32) {
    output.extend_from_slice(&value.to_le_bytes());
}

pub(crate) fn write_f64(output: &mut Vec<u8>, value: f64) {
    output.extend_from_slice(&value.to_le_bytes());
}

#[cfg(feature = "ros1")]
pub(crate) fn write_string(output: &mut Vec<u8>, value: &str) {
    write_u32(output, value.len() as u32);
    output.extend_from_slice(value.as_bytes());
}

#[cfg(feature = "ros1")]
pub(crate) fn write_bytes(output: &mut Vec<u8>, value: &[u8]) {
    write_u32(output, value.len() as u32);
    output.extend_from_slice(value);
}
