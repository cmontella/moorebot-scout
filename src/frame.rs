use crate::{codec::Decoder, DecodeError};
use std::fmt;

/// Maximum encoded payload accepted from a Scout media frame.
///
/// The limit is checked before decoder-owned memory is allocated or copied.
pub const MAX_FRAME_DATA_BYTES: usize = 16 * 1024 * 1024;

/// The payload type used by the Scout's `roller_eye/frame` message.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StreamType {
    H264,
    Jpeg,
    Aac,
    Unknown(i8),
}

impl From<i8> for StreamType {
    fn from(value: i8) -> Self {
        match value {
            0 => Self::H264,
            1 => Self::Jpeg,
            2 => Self::Aac,
            other => Self::Unknown(other),
        }
    }
}

impl fmt::Display for StreamType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::H264 => f.write_str("H.264"),
            Self::Jpeg => f.write_str("JPEG"),
            Self::Aac => f.write_str("AAC"),
            Self::Unknown(value) => write!(f, "unknown ({value})"),
        }
    }
}

/// One ROS-serialized media frame from the Scout.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ScoutFrame {
    pub sequence: u32,
    /// Monotonic robot timestamp in milliseconds.
    pub timestamp_ms: u64,
    pub session: u32,
    pub stream_type: StreamType,
    pub original_sequence: u32,
    pub parameters: [i32; 4],
    pub data: Vec<u8>,
}

impl ScoutFrame {
    /// Decode the ROS 1 wire representation of `roller_eye/frame`.
    pub fn decode_ros(input: &[u8]) -> Result<Self, DecodeError> {
        let mut decoder = Decoder::new(input);
        let sequence = decoder.read_u32()?;
        let timestamp_ms = decoder.read_u64()?;
        let session = decoder.read_u32()?;
        let stream_type = StreamType::from(decoder.read_i8()?);
        let original_sequence = decoder.read_u32()?;
        let parameters = [
            decoder.read_i32()?,
            decoder.read_i32()?,
            decoder.read_i32()?,
            decoder.read_i32()?,
        ];
        let data = decoder.read_vec_limited(MAX_FRAME_DATA_BYTES)?;
        decoder.finish()?;

        Ok(Self {
            sequence,
            timestamp_ms,
            session,
            stream_type,
            original_sequence,
            parameters,
            data,
        })
    }

    pub fn video_dimensions(&self) -> Option<(u32, u32)> {
        match self.stream_type {
            StreamType::H264 | StreamType::Jpeg
                if self.parameters[0] > 0 && self.parameters[1] > 0 =>
            {
                Some((self.parameters[0] as u32, self.parameters[1] as u32))
            }
            _ => None,
        }
    }

    pub fn is_keyframe(&self) -> Option<bool> {
        match self.stream_type {
            StreamType::H264 => Some(self.parameters[2] != 0),
            _ => None,
        }
    }

    pub fn audio_format(&self) -> Option<AudioFormat> {
        match self.stream_type {
            StreamType::Aac => {
                let sample_rate_hz = u32::try_from(self.parameters[0]).ok()?;
                let bit_width = u16::try_from(self.parameters[1]).ok()?;
                let channels = u16::try_from(self.parameters[2]).ok()?;

                (sample_rate_hz > 0 && bit_width > 0 && channels > 0).then_some(AudioFormat {
                    sample_rate_hz,
                    bit_width,
                    channels,
                })
            }
            _ => None,
        }
    }

    pub fn has_jpeg_markers(&self) -> bool {
        self.data.starts_with(&[0xff, 0xd8]) && self.data.ends_with(&[0xff, 0xd9])
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AudioFormat {
    pub sample_rate_hz: u32,
    pub bit_width: u16,
    pub channels: u16,
}
