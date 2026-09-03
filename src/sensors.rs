use crate::{DecodeError, codec::Decoder};

/// Maximum accepted byte length for a ROS sensor header frame identifier.
pub const MAX_FRAME_ID_BYTES: usize = 4 * 1024;

/// Maximum accepted number of integers in the Scout battery status vector.
pub const MAX_BATTERY_STATUS_VALUES: usize = 64;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RosHeader {
    pub sequence: u32,
    pub stamp: RosTime,
    pub frame_id: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RosTime {
    pub seconds: u32,
    pub nanoseconds: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ImuSample {
    pub header: RosHeader,
    pub orientation: Quaternion,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: Vector3,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: Vector3,
    pub linear_acceleration_covariance: [f64; 9],
}

impl ImuSample {
    pub fn decode_ros(input: &[u8]) -> Result<Self, DecodeError> {
        let mut decoder = Decoder::new(input);
        let sample = Self {
            header: decode_header(&mut decoder)?,
            orientation: decode_quaternion(&mut decoder)?,
            orientation_covariance: decode_covariance(&mut decoder)?,
            angular_velocity: decode_vector3(&mut decoder)?,
            angular_velocity_covariance: decode_covariance(&mut decoder)?,
            linear_acceleration: decode_vector3(&mut decoder)?,
            linear_acceleration_covariance: decode_covariance(&mut decoder)?,
        };
        decoder.finish()?;
        Ok(sample)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct RangeSample {
    pub header: RosHeader,
    pub radiation_type: u8,
    pub field_of_view_rad: f32,
    pub min_range_m: f32,
    pub max_range_m: f32,
    pub range_m: f32,
}

impl RangeSample {
    pub fn decode_ros(input: &[u8]) -> Result<Self, DecodeError> {
        let mut decoder = Decoder::new(input);
        let sample = Self {
            header: decode_header(&mut decoder)?,
            radiation_type: decoder.read_u8()?,
            field_of_view_rad: decoder.read_f32()?,
            min_range_m: decoder.read_f32()?,
            max_range_m: decoder.read_f32()?,
            range_m: decoder.read_f32()?,
        };
        decoder.finish()?;
        Ok(sample)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct IlluminanceSample {
    pub header: RosHeader,
    pub illuminance: f64,
    pub variance: f64,
}

impl IlluminanceSample {
    pub fn decode_ros(input: &[u8]) -> Result<Self, DecodeError> {
        let mut decoder = Decoder::new(input);
        let sample = Self {
            header: decode_header(&mut decoder)?,
            illuminance: decoder.read_f64()?,
            variance: decoder.read_f64()?,
        };
        decoder.finish()?;
        Ok(sample)
    }

    /// The Scout documentation describes its reading as two packed 16-bit
    /// light channels. This returns `(CH0, CH1)` when the value is integral.
    pub fn packed_channels(&self) -> Option<(u16, u16)> {
        if !self.illuminance.is_finite()
            || self.illuminance < 0.0
            || self.illuminance > u32::MAX as f64
            || self.illuminance.fract() != 0.0
        {
            return None;
        }
        let packed = self.illuminance as u32;
        Some(((packed >> 16) as u16, packed as u16))
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BatteryState {
    Charging,
    Discharging,
    Full,
    Unknown(i32),
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BatteryStatus {
    pub state: BatteryState,
    pub percentage: u8,
    pub externally_powered: bool,
    pub raw: Vec<i32>,
}

impl BatteryStatus {
    pub fn decode_ros(input: &[u8]) -> Result<Self, DecodeError> {
        let mut decoder = Decoder::new(input);
        let raw_length = decoder.read_u32()?;
        let length =
            usize::try_from(raw_length).map_err(|_| DecodeError::InvalidLength(raw_length))?;
        if length > MAX_BATTERY_STATUS_VALUES {
            return Err(DecodeError::InvalidLength(raw_length));
        }
        if length < 3 {
            return Err(DecodeError::InvalidValue(
                "battery status must have at least three values",
            ));
        }
        let required_bytes = length
            .checked_mul(std::mem::size_of::<i32>())
            .ok_or(DecodeError::InvalidLength(raw_length))?;
        if decoder.remaining() < required_bytes {
            return Err(DecodeError::UnexpectedEnd {
                needed: required_bytes,
                remaining: decoder.remaining(),
            });
        }
        let mut raw = Vec::with_capacity(length);
        for _ in 0..length {
            raw.push(decoder.read_i32()?);
        }
        decoder.finish()?;

        let percentage = u8::try_from(raw[1])
            .ok()
            .filter(|value| *value <= 100)
            .ok_or(DecodeError::InvalidValue(
                "battery percentage must be between 0 and 100",
            ))?;
        let state = match raw[0] {
            0 => BatteryState::Charging,
            1 => BatteryState::Discharging,
            2 => BatteryState::Full,
            other => BatteryState::Unknown(other),
        };

        Ok(Self {
            state,
            percentage,
            externally_powered: raw[2] != 0,
            raw,
        })
    }
}

fn decode_header(decoder: &mut Decoder<'_>) -> Result<RosHeader, DecodeError> {
    Ok(RosHeader {
        sequence: decoder.read_u32()?,
        stamp: RosTime {
            seconds: decoder.read_u32()?,
            nanoseconds: decoder.read_u32()?,
        },
        frame_id: decoder.read_string_limited(MAX_FRAME_ID_BYTES)?,
    })
}

fn decode_vector3(decoder: &mut Decoder<'_>) -> Result<Vector3, DecodeError> {
    Ok(Vector3 {
        x: decoder.read_f64()?,
        y: decoder.read_f64()?,
        z: decoder.read_f64()?,
    })
}

fn decode_quaternion(decoder: &mut Decoder<'_>) -> Result<Quaternion, DecodeError> {
    Ok(Quaternion {
        x: decoder.read_f64()?,
        y: decoder.read_f64()?,
        z: decoder.read_f64()?,
        w: decoder.read_f64()?,
    })
}

fn decode_covariance(decoder: &mut Decoder<'_>) -> Result<[f64; 9], DecodeError> {
    let mut covariance = [0.0; 9];
    for value in &mut covariance {
        *value = decoder.read_f64()?;
    }
    Ok(covariance)
}
