use crate::codec::write_f64;
use std::fmt;

/// Velocity expressed with standard ROS mobile-base axis semantics.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Velocity {
    /// Forward/backward speed. Positive is forward.
    pub forward_mps: f64,
    /// Left/right speed. Positive is left.
    pub lateral_mps: f64,
    /// Rotation speed. Positive is counter-clockwise.
    pub yaw_rps: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionLimits {
    pub max_forward_mps: f64,
    pub max_lateral_mps: f64,
    pub max_yaw_rps: f64,
}

impl Default for MotionLimits {
    fn default() -> Self {
        Self {
            max_forward_mps: 0.47,
            max_lateral_mps: 0.2,
            max_yaw_rps: 2.9,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ScoutTwist {
    pub linear_x: f64,
    pub linear_y: f64,
    pub linear_z: f64,
    pub angular_x: f64,
    pub angular_y: f64,
    pub angular_z: f64,
}

impl ScoutTwist {
    pub const fn zero() -> Self {
        Self {
            linear_x: 0.0,
            linear_y: 0.0,
            linear_z: 0.0,
            angular_x: 0.0,
            angular_y: 0.0,
            angular_z: 0.0,
        }
    }

    /// Encode this value as a ROS 1 `geometry_msgs/Twist` body.
    pub fn encode_ros(self) -> Vec<u8> {
        let mut output = Vec::with_capacity(6 * std::mem::size_of::<f64>());
        for value in [
            self.linear_x,
            self.linear_y,
            self.linear_z,
            self.angular_x,
            self.angular_y,
            self.angular_z,
        ] {
            write_f64(&mut output, value);
        }
        output
    }
}

impl Velocity {
    /// Clamp a standard velocity command and map it to the Scout's swapped
    /// linear axes (`linear.y` is forward and `linear.x` is lateral).
    pub fn to_scout_twist(self, limits: MotionLimits) -> Result<ScoutTwist, MotionError> {
        validate_finite("forward velocity", self.forward_mps)?;
        validate_finite("lateral velocity", self.lateral_mps)?;
        validate_finite("yaw velocity", self.yaw_rps)?;
        validate_limit("forward limit", limits.max_forward_mps)?;
        validate_limit("lateral limit", limits.max_lateral_mps)?;
        validate_limit("yaw limit", limits.max_yaw_rps)?;

        Ok(ScoutTwist {
            linear_x: self
                .lateral_mps
                .clamp(-limits.max_lateral_mps, limits.max_lateral_mps),
            linear_y: self
                .forward_mps
                .clamp(-limits.max_forward_mps, limits.max_forward_mps),
            angular_z: self.yaw_rps.clamp(-limits.max_yaw_rps, limits.max_yaw_rps),
            ..ScoutTwist::zero()
        })
    }
}

fn validate_finite(field: &'static str, value: f64) -> Result<(), MotionError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(MotionError::NotFinite(field))
    }
}

fn validate_limit(field: &'static str, value: f64) -> Result<(), MotionError> {
    validate_finite(field, value)?;
    if value > 0.0 {
        Ok(())
    } else {
        Err(MotionError::NonPositiveLimit(field))
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotionError {
    NotFinite(&'static str),
    NonPositiveLimit(&'static str),
}

impl fmt::Display for MotionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotFinite(field) => write!(f, "{field} must be finite"),
            Self::NonPositiveLimit(field) => write!(f, "{field} must be greater than zero"),
        }
    }
}

impl std::error::Error for MotionError {}
