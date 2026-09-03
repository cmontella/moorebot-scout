use std::fmt;

/// Failure while decoding a ROS-serialized message.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DecodeError {
    UnexpectedEnd { needed: usize, remaining: usize },
    InvalidLength(u32),
    InvalidUtf8,
    InvalidValue(&'static str),
    TrailingBytes(usize),
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnexpectedEnd { needed, remaining } => write!(
                f,
                "message ended early: needed {needed} bytes, only {remaining} remain"
            ),
            Self::InvalidLength(length) => {
                write!(f, "message contains an unsupported length: {length}")
            }
            Self::InvalidUtf8 => f.write_str("message contains a non-UTF-8 ROS string"),
            Self::InvalidValue(description) => write!(f, "invalid value: {description}"),
            Self::TrailingBytes(count) => write!(f, "message has {count} trailing bytes"),
        }
    }
}

impl std::error::Error for DecodeError {}
