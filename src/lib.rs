//! Protocol and ROS 1 support for the Moorebot Scout robot.
//!
//! The protocol modules do not require ROS to be installed. The optional
//! `ros1` feature (enabled by default) adds a client that talks directly to the
//! ROS master already running on the Scout.

mod codec;
mod error;

pub mod frame;
pub mod motion;
pub mod sensors;
pub mod services;
pub mod topics;

#[cfg(feature = "ros1")]
pub mod ros1;

pub use error::DecodeError;
