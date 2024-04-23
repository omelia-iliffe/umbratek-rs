//! Umbratek-rs provides support for the Umbratek-Adra actuators
//!
//! # Optional features
//!
//! You can enable the `log` feature to have the library use `log::trace!()` to log all sent instructions and received replies.

pub use serial2;

pub use bus::{Bus, Response};
pub use error::InvalidChecksum;
pub use error::InvalidHeaderPrefix;
pub use error::InvalidInstruction;
pub use error::InvalidMessage;
pub use error::InvalidPacketId;
pub use error::InvalidParameterCount;
pub use error::MotorError;
pub use error::ReadError;
pub use error::TransferError;
pub use error::WriteError;
pub use protocol::*;
pub use protocol::registers;

// #[macro_use]
// mod _log;
mod checksum;
mod error;

mod bus;
mod commands;

pub mod protocol;

