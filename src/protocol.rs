use std::io::Read;
use byteorder::{ReadBytesExt, BE};
use num_derive::FromPrimitive;

macro_rules! register {
    (@register $reg:ident : $addr:expr, $type:ty) => {
        #[derive(Debug, Clone, PartialEq)]
        pub struct $reg($type);

        impl $reg {
            pub fn new(value: $type) -> Self {
                $reg(value)
            }
            pub fn value(&self) -> $type {
                self.0
            }
        }

        impl Register for $reg {
            fn address() -> u8 {
                $addr
            }
            fn size() -> u8 {
                std::mem::size_of::<$type>() as u8
            }
        }
    };
    (@writable $reg:ident : $addr:expr, $type:ty) => {
        impl Writable for $reg {
            fn as_bytes(&self) -> Vec<u8> {
                self.0.into_bytes()
            }
        }
    };
    (@readable $reg:ident : $addr:expr, $type:ty) => {
        impl Readable for $reg {
            fn try_from_bytes(bytes: &[u8]) -> Result<Self, std::io::Error> {
                let value = <$type>::try_from_bytes(bytes)?;
                Ok($reg(value))
            }
        }

    };
    ($reg:ident : $addr:expr, $type:ty, RW) => {
        register!(@register $reg : $addr, $type);
        register!(@writable $reg : $addr, $type);
        register!(@readable $reg : $addr, $type);
    };
    ($reg:ident : $addr:expr, $type:ty, R) => {
        register!(@register $reg : $addr, $type);
        register!(@readable $reg : $addr, $type);
    };
    ($reg:ident : $addr:expr, $type:ty, W) => {
        register!(@register $reg : $addr, $type);
        register!(@writable $reg : $addr, $type);
    };
}


pub trait Register {
    fn address() -> u8;
    fn size() -> u8;
}

pub trait Writable {
    fn as_bytes(&self) -> Vec<u8>;
}

pub trait Readable {
    fn try_from_bytes(bytes: &[u8]) -> Result<Self, std::io::Error> where Self: Sized;
}

pub trait TryFromBytes {
    fn try_from_bytes(bytes: &[u8]) -> Result<Self, std::io::Error> where Self: Sized;
}

pub trait IntoBytes {
    fn into_bytes(&self) -> Vec<u8>;
}

pub mod registers {
    use super::*;
    register!(UUID: 0x01, [u8; 12], R);
    register!(SWVersion: 0x02, [u8; 12], R);
    register!(HWVersion: 0x03, [u8; 12], R);
    register!(MultiVersion: 0x04, [u8; 12], R);
    register!(MechRatio: 0x05, f32, RW);
    register!(ComID: 0x08, u8, W);
    register!(ComBaud: 0x09, u32, W);
    register!(ResetErr: 0x0C, (), W);
    register!(ResetDriver: 0x0D, (), W);
    register!(EraseParm: 0x0E, (), W);
    register!(SavedParm: 0x0F, (), W);

    register!(ElecRatio: 0x10, f32, RW);
    register!(MotionDir: 0x11, bool, RW);
    register!(IWDGCyc: 0x12, u32, RW);
    register!(TempLimit: 0x18, Limit<i8>, RW);
    register!(VoltLimit: 0x19, Limit<u8>, RW);
    register!(CurrLimit: 0x1A, f32, RW);
// register!(BrakeDelay: 0x1B, u32, RW);
// register!(BrakePWM: 0x1F, u8, RW);

    register!(MotionMode: 0x20, MotionModes, RW);
    register!(MotionEnable: 0x21, bool, RW);
    register!(BrakeEnable: 0x22, bool, RW);
    register!(TempDriver: 0x28, f32, R);
    register!(TempMotor: 0x29, f32, R);
    register!(BusVolt: 0x2A, f32, R);
    register!(BusCurr: 0x2B, f32, R);
    register!(BattVolt: 0x2C, f32, R);
    register!(ErrorCode: 0x2F, ErrorCodes, R);

    register!(PositionTarget: 0x30, f32, RW);
    register!(PositionCurrent: 0x31, f32, R);
    register!(PositionLimitMax: 0x32, f32, RW);
    register!(PositionLimitMin: 0x33, f32, RW);
    register!(PositionLimitDiff: 0x34, f32, RW);
    register!(PositionPIDP: 0x35, f32, RW);
    register!(PositionSmoothCyc: 0x36, u8, RW);
    // register!(PositionADRCParam: 0x39, f32, RW);
    register!(PositionCalZero: 0x3F, u8, W);

    register!(VelocityTarget: 0x40, f32, RW);
    register!(VelocityCurrent: 0x41, f32, R);
    register!(VelocityLimitMax: 0x42, f32, RW);
    register!(VelocityLimitMin: 0x43, f32, RW);
    register!(VelocityLimitDiff: 0x44, f32, RW);
    register!(VelocityPIDP: 0x45, f32, RW);
    register!(VelocityPIDI: 0x46, f32, RW);
    register!(VelocitySmoothCyc: 0x47, u8, RW);
// register!(VelocityADRCParam: 0x49, f32, RW);

    register!(TorqueTarget: 0x50, f32, RW);
    register!(TorqueCurrent: 0x51, f32, R);
    register!(TorqueLimitMax: 0x52, f32, RW);
    register!(TorqueLimitMin: 0x53, f32, RW);
    register!(TorqueLimitDiff: 0x54, f32, RW);
    register!(TorquePIDP: 0x55, f32, RW);
    register!(TorquePIDI: 0x56, f32, RW);
    register!(TorqueSmoothCyc: 0x57, u8, RW);
// register!(TorqueADRCParam: 0x59, f32, RW);
}

#[allow(non_camel_case_types)] // TODO: Fix cases
#[derive(Debug, FromPrimitive, PartialEq, Clone, Copy)]
pub enum ErrorCodes {
    NO_ERROR = 0,
    FLASH_ERROR = 1,
    PRE_DRIVER_COMMUNICATION_ERROR = 2,
    MULTI_TURN_COMMUNICATION_ERROR = 3,
    CURRENT_SAMPLING_ERROR = 4,
    EEPROM_ERROR = 5,
    STORAGE_DATA_ERROR = 6,
    BATTERY_VOLTAGE_TOO_LOW = 7,
    ENCODER_CALCULATE_ERROR = 8,  // Keep motor stationary during power on,
    ENCODER_ERROR1 = 9,
    ENCODER_ERROR2 = 10,
    ELECTRICAL_ANGLE_ERROR = 11,
    ENCODER_LINEAR_ERROR = 12,
    MULTI_TURN_CALIBRATE_ERROR = 13,
    ZERO_CALIBRATE_ERROR = 14,
    HALL_CALIBRATION_ERROR = 15,
    ENCODER_CALCULATE_ERROR1 = 16,
    ENCODER_CALCULATE_ERROR2 = 17,
    MU_COMMUNICATION_ERROR = 18,
    MU_STATUS_ERROR = 19,
    PRE_DRIVE_ALARM = 21,
    MULTI_TURN_CALCULATE_ERROR = 22,
    CURRENT_TEMPERATURE_OUT_OF_RANGE = 23,
    CURRENT_VOLTAGE_OUT_OF_RANGE = 24,
    PHASE_A_ALARM = 25,
    PHASE_B_ALARM = 26,
    PHASE_C_ALARM = 27,
    POSITION_FOLLOWING_ALARM = 31, // Auto cleared when torque enabled
    CURRENT_SPEED_OUT_OF_RANGE = 32,
    CURRENT_POWER_OUT_OF_RANGE = 33,
    TARGET_POSITION_OUT_OF_RANGE = 34,
    TARGET_SPEED_OUT_OF_RANGE = 35,
    TARGET_TORQUE_OUT_OF_RANGE = 36,
    CURRENT_POSITION_OUT_OF_RANGE = 37,
    REGISTER_ADDRESS_ERROR = 40,
    REGISTER_VALUE_ERROR = 41,
    BROADCAST_READ_COMMAND_TIMED_OUT = 42,
    ENCODER_SPEED_ERROR = 91
}

impl TryFromBytes for ErrorCodes {
    fn try_from_bytes(bytes: &[u8]) -> Result<Self, std::io::Error> {
        let mut rdr = std::io::Cursor::new(bytes);
        let value = rdr.read_u8()?;
        match num_traits::FromPrimitive::from_u8(value) {
            Some(v) => Ok(v),
            None => Err(std::io::Error::new(std::io::ErrorKind::InvalidData, "Invalid Error Code")),
        }
    }
}

impl IntoBytes for ErrorCodes {
    fn into_bytes(&self) -> Vec<u8> {
        vec![*self as u8]
    }
}

#[derive(Debug, FromPrimitive, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum MotionModes {
    Position = 1,
    Velocity = 2,
    Torque = 3,
}
impl IntoBytes for MotionModes {
    fn into_bytes(&self) -> Vec<u8> {
        vec![*self as u8]
    }
}

impl TryFromBytes for MotionModes {
    fn try_from_bytes(bytes: &[u8]) -> Result<Self, std::io::Error> {
        let mut rdr = std::io::Cursor::new(bytes);
        let value = rdr.read_u8()?;
        match num_traits::FromPrimitive::from_u8(value) {
            Some(v) => Ok(v),
            None => Err(std::io::Error::new(std::io::ErrorKind::InvalidData, "Invalid Motion Mode")),
        }
    }
}

impl<const N: usize> TryFromBytes for [u8; N] {
    fn try_from_bytes(bytes: &[u8]) -> Result<Self, std::io::Error> where Self: Sized {
        let mut buf = [0; N];
        let mut rdr = std::io::Cursor::new(bytes);
        rdr.read_exact(&mut buf)?;
        Ok(buf)
    }
}

impl<const N: usize> IntoBytes for [u8; N] {
    fn into_bytes(&self) -> Vec<u8> {
        self.to_vec()
    }
}

impl TryFromBytes for bool {
    fn try_from_bytes(bytes: &[u8]) -> Result<Self, std::io::Error> {
        let mut rdr = std::io::Cursor::new(bytes);
        let value = rdr.read_u8()?;
        Ok(value != 0)
    }
}

impl IntoBytes for bool {
    fn into_bytes(&self) -> Vec<u8> {
        vec![*self as u8]
    }
}
impl TryFromBytes for i8 {
    fn try_from_bytes(bytes: &[u8]) -> Result<Self, std::io::Error> {
        let mut rdr = std::io::Cursor::new(bytes);
        let value = rdr.read_i8()?;
        Ok(value)
    }
}

impl IntoBytes for i8 {
    fn into_bytes(&self) -> Vec<u8> {
        vec![*self as u8]
    }
}
impl TryFromBytes for u8 {
    fn try_from_bytes(bytes: &[u8]) -> Result<Self, std::io::Error> {
        let mut rdr = std::io::Cursor::new(bytes);
        let value = rdr.read_u8()?;
        Ok(value)
    }
}

impl IntoBytes for u8 {
    fn into_bytes(&self) -> Vec<u8> {
        vec![*self]
    }
}

impl TryFromBytes for u16 {
    fn try_from_bytes(bytes: &[u8]) -> Result<Self, std::io::Error> {
        let mut rdr = std::io::Cursor::new(bytes);
        let value = rdr.read_u16::<BE>()?;
        Ok(value)
    }
}

impl IntoBytes for u16 {
    fn into_bytes(&self) -> Vec<u8> {
        self.to_be_bytes().to_vec()
    }
}
impl TryFromBytes for u32 {
    fn try_from_bytes(bytes: &[u8]) -> Result<Self, std::io::Error> {
        let mut rdr = std::io::Cursor::new(bytes);
        let value = rdr.read_u32::<BE>()?;
        Ok(value)
    }
}
impl IntoBytes for u32 {
    fn into_bytes(&self) -> Vec<u8> {
        self.to_be_bytes().to_vec()
    }
}

impl TryFromBytes for f32 {
    fn try_from_bytes(bytes: &[u8]) -> Result<Self, std::io::Error> {
        let mut rdr = std::io::Cursor::new(bytes);
        let value = rdr.read_f32::<BE>()?;
        Ok(value)
    }
}

impl IntoBytes for f32 {
    fn into_bytes(&self) ->Vec<u8> {
        self.to_be_bytes().to_vec()
    }
}

impl TryFromBytes for () {
    fn try_from_bytes(_bytes: &[u8]) -> Result<Self, std::io::Error> {
        Ok(())
    }
}

impl IntoBytes for () {
    fn into_bytes(&self) -> Vec<u8> {
        vec![]
    }
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Limit<T> {
    min: T,
    max: T,
}

impl<T> TryFromBytes for Limit<T> where T:TryFromBytes{
    fn try_from_bytes(bytes: &[u8]) -> Result<Self, std::io::Error> where Self: Sized {
        let min = T::try_from_bytes(&bytes[..std::mem::size_of::<T>()])?;
        let max = T::try_from_bytes(&bytes[std::mem::size_of::<T>()..])?;
        Ok(Limit { min, max })
    }
}

impl<T> IntoBytes for Limit<T> where T:IntoBytes{
    fn into_bytes(&self) -> Vec<u8> {
        let mut bytes = self.min.into_bytes();
        bytes.extend(self.max.into_bytes());
        bytes
    }
}