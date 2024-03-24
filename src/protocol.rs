use std::io::Read;
use byteorder::{ReadBytesExt, BE};
use num_derive::FromPrimitive;
use zerocopy::AsBytes;

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
            fn address() -> RegisterAddress {
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

#[repr(u8)]
#[derive(Debug, Clone, Copy, AsBytes, FromPrimitive, PartialEq, Eq, Hash)]
#[allow(non_camel_case_types)] // TODO: Fix cases
pub enum RegisterAddress {
    UUID = 0x01,
    SW_VERSION = 0x02,
    HW_VERSION = 0x03,
    MULTI_VERSION = 0x04,
    MECH_RATIO = 0x05,
    COM_ID = 0x08,
    COM_BAUD = 0x09,
    RESET_ERR = 0x0C,
    RESET_DRIVER = 0x0D,
    ERASE_PARM = 0x0E,
    SAVED_PARM = 0x0F,
    ELEC_RATIO = 0x10,
    MOTION_DIR = 0x11,
    IWDG_CYC = 0x12,
    TEMP_LIMIT = 0x18,
    VOLT_LIMIT = 0x19,
    CURR_LIMIT = 0x1A,
    BRAKE_DELAY = 0x1B,
    BRAKE_PWM = 0x1F,
    MOTION_MODE = 0x20,
    MOTION_ENABLE = 0x21,
    BRAKE_ENABLE = 0x22,
    TEMP_DRIVER = 0x28,
    TEMP_MOTOR = 0x29,
    BUS_VOLT = 0x2A,
    BUS_CURR = 0x2B,
    BATT_VOLT = 0x2C,
    ERROR_CODE = 0x2F,
    POS_TARGET = 0x30,
    POS_CURRENT = 0x31,
    POS_LIMIT_MAX = 0x32,
    POS_LIMIT_MIN = 0x33,
    POS_LIMIT_DIFF = 0x34,
    POS_PIDP = 0x35,
    POS_SMOOTH_CYC = 0x36,
    POS_ADRC_PARAM = 0x39,
    POS_CAL_ZERO = 0x3F,
    VEL_TARGET = 0x40,
    VEL_CURRENT = 0x41,
    VEL_LIMIT_MAX = 0x42,
    VEL_LIMIT_MIN = 0x43,
    VEL_LIMIT_DIFF = 0x44,
    VEL_PIDP = 0x45,
    VEL_PIDI = 0x46,
    VEL_SMOOTH_CYC = 0x47,
    VEL_ADRC_PARAM = 0x49,
    TAU_TARGET = 0x50,
    TAU_CURRENT = 0x51,
    TAU_LIMIT_MAX = 0x52,
    TAU_LIMIT_MIN = 0x53,
    TAU_LIMIT_DIFF = 0x54,
    TAU_PIDP = 0x55,
    TAU_PIDI = 0x56,
    TAU_SMOOTH_CYC = 0x57,
    TAU_ADRC_PARAM = 0x59,
    CPOS_TARGET = 0x60,
    CTAU_TARGET = 0x61,
    CPOSTAU_TARGET = 0x62,
    CPOSVEL_TARGET = 0x64,
    SPOSTAU_CURRENT = 0x68,
    CPOSTAU_CURRENT = 0x69
// 
// # Registers read via ADRC, VALUE IS NOT THE ADDRESS
// POS_ACCELERATION = 1000
// POS_VELOCITY_COMP = 1001
// POS_INTERFERENCE_COMP = 1003
// POS_TRACKING_TIME = 1004
// POS_TRACKING_DIFF = 1005
// VEL_ACCELERATION = 1006
// VEL_VELOCITY_COMP = 1007
// VEL_INTERFERENCE_COMP = 1008
// VEL_TRACKING_TIME = 1009
// VEL_TRACKING_DIFF = 1010
// TAU_ACCELERATION = 1011
// TAU_VELOCITY_COMP = 1012
// TAU_INTERFERENCE_COMP = 1013
// TAU_TRACKING_TIME = 1014
// TAU_TRACKING_DIFF = 1015
}

pub trait Register {
    fn address() -> RegisterAddress;
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
    register!(UUID: RegisterAddress::UUID, [u8; 12], R);
    register!(SWVersion: RegisterAddress::SW_VERSION, [u8; 12], R);
    register!(HWVersion: RegisterAddress::HW_VERSION, [u8; 12], R);
    register!(MultiVersion: RegisterAddress::MULTI_VERSION, [u8; 12], R);
    register!(MechRatio: RegisterAddress::MECH_RATIO, f32, RW);
    register!(ComID: RegisterAddress::COM_ID, u8, W);
    register!(ComBaud: RegisterAddress::COM_BAUD, u32, W);
    register!(ResetErr: RegisterAddress::RESET_ERR, (), W);
    register!(ResetDriver: RegisterAddress::RESET_DRIVER, (), W);
    register!(EraseParm: RegisterAddress::ERASE_PARM, (), W);
    register!(SavedParm: RegisterAddress::SAVED_PARM, (), W);

    register!(ElecRatio: RegisterAddress::ELEC_RATIO, f32, RW);
    register!(MotionDir: RegisterAddress::MOTION_DIR, bool, RW);
    register!(IWDGCyc: RegisterAddress::IWDG_CYC, u32, RW);
    register!(TempLimit: RegisterAddress::TEMP_LIMIT, Limit<i8>, RW);
    register!(VoltLimit: RegisterAddress::VOLT_LIMIT, Limit<u8>, RW);
    register!(CurrLimit: RegisterAddress::CURR_LIMIT, f32, RW);
// register!(BrakeDelay: RegisterAddress::BRAKE_DELAY, u32, RW);
// register!(BrakePWM: RegisterAddress::BRAKE_PWM, u8, RW);

    register!(MotionMode: RegisterAddress::MOTION_MODE, MotionModes, RW);
    register!(MotionEnable: RegisterAddress::MOTION_ENABLE, bool, RW);
    register!(BrakeEnable: RegisterAddress::BRAKE_ENABLE, bool, RW);
    register!(TempDriver: RegisterAddress::TEMP_DRIVER, f32, R);
    register!(TempMotor: RegisterAddress::TEMP_MOTOR, f32, R);
    register!(BusVolt: RegisterAddress::BUS_VOLT, f32, R);
    register!(BusCurr: RegisterAddress::BUS_CURR, f32, R);
    register!(BattVolt: RegisterAddress::BATT_VOLT, f32, R);
    register!(ErrorCode: RegisterAddress::ERROR_CODE, ErrorCodes, R);

    register!(PositionTarget: RegisterAddress::POS_TARGET, f32, RW);
    register!(PositionCurrent: RegisterAddress::POS_CURRENT, f32, R);
    register!(PositionLimitMax: RegisterAddress::POS_LIMIT_MAX, f32, RW);
    register!(PositionLimitMin: RegisterAddress::POS_LIMIT_MIN, f32, RW);
    register!(PositionLimitDiff: RegisterAddress::POS_LIMIT_DIFF, f32, RW);
    register!(PositionPIDP: RegisterAddress::POS_PIDP, f32, RW);
    register!(PositionSmoothCyc: RegisterAddress::POS_SMOOTH_CYC, u8, RW);
    // register!(PositionADRCParam: RegisterAddress::POS_ADRC_PARAM, f32, RW);
    register!(PositionCalZero: RegisterAddress::POS_CAL_ZERO, u8, W);

    register!(VelocityTarget: RegisterAddress::VEL_TARGET, f32, RW);
    register!(VelocityCurrent: RegisterAddress::VEL_CURRENT, f32, R);
    register!(VelocityLimitMax: RegisterAddress::VEL_LIMIT_MAX, f32, RW);
    register!(VelocityLimitMin: RegisterAddress::VEL_LIMIT_MIN, f32, RW);
    register!(VelocityLimitDiff: RegisterAddress::VEL_LIMIT_DIFF, f32, RW);
    register!(VelocityPIDP: RegisterAddress::VEL_PIDP, f32, RW);
    register!(VelocityPIDI: RegisterAddress::VEL_PIDI, f32, RW);
    register!(VelocitySmoothCyc: RegisterAddress::VEL_SMOOTH_CYC, u8, RW);
// register!(VelocityADRCParam: RegisterAddress::VEL_ADRC_PARAM, f32, RW);

    register!(TorqueTarget: RegisterAddress::TAU_TARGET, f32, RW);
    register!(TorqueCurrent: RegisterAddress::TAU_CURRENT, f32, R);
    register!(TorqueLimitMax: RegisterAddress::TAU_LIMIT_MAX, f32, RW);
    register!(TorqueLimitMin: RegisterAddress::TAU_LIMIT_MIN, f32, RW);
    register!(TorqueLimitDiff: RegisterAddress::TAU_LIMIT_DIFF, f32, RW);
    register!(TorquePIDP: RegisterAddress::TAU_PIDP, f32, RW);
    register!(TorquePIDI: RegisterAddress::TAU_PIDI, f32, RW);
    register!(TorqueSmoothCyc: RegisterAddress::TAU_SMOOTH_CYC, u8, RW);
// register!(TorqueADRCParam: RegisterAddress::TAU_ADRC_PARAM, f32, RW);
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