use byteorder::{ReadBytesExt, BE};
use num_derive::FromPrimitive;
use zerocopy::AsBytes;


pub enum MotionMode {
    Position = 1,
    Velocity = 2,
    Torque = 3,
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
    fn from_bytes(bytes: &[u8]) -> Result<Self, std::io::Error> where Self: Sized;

}

#[derive(Debug, Clone, PartialEq)]
pub struct PositionCurrent(f32);

impl PositionCurrent {
    pub fn value(&self) -> f32 {
        self.0
    }

}

impl Register for PositionCurrent {
    fn address() -> RegisterAddress {
        RegisterAddress::POS_CURRENT
    }
    fn size() -> u8 {
        4
    }
}
impl Readable for PositionCurrent {
    fn from_bytes(bytes: &[u8]) -> Result<Self, std::io::Error> {
        let mut rdr = std::io::Cursor::new(bytes);
        let value = rdr.read_f32::<BE>()?;
        Ok(PositionCurrent(value))
    }
}

impl Writable for PositionCurrent {
    fn as_bytes(&self) -> Vec<u8> {
        self.0.to_be_bytes().to_vec()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PositionTarget(f32);

impl PositionTarget {
    pub fn new(value: f32) -> Self {
        PositionTarget(value)
    }
    pub fn value(&self) -> f32 {
        self.0
    }
}

impl Register for PositionTarget {
    fn address() -> RegisterAddress {
        RegisterAddress::POS_TARGET
    }
    fn size() -> u8 {
        4
    }
}

impl Readable for PositionTarget {
    fn from_bytes(bytes: &[u8]) -> Result<Self, std::io::Error> {
        let mut rdr = std::io::Cursor::new(bytes);
        let value = rdr.read_f32::<BE>()?;
        Ok(PositionTarget(value))
    }
}

impl Writable for PositionTarget {
    fn as_bytes(&self) -> Vec<u8> {
        self.0.to_be_bytes().to_vec()
    }
}

#[allow(non_camel_case_types)] // TODO: Fix cases
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