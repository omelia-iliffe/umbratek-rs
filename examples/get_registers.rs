use umbratek::{Bus, ReadError, TransferError};
use umbratek::protocol::registers;

fn main() -> Result<(), TransferError> {
    env_logger::Builder::from_default_env().init();
    let motor_id = std::env::var("MOTOR_ID").map_or(1, |v| v.parse::<u8>().unwrap_or(1));

    let baud_rate = std::env::var("BAUD_RATE").map_or(921600, |v| v.parse::<u32>().unwrap_or(921600));
    let mut bus = Bus::open_with_buffers("/dev/ttyUSB0", baud_rate, vec![0; 1024], vec![0; 1024]).map_err(ReadError::from)?;


    // print every readable register
    println!("{:?}", bus.read::<registers::UUID>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::SWVersion>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::HWVersion>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::MultiVersion>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::MechRatio>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::ElecRatio>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::IWDGCyc>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::TempLimit>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::VoltLimit>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::CurrLimit>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::MotionMode>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::MotionEnable>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::BrakeEnable>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::TempDriver>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::TempMotor>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::BusVolt>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::BusCurr>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::BattVolt>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::ErrorCode>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::PositionTarget>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::PositionCurrent>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::PositionLimitMax>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::PositionLimitMin>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::PositionLimitDiff>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::PositionPIDP>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::PositionSmoothCyc>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::VelocityTarget>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::VelocityCurrent>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::VelocityLimitMax>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::VelocityLimitMin>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::VelocityLimitDiff>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::VelocityPIDP>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::VelocityPIDI>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::VelocitySmoothCyc>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::TorqueTarget>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::TorqueCurrent>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::TorqueLimitMax>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::TorqueLimitMin>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::TorqueLimitDiff>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::TorquePIDP>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::TorquePIDI>(motor_id)?.data);
    println!("{:?}", bus.read::<registers::TorqueSmoothCyc>(motor_id)?.data);

    Ok(())
}
