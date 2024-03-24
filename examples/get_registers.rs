use umbratek_rs::{Bus, TransferError, ReadError};
use umbratek_rs::protocol::{registers};

const MOTOR_ID: u8 = 1;

fn main() -> Result<(), TransferError> {
    env_logger::Builder::from_default_env().init();
    let mut bus = Bus::open_with_buffers(
        "/dev/ttyUSB0",
        4000000,
        vec![0; 1024],
        vec![0; 1024],
    ).map_err(ReadError::from)?;

    // print every readable register
    println!("{:?}", bus.read::<registers::UUID>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::SWVersion>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::HWVersion>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::MultiVersion>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::MechRatio>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::ElecRatio>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::IWDGCyc>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::TempLimit>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::VoltLimit>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::CurrLimit>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::MotionMode>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::MotionEnable>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::BrakeEnable>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::TempDriver>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::TempMotor>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::BusVolt>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::BusCurr>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::BattVolt>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::ErrorCode>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::PositionTarget>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::PositionCurrent>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::PositionLimitMax>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::PositionLimitMin>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::PositionLimitDiff>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::PositionPIDP>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::PositionSmoothCyc>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::VelocityTarget>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::VelocityCurrent>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::VelocityLimitMax>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::VelocityLimitMin>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::VelocityLimitDiff>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::VelocityPIDP>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::VelocityPIDI>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::VelocitySmoothCyc>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::TorqueTarget>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::TorqueCurrent>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::TorqueLimitMax>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::TorqueLimitMin>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::TorqueLimitDiff>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::TorquePIDP>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::TorquePIDI>(MOTOR_ID)?.data);
    println!("{:?}", bus.read::<registers::TorqueSmoothCyc>(MOTOR_ID)?.data);






    Ok(())
}

