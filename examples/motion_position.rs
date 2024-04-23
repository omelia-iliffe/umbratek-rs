use std::thread::sleep;

use umbratek::{Bus, ReadError, TransferError};
use umbratek::protocol::registers;

const MOTOR_ID: u8 = 1;

fn main() -> Result<(), TransferError> {
    env_logger::Builder::from_default_env().init();
    let mut bus = Bus::open_with_buffers("/dev/ttyUSB0", 4000000, vec![0; 1024], vec![0; 1024]).map_err(ReadError::from)?;

    let _ = bus
        .write::<registers::MotionEnable>(MOTOR_ID, true)
        .expect("failed to enable pos mode");
    let min_pos = dbg!(bus.read::<registers::PositionLimitMin>(MOTOR_ID)?);
    let max_pos = dbg!(bus.read::<registers::PositionLimitMax>(MOTOR_ID)?);
    sleep(std::time::Duration::from_millis(1000));
    loop {
        let curr_pos = bus.read::<registers::PositionCurrent>(1)?;
        let error = bus.read::<registers::ErrorCode>(MOTOR_ID)?;
        println!("Motor {}, Position: {}, Error: {:?}", MOTOR_ID, curr_pos, error);

        let _ = bus.write::<registers::PositionTarget>(1, min_pos)?;
        sleep(std::time::Duration::from_secs(1));

        let curr_pos = bus.read::<registers::PositionCurrent>(1)?;
        let error = bus.read::<registers::ErrorCode>(MOTOR_ID)?;
        println!("Motor {}, Position: {}, Error: {:?}", MOTOR_ID, curr_pos, error);

        let _ = bus.write::<registers::PositionTarget>(1, max_pos)?;
        sleep(std::time::Duration::from_secs(1));
    }
}
