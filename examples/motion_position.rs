use std::thread::sleep;
use umbratek_rs::protocol::registers;
use umbratek_rs::{Bus, ReadError, TransferError};

const MOTOR_ID: u8 = 1;

fn main() -> Result<(), TransferError> {
	env_logger::Builder::from_default_env().init();
	let mut bus = Bus::open_with_buffers("/dev/ttyUSB0", 4000000, vec![0; 1024], vec![0; 1024]).map_err(ReadError::from)?;

	let _ = bus
		.write(MOTOR_ID, registers::MotionEnable::new(true))
		.expect("failed to enable pos mode");
	let min_pos = dbg!(bus.read::<registers::PositionLimitMin>(MOTOR_ID)?.data);
	let max_pos = dbg!(bus.read::<registers::PositionLimitMax>(MOTOR_ID)?.data);
	sleep(std::time::Duration::from_millis(1000));
	loop {
		let curr_pos = bus.read::<registers::PositionCurrent>(1)?.data;
		let error = bus.read::<registers::ErrorCode>(MOTOR_ID)?.data;
		println!("Motor {}, Position: {}, Error: {:?}", MOTOR_ID, curr_pos.value(), error.value());

		let _ = bus.write(1, registers::PositionTarget::new(min_pos.value()))?;
		sleep(std::time::Duration::from_secs(1));

		let curr_pos = bus.read::<registers::PositionCurrent>(1)?.data;
		let error = bus.read::<registers::ErrorCode>(MOTOR_ID)?.data;
		println!("Motor {}, Position: {}, Error: {:?}", MOTOR_ID, curr_pos.value(), error.value());

		let _ = bus.write(1, registers::PositionTarget::new(max_pos.value()))?;
		sleep(std::time::Duration::from_secs(1));
	}
}
