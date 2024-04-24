use std::time::Duration;

use umbratek::{Bus, ReadError, registers, TransferError};
use umbratek::broadcast_registers::BroadcastPosition;

fn main() -> Result<(), TransferError> {
    env_logger::Builder::from_default_env().init();

    let baud_rate = std::env::var("BAUD_RATE").map_or(4_000_000, |v| v.parse::<u32>().unwrap_or(4_000_000));
    let mut bus = Bus::open_with_buffers("/dev/ttyUSB0", baud_rate, vec![0; 1024], vec![0; 1024]).map_err(ReadError::from)?;

    bus.write::<registers::MotionEnable>(1, true).unwrap();

    let broadcast_write = BroadcastPosition::new(1, 2, vec![20.0, 2.0]);
    loop {
        let _ = bus.broadcast_write(&broadcast_write);
        std::thread::sleep(Duration::from_millis(1000));
    }
}
