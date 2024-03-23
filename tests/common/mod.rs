use std::path::PathBuf;
use umbratek_rs::Bus;
const MOTOR_ID: u8 = 40;

pub(crate) fn setup() -> Bus<Vec<u8>, Vec<u8>> {
    let serial_port: PathBuf = "/dev/ttyUSB0".into();
    Bus::open_with_buffers(
        &serial_port,
        4000000,
        std::time::Duration::from_millis(20),
        vec![0; 1024],
        vec![0; 1024],
    ).expect("Failed to open bus")
}