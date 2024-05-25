use std::time::Duration;

use serial2::SerialPort;

use umbratek::{Bus, registers, TransferError};
use umbratek::broadcast_registers::{BroadcastPosition, BroadcastPosTauCurrent};

fn main() -> Result<(), TransferError> {
    env_logger::Builder::from_default_env().init();

    let baud_rate = std::env::var("BAUD_RATE").map_or(4_000_000, |v| v.parse::<u32>().unwrap_or(4_000_000));
    let port = std::env::var("PORT").unwrap_or("/dev/ttyUSB0".to_string());

    let stream = SerialPort::open(&port, baud_rate).expect("failed to open port");
    if std::env::var("RS485").is_ok() {
        let rs4xx_config = serial2::rs4xx::Rs485Config::new();
        stream
            .set_rs4xx_mode(rs4xx_config)
            .expect("failed to set rs485 mode")
    }

    log::info!("opened port ({}, {}, rs485={})", port, baud_rate, std::env::var("RS485").is_ok());
    let mut bus = Bus::with_buffers(stream, vec![0; 1024], vec![0; 1024]);

    bus.write::<registers::MotionEnable>(3, true).unwrap();

    let broadcast_write = BroadcastPosition::new(3, 4, vec![20.0, 2.0]);
    let broadcast_read = BroadcastPosTauCurrent::new(3, 4);
    loop {
        let _ = bus.broadcast_write(&broadcast_write);

        let start = std::time::Instant::now();
        let res = bus.broadcast_read(&broadcast_read)?;

        res.into_iter().for_each(|r| {
            match r {
                Ok(res) => {
                    let (pos, tau) = res.data;
                    log::info!("{}: {:?}, {:?}",res.motor_id, pos, tau);
                }
                Err(e) => {
                    log::warn!("error {:?}", e);
                }
            }
        });
        log::trace!("time: {:?}", start.elapsed());

        std::thread::sleep(Duration::from_millis(1000));
    }
}
