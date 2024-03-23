use umbratek_rs::protocol::PositionCurrent;

mod common;

#[test]
fn test_read() {
    let mut bus = common::setup();
    let pos = bus.read::<PositionCurrent>(1);
    debug_assert!(pos.is_ok());
}