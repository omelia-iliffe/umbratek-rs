pub mod read;
pub mod write;
mod broadcast_write;

#[derive(Debug, Clone, Eq, PartialEq)]
pub struct BroadcastWriteData<T> {
    pub id: u8,
    pub data: T,
}

