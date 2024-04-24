use crate::{Bus, Writable, WriteError};
use crate::broadcast_registers::BroadcastRegister;
use crate::commands::BroadcastWriteData;

impl<ReadBuffer, WriteBuffer> Bus<ReadBuffer, WriteBuffer>
    where
        ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
        WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
{
    pub fn broadcast_write_raw<Iter>(&mut self, command: u8, count: u8, data: Iter) -> Result<(), WriteError>
        where
            Iter: IntoIterator<Item=BroadcastWriteData<Vec<u8>>>,
            Iter::IntoIter: std::iter::ExactSizeIterator,
    // Buf: AsRef<[u8]> + 'a,
    {
        let data = data.into_iter();
        let motors = data.len();
        let (ids, data): (Vec<_>, Vec<_>) = data.map(|d| {
            (d.id, d.data)
        }).unzip();
        let (first, last) = ids.first().zip(ids.last()).unwrap();

        let stride = usize::from(count);
        let parameter_count = 2 + motors * stride;
        self.write_instruction(0x55, command, parameter_count, true, |buffer| {
            buffer[0] = *first;
            buffer[1] = *last;
            for (i, data) in data.into_iter().enumerate() {
                assert_eq!(data.len(), count as usize);
                let buffer = &mut dbg!(&mut buffer[2 + i * stride..])[..stride];
                buffer.copy_from_slice(&data);
            }
        })
    }

    pub fn broadcast_write<RegRef, REG>(&mut self, register: RegRef) -> Result<(), WriteError> where RegRef: AsRef<REG>, REG: BroadcastRegister + Writable {
        let data = register.as_ref().as_bytes();
        self.write_instruction(0x55, REG::ADDRESS, data.len(), true, |buffer| {
            buffer.copy_from_slice(&data);
        })
    }
}