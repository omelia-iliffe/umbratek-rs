use crate::{Bus, Writable, WriteError};
use crate::broadcast_registers::BroadcastRegister;

impl<ReadBuffer, WriteBuffer> Bus<ReadBuffer, WriteBuffer>
    where
        ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
        WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
{
    pub fn broadcast_write_raw<Iter>(&mut self, command: u8, count: u8, start_id: u8, end_id: u8, data: Iter) -> Result<(), WriteError>
        where
            Iter: IntoIterator<Item=Vec<u8>>,
            Iter::IntoIter: std::iter::ExactSizeIterator,
    // Buf: AsRef<[u8]> + 'a,
    {
        let data = data.into_iter();
        debug_assert_eq!(data.len(), (end_id - start_id + 1) as usize);


        let stride = usize::from(count);
        let parameter_count = 2 + data.len() * stride;
        self.write_instruction(0x55, command, parameter_count, true, |buffer| {
            buffer[0] = start_id;
            buffer[1] = end_id;
            for (i, data) in data.into_iter().enumerate() {
                assert_eq!(data.len(), count as usize);
                let buffer = &mut &mut buffer[2 + i * stride..][..stride];
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