use crate::{Bus, ReadError, Response, TransferError};
use crate::broadcast_registers::{BroadcastReadable, BroadcastRegister};

type ResponseResult<REG> = Result<Response<<REG as BroadcastRegister>::Inner>, ReadError>;

impl<ReadBuffer, WriteBuffer> Bus<ReadBuffer, WriteBuffer>
    where
        ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
        WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
{
    pub fn broadcast_read_cb<F>(&mut self, command: u8, count: u8, start_id: u8, end_id: u8, mut on_response: F) -> Result<(), TransferError>
        where
            F: FnMut(Result<Response<&[u8]>, ReadError>),
    {
        self.write_instruction(0x55, command, 2, false, |buffer| {
            buffer[0] = start_id;
            buffer[1] = end_id;
        })?;
        for motor_id in start_id..=end_id {
            let response = self.read_status_response(usize::from(count)).and_then(|response| {
                crate::error::InvalidPacketId::check(response.sender_id(), motor_id)?;
                crate::error::InvalidParameterCount::check(response.data().len(), count.into())?;
                Ok(response)
            });

            match response {
                Ok(response) => on_response(Ok((&response).into())),
                Err(e) => on_response(Err(e)),
            }
        }
        Ok(())
    }

    pub fn broadcast_read<REG>(&mut self, register: impl AsRef<REG>) -> Result<Vec<ResponseResult<REG>>, TransferError> where REG: BroadcastRegister + BroadcastReadable {
        let reg = register.as_ref();
        let mut result = Vec::with_capacity(reg.num_ids() * REG::SIZE as usize);
        let (first, last) = reg.ids();

        self.broadcast_read_cb(REG::ADDRESS, REG::SIZE + 1, first, last, |response| {
            let r = match response {
                Err(e) => Err(e),
                Ok(response) => match REG::try_inner_from_bytes(&response.data[1..]) {
                    Ok(data) => Ok(Response {
                        motor_id: response.motor_id,
                        data,
                    }),
                    Err(e) => Err(e.into()),
                },
            };
            result.push(r);
        })?;
        Ok(result)
    }
}
