use zerocopy::AsBytes;
use crate::{Bus, ReadError, Response, TransferError};
use crate::bus::ResponsePacket;
use crate::protocol::{Readable, Register};
impl<ReadBuffer, WriteBuffer> Bus<ReadBuffer, WriteBuffer>
where
	ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
	WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
{
	/// Read an arbitrary number of bytes from a specific motor.
	pub fn read_raw(&mut self, motor_id: u8, address: u8, count: u8) -> Result<ResponsePacket<ReadBuffer, WriteBuffer>, TransferError> {
		self.write_instruction(motor_id, address, 0, false, |buffer| {
			buffer[0] = count;
		})?;
		let response = self.read_status_response(count as usize)?;
		crate::error::InvalidPacketId::check(response.sender_id(), motor_id).map_err(crate::ReadError::from)?;

		crate::error::InvalidParameterCount::check(response.data().len(), count.into()).map_err(crate::ReadError::from)?;
		Ok(response)
	}

	pub fn read<REG: Register + Readable>(&mut self, motor_id: u8) -> Result<Response<REG>, TransferError> {
		self.read_raw(motor_id, REG::address().as_bytes()[0], REG::size()).and_then(|response|
			Ok(Response {
				data: REG::from_bytes(response.data()).map_err(ReadError::from)?,
				motor_id: response.sender_id(),
				error: response.error(),
			}))
	}
}
