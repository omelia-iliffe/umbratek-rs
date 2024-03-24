use crate::protocol::{Register, Writable};
use crate::{Bus, Response, TransferError};
impl<ReadBuffer, WriteBuffer> Bus<ReadBuffer, WriteBuffer>
where
	ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
	WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
{
	/// Write an arbitrary number of bytes to a specific motor.
	pub fn write_raw(&mut self, motor_id: u8, command: u8, data: &[u8]) -> Result<Response<()>, TransferError> {
		self.write_instruction(motor_id, command, data.len(), true, |buffer| buffer.copy_from_slice(data))?;
		// writes return 6 bytes
		// reads return 6 + data_length
		let response = self.read_status_response(0)?;
		crate::error::InvalidPacketId::check(response.sender_id(), motor_id).map_err(crate::ReadError::from)?;

		crate::error::InvalidParameterCount::check(response.data().len(), 0).map_err(crate::ReadError::from)?;
		Ok(response.into())
	}

	pub fn write<Reg: Register + Writable>(&mut self, motor_id: u8, register: Reg) -> Result<Response<()>, TransferError> {
		self.write_raw(motor_id, Reg::address(), &register.as_bytes())
	}
}
