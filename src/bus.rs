use serial2::SerialPort;
use std::io::Read;
use std::path::Path;
use std::time::{Duration, Instant};

use crate::checksum::calculate_checksum;
use crate::{ReadError, TransferError, WriteError};

// UMBRATEK PACKET REQUEST
// |   1   |    1     |    1      |    1         | N    | 2   |
// |SENDER | RECEIVER | 0   | LEN | RW | COMMAND | DATA | CRC |

// UMBRATEK PACKET RESPONSE
// |   1   |    1     |    1      |    1         | N    | 2   |
// |SENDER | RECEIVER | ERR | LEN | RW | COMMAND | DATA | CRC |
const HEADER_SIZE: usize = 4;

/// Dynamixel Protocol 2 communication bus.
pub struct Bus<ReadBuffer, WriteBuffer> {
	/// The underlying stream (normally a serial port).
	serial_port: SerialPort,

	/// The timeout for reading a single response.
	read_timeout: Duration,

	/// The buffer for reading incoming messages.
	read_buffer: ReadBuffer,

	/// The total number of valid bytes in the read buffer.
	read_len: usize,

	/// The buffer for outgoing messages.
	write_buffer: WriteBuffer,
}

impl Bus<Vec<u8>, Vec<u8>> {
	/// Open a serial port with the given baud rate.
	///
	/// This will allocate a new read and write buffer of 128 bytes each.
	/// Use [`Self::open_with_buffers()`] if you want to use a custom buffers.
	pub fn open(path: impl AsRef<Path>, baud_rate: u32, read_timeout: Duration) -> std::io::Result<Self> {
		let port = SerialPort::open(path, baud_rate)?;
		Ok(Self::new(port, read_timeout))
	}

	/// Create a new bus for an open serial port.
	///
	/// This will allocate a new read and write buffer of 128 bytes each.
	/// Use [`Self::with_buffers()`] if you want to use a custom buffers.
	pub fn new(stream: SerialPort, read_timeout: Duration) -> Self {
		Self::with_buffers(stream, read_timeout, vec![0; 128], vec![0; 128])
	}
}

impl<ReadBuffer, WriteBuffer> Bus<ReadBuffer, WriteBuffer>
where
	ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
	WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
{
	/// Open a serial port with the given baud rate.
	///
	/// This will allocate a new read and write buffer of 128 bytes each.
	pub fn open_with_buffers(
		path: impl AsRef<Path>,
		baud_rate: u32,
		read_timeout: Duration,
		read_buffer: ReadBuffer,
		write_buffer: WriteBuffer,
	) -> std::io::Result<Self> {
		let port = SerialPort::open(path, baud_rate)?;
		Ok(Self::with_buffers(port, read_timeout, read_buffer, write_buffer))
	}

	/// Create a new bus using pre-allocated buffers.
	pub fn with_buffers(serial_port: SerialPort, read_timeout: Duration, read_buffer: ReadBuffer, mut write_buffer: WriteBuffer) -> Self {
		// Pre-fill write buffer with the header prefix.
		assert!(write_buffer.as_mut().len() >= HEADER_SIZE + 2);
		//write_buffer.as_mut()[..4].copy_from_slice(&HEADER_PREFIX);

		Self {
			serial_port,
			read_timeout,
			read_buffer,
			read_len: 0,
			write_buffer,
		}
	}

	/// Write a raw instruction to a stream, and read a single raw response.
	///
	/// This function also checks that the packet ID of the status response matches the one from the instruction.
	///
	/// This is not suitable for broadcast instructions.
	/// For broadcast instructions, each motor sends an individual response or no response is send at all.
	/// Instead, use [`Self::write_instruction`] and [`Self::read_status_response`].
	pub fn transfer_single<F>(
		&mut self,
		sender_id: u8,
		command: u8,
		parameter_count: usize,
		encode_parameters: F,
	) -> Result<ResponsePacket<ReadBuffer, WriteBuffer>, TransferError>
	where
		F: FnOnce(&mut [u8]),
	{
		self.write_instruction(sender_id, command, parameter_count, false, encode_parameters)?;
		// writes return 6 bytes
		// reads return 6 + data_length
		let response = self.read_status_response(parameter_count)?;
		crate::error::InvalidPacketId::check(response.sender_id(), sender_id).map_err(crate::ReadError::from)?;
		Ok(response)
	}

	/// Write an instruction message to the bus.
	pub fn write_instruction<F>(
		&mut self,
		motor_id: u8,
		command: u8,
		parameter_count: usize,
		write: bool,
		encode_parameters: F,
	) -> Result<(), WriteError>
	where
		F: FnOnce(&mut [u8]),
	{
		// Throw away old data in the read buffer.
		// Ideally, we would also flush the kernel buffer, but the serial crate doesn't expose that.
		// We don't do this when reading a reply, because we might have multiple replies for one instruction,
		// and read() can potentially read more than one reply per syscall.
		self.read_len = 0;

		let buffer = self.write_buffer.as_mut();
		if buffer.len() < HEADER_SIZE + parameter_count + 2 {
			// TODO: return proper error.
			panic!("write buffer not large enough for outgoing mesage");
		}

		let stuffed_message = Self::format_write(buffer, motor_id, command, parameter_count, write, encode_parameters);
		log::trace!("sending instruction: {:02X?}", stuffed_message);
		self.serial_port.discard_input_buffer().map_err(WriteError::DiscardBuffer)?;
		self.serial_port.write_all(stuffed_message).map_err(WriteError::Write)?;
		Ok(())
	}

	fn format_write<F>(buffer: &mut [u8], motor_id: u8, command: u8, parameter_count: usize, write: bool, encode_parameters: F,
	) -> &[u8] where
		F: FnOnce(&mut [u8]), {
		let state = 0;

		// Add the header, with a placeholder for the length field.
		buffer[0] = 0xAA;
		buffer[1] = motor_id;
		buffer[2] = (state & 0x01 << 7) | (parameter_count as u8 + 1) & 0x7F;
		buffer[3] = ((write as u8) << 7) | (command & 0x7F);
		if parameter_count > 0 {
			encode_parameters(&mut buffer[HEADER_SIZE..][..parameter_count as usize]);
		}

		// Add checksum.
		let checksum_index = HEADER_SIZE + parameter_count;
		let checksum = calculate_checksum(&buffer[..checksum_index]);

		buffer[checksum_index..][..2].copy_from_slice(&checksum);
		// write_u16_le(&mut buffer[checksum_index..], checksum);

		// Send message.
		&buffer[..checksum_index + 2]
	}
	/// Read a raw status response from the bus.
	pub fn read_status_response(&mut self, parameter_count: usize) -> Result<ResponsePacket<ReadBuffer, WriteBuffer>, ReadError> {
		let deadline = Instant::now() + self.read_timeout;
		//let stuffed_message_len = loop {
		//    self.remove_garbage();

		//    // The call to remove_garbage() removes all leading bytes that don't match a status header.
		//    // So if there's enough bytes left, it's a status header.
		//    if self.read_len > STATUS_HEADER_SIZE {
		//        let read_buffer = &self.read_buffer.as_mut()[..self.read_len];
		//        let body_len = read_buffer[5] as usize + read_buffer[6] as usize * 256;
		//        let body_len = body_len - 2; // Length includes instruction and error fields, which is already included in STATUS_HEADER_SIZE too.

		//        if self.read_len >= STATUS_HEADER_SIZE + body_len {
		//            break STATUS_HEADER_SIZE + body_len;
		//        }
		//    }

		//    if Instant::now() > deadline {
		//        trace!(
		//            "timeout reading status response, data in buffer: {:02X?}",
		//            &self.read_buffer.as_ref()[..self.read_len]
		//        );
		//        return Err(std::io::ErrorKind::TimedOut.into());
		//    }

		//    // Try to read more data into the buffer.
		//    let new_data = self
		//        .serial_port
		//        .read(&mut self.read_buffer.as_mut()[self.read_len..])?;
		//    if new_data == 0 {
		//        continue;
		//    }

		//    self.read_len += new_data;
		//};
		let message_length = parameter_count + HEADER_SIZE + 2; // + checksum

		let buffer = self.read_buffer.as_mut();
		log::trace!("Reading {} bytes", message_length);
		self.serial_port
			.read_exact(&mut buffer[self.read_len..self.read_len + message_length])?;
		let parameters_end = parameter_count + HEADER_SIZE;
		log::trace!("read packet: {:02X?}", &buffer[..message_length]);

		let checksum_message:[u8; 2] = buffer[parameters_end..][..2].try_into().map_err(|_| ReadError::Io(std::io::Error::new(std::io::ErrorKind::InvalidData, "Failed to convert checksum")))?;
		let checksum_computed = calculate_checksum(&buffer[..parameters_end]);
		if checksum_message != checksum_computed {
			self.consume_read_bytes(message_length);
			return Err(crate::InvalidChecksum {
				message: checksum_message,
				computed: checksum_computed,
			}
			.into());
		}

		// Remove byte-stuffing from the data.
		//let parameter_count =
		//  bytestuff::unstuff_inplace(&mut buffer[STATUS_HEADER_SIZE..parameters_end]);

		// Creating the status packet struct here means that the data gets purged from the buffer even if we return early using the try operator.
		self.read_len += message_length;
		let response = ResponsePacket { bus: self, message_length, parameter_count};
		debug_assert_eq!(response.data_length(), parameter_count as u8);

		// crate::InvalidInstruction::check(response.instruction_id(), crate::instructions::instruction_id::STATUS)?;
		// crate::MotorError::check(response.error())?;
		Ok(response)
	}
	pub fn scan_motors(&mut self) -> Result<(), crate::MotorError>{
		todo!();
	}

	// /// Remove leading garbage data from the read buffer.
	// fn remove_garbage(&mut self) {
	// 	let read_buffer = self.read_buffer.as_mut();
	// 	let garbage_len = find_header(&read_buffer[..self.read_len]);
	// 	if garbage_len > 0 {
	// 		debug!("skipping {} bytes of leading garbage.", garbage_len);
	// 		trace!("skipped garbage: {:02X?}", &read_buffer[..garbage_len]);
	// 	}
	// 	self.consume_read_bytes(garbage_len);
	// }

	fn consume_read_bytes(&mut self, len: usize) {
		debug_assert!(len <= self.read_len);
		self.read_buffer.as_mut().copy_within(len..self.read_len, 0);
		self.read_len -= len;
	}
}

/// A status response that is currently in the read buffer of a bus.
///
/// When dropped, the response data is removed from the read buffer.
pub struct ResponsePacket<'a, ReadBuffer, WriteBuffer>
where
	ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
	WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
{
	/// The bus that read the message.
	bus: &'a mut Bus<ReadBuffer, WriteBuffer>,

	/// The total length of the message.
	message_length: usize,

	parameter_count: usize,
}

impl<'a, ReadBuffer, WriteBuffer> ResponsePacket<'a, ReadBuffer, WriteBuffer>
where
	ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
	WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
{
	/// Get the raw bytes of the message.
	///
	/// This includes the message header and the data.
	/// It does not include the CRC or byte-stuffing.
	pub fn as_bytes(&self) -> &[u8] {
		&self.bus.read_buffer.as_ref()[..HEADER_SIZE + self.parameter_count]
	}

	/// The sender ID of the response.
	pub fn sender_id(&self) -> u8 {
		self.as_bytes()[0]
	}

	/// The receiver ID of the response.
	pub fn receiver_id(&self) -> u8 {
		self.as_bytes()[1]
	}

	/// The state bit of the response.
	pub fn error(&self) -> bool {
		(self.as_bytes()[2] & 80 >> 7) == 1
	}

	// The number of bytes of data in the response + the instruction byte
	// This doesn't include the HEADER and CRC bytes
	pub fn message_length(&self) -> u8 {
		self.as_bytes()[2] & 0x7F
	}

	/// The write bit of the response.
	pub fn write(&self) -> bool {
		(self.as_bytes()[3] & 80 >> 7) == 1
	}

	pub fn command(&self) -> u8 {
		self.as_bytes()[3] & 0x7F
	}

	pub fn data_length(&self) -> u8 {
		self.message_length() - 1
	}

	/// The data of the response.
	pub fn data(&self) -> &[u8] {
		&self.as_bytes()[HEADER_SIZE..][..self.parameter_count]
	}
}

impl<'a, ReadBuffer, WriteBuffer> Drop for ResponsePacket<'a, ReadBuffer, WriteBuffer>
where
	ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
	WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
{
	fn drop(&mut self) {
		self.bus.consume_read_bytes(self.message_length);
	}
}

/// A response from a motor.
#[derive(Debug)]
pub struct Response<T> {
	/// The motor that sent the response.
	pub motor_id: u8,

	/// The alert bit from the response message.
	///
	/// If this is set, you can normally check the "Hardware Error" register for more details.
	/// Consult your motor manual for more details.
	pub error: bool,

	/// The data from the motor.
	pub data: T,
}


// impl<'a, ReadBuffer, WriteBuffer, Reg> TryFrom<ResponsePacket<'a, ReadBuffer, WriteBuffer>> for Response<Reg>
// where
// 	ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
// 	WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
// 	Reg: Register
// {
// 	type Error = std::io::Error;
//
// 	fn try_from(status_packet: ResponsePacket<'a, ReadBuffer, WriteBuffer>) -> Result<Self, Self::Error> {
// 		Ok(Self {
// 			data: Reg::from_bytes(status_packet.data())?,
// 			motor_id: status_packet.sender_id(),
// 			error: status_packet.error(),
// 		})
// 	}
// }
//
impl<'a, ReadBuffer, WriteBuffer> From<ResponsePacket<'a, ReadBuffer, WriteBuffer>> for Response<()>
where
	ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
	WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
{
	fn from(status_packet: ResponsePacket<'a, ReadBuffer, WriteBuffer>) -> Self {
		Self {
			data: (),
			motor_id: status_packet.sender_id(),
			error: status_packet.error(),
		}
	}
}
//
// impl<'a, ReadBuffer, WriteBuffer> From<ResponsePacket<'a, ReadBuffer, WriteBuffer>> for Response<Vec<u8>>
// where
// 	ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
// 	WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
// {
// 	fn from(status_packet: ResponsePacket<'a, ReadBuffer, WriteBuffer>) -> Self {
// 		Self {
// 			data: status_packet.data().to_owned(),
// 			motor_id: status_packet.sender_id(),
// 			error: status_packet.error(),
// 		}
// 	}
// }
//
// impl<'a, ReadBuffer, WriteBuffer> From<ResponsePacket<'a, ReadBuffer, WriteBuffer>> for Response<u8>
// where
// 	ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
// 	WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
// {
// 	fn from(status_packet: ResponsePacket<'a, ReadBuffer, WriteBuffer>) -> Self {
// 		Self {
// 			data: read_u8_le(status_packet.data()),
// 			motor_id: status_packet.sender_id(),
// 			error: status_packet.error(),
// 		}
// 	}
// }
//
// impl<'a, ReadBuffer, WriteBuffer> From<ResponsePacket<'a, ReadBuffer, WriteBuffer>> for Response<u16>
// where
// 	ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
// 	WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
// {
// 	fn from(status_packet: ResponsePacket<'a, ReadBuffer, WriteBuffer>) -> Self {
// 		Self {
// 			data: read_u16_le(status_packet.data()),
// 			motor_id: status_packet.sender_id(),
// 			error: status_packet.error(),
// 		}
// 	}
// }
//
// impl<'a, ReadBuffer, WriteBuffer> From<ResponsePacket<'a, ReadBuffer, WriteBuffer>> for Response<u32>
// where
// 	ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
// 	WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
// {
// 	fn from(status_packet: ResponsePacket<'a, ReadBuffer, WriteBuffer>) -> Self {
// 		Self {
// 			data: read_u32_le(status_packet.data()),
// 			motor_id: status_packet.sender_id(),
// 			error: status_packet.error(),
// 		}
// 	}
// }


#[cfg(test)]
mod test {
	use zerocopy::AsBytes;
	use super::*;
	use crate::RegisterAddress;
	#[test]
	fn test_format_write() {
		let mut buff = vec![0; 128];
		let msg = Bus::<Vec<u8>, Vec<u8>>::format_write(buff.as_mut(), 0x01, RegisterAddress::POS_CURRENT.as_bytes()[0], 0, false, |_buffer| {});

		assert_eq!(msg, [0xAA, 0x01, 0x01, 0x31, 0xB0, 0x78]);


	}
	// #[test]
// 	fn test_find_garbage_end() {
// 		assert!(find_header(&[0xFF]) == 0);
// 		assert!(find_header(&[0xFF, 0xFF]) == 0);
// 		assert!(find_header(&[0xFF, 0xFF, 0xFD]) == 0);
// 		assert!(find_header(&[0xFF, 0xFF, 0xFD, 0x00]) == 0);
// 		assert!(find_header(&[0xFF, 0xFF, 0xFD, 0x00, 9]) == 0);

// 		assert!(find_header(&[0, 1, 2, 3, 4, 0xFF]) == 5);
// 		assert!(find_header(&[0, 1, 2, 3, 4, 0xFF, 0xFF]) == 5);
// 		assert!(find_header(&[0, 1, 2, 3, 4, 0xFF, 0xFF, 0xFD]) == 5);
// 		assert!(find_header(&[0, 1, 2, 3, 4, 0xFF, 0xFF, 0xFD, 0x00]) == 5);
// 		assert!(find_header(&[0, 1, 2, 3, 4, 0xFF, 0xFF, 0xFD, 0x00, 9]) == 5);

// 		assert!(find_header(&[0xFF, 1]) == 2);
// 		assert!(find_header(&[0, 1, 2, 3, 4, 0xFF, 6]) == 7);
// 	}
}
