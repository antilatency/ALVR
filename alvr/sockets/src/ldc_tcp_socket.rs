//! Stream-aware TCP socket with packet interface. The stream ID is used to select the correct
//! buffer pool for the receive end, to reduce unnecessarily large allocations.
//! The API is compatible with both blocking and non-blocking TcpStream instances

use alvr_common::{prelude::*, RelaxedAtomic};
use std::{
    collections::{HashMap, VecDeque},
    io::{ErrorKind, Read, Write},
    net::TcpStream,
    sync::Arc,
};

// Writes all buffer bytes into the socket. In case the socket returns early, retry, in which case
// the socket could be temporarily locked by the read thread.
// Return Ok(true) if success, Ok(false) if running, in which case the socket SHOULD be closed
// because the packet delimiters are out of sync.
fn interruptible_write_all(
    mut socket: &TcpStream,
    mut buffer: &[u8],
    running: &RelaxedAtomic,
) -> IntResult {
    loop {
        let res = socket.write(buffer);

        check_interrupt!(running.value());

        match res {
            Ok(size) => {
                if size == buffer.len() {
                    return Ok(());
                } else {
                    buffer = &buffer[..size];
                }
            }
            Err(e) => {
                if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::Interrupted {
                    continue;
                } else {
                    return int_fmt_e!("{e}");
                }
            }
        }
    }
}

fn interruptible_read_all(
    mut socket: &TcpStream,
    mut buffer: &mut [u8],
    running: &RelaxedAtomic,
) -> IntResult {
    loop {
        let res = socket.read(buffer);

        check_interrupt!(running.value());

        match res {
            Ok(size) => {
                if size == buffer.len() {
                    return Ok(());
                } else {
                    buffer = &mut buffer[..size];
                }
            }
            Err(e) => {
                if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::Interrupted {
                    continue;
                } else {
                    return int_fmt_e!("{e}");
                }
            }
        }
    }
}

// Stream-aware length-delimited-coded TCP send wrapper
pub struct LdcTcpSender {
    running: Arc<RelaxedAtomic>,
    socket: TcpStream,

    // The stream cursor is in a valid position. Becomes false if the send operation is interrupted.
    valid: bool,
}

impl LdcTcpSender {
    // Valid usage: the TcpStream sending end must be used exclusively by this class and by only one
    // instance
    pub fn new(socket: TcpStream, running: Arc<RelaxedAtomic>) -> Self {
        Self {
            running,
            socket,
            valid: true,
        }
    }

    // Note: send() takes mut self because it cannot have concurrent send actions
    pub fn send(&mut self, stream_id: u8, buffer: &[u8]) -> IntResult {
        if !self.valid {
            return interrupt();
        }

        let mut prefix = [0; 9];
        prefix[0] = stream_id;
        prefix.copy_from_slice(&(buffer.len() as u64).to_le_bytes());

        // let res = interruptible_write_all(&self.socket, &prefix, &self.running);

        if let Err(e) = interruptible_write_all(&self.socket, &prefix, &self.running)
            .and_then(|()| interruptible_write_all(&self.socket, buffer, &self.running))
        {
            self.valid = false;
            return Err(e);
        }

        Ok(())
    }
}

// Stream-aware length-delimited-coded TCP receive wrapper
// This is optimized with the assumption that packets from the same stream ID are similar in size.
pub struct LdcTcpReceiver {
    running: Arc<RelaxedAtomic>,
    socket: TcpStream,
    buffers: HashMap<u8, VecDeque<Vec<u8>>>,
    valid: bool,
}

impl LdcTcpReceiver {
    // Valid usage: the TcpStream receiving end must be used exclusively by this class and by only
    // one instance
    pub fn new(socket: TcpStream, running: Arc<RelaxedAtomic>) -> Self {
        Self {
            running,
            socket,
            buffers: HashMap::new(),
            valid: true,
        }
    }

    // Return a buffer for a specific stream ID.
    // Why not providing the buffer directly in rcev()? At the time of receive we don't know what
    // type of packet we get and the buffer should be selected from the correct pool for the
    // specific stream ID.
    pub fn push_buffer(&mut self, stream_id: u8, buffer: Vec<u8>) {
        self.buffers.entry(stream_id).or_default().push_back(buffer);
    }

    // Receive a packet. If there are no available buffers for a specific stream ID pool, or the
    // available buffers are too small, a new buffer is allocated.
    // Note: recv() takes mut self because it cannot have concurrent send actions
    pub fn recv(&mut self) -> IntResult<(u8, Vec<u8>)> {
        if !self.valid {
            return interrupt();
        }

        let mut prefix = [0; 9];
        if let Err(e) = interruptible_read_all(&self.socket, &mut prefix, &self.running) {
            self.valid = false;
            return Err(e);
        }

        let stream_id = prefix[0];

        let mut buffer_size_buffer = [0; 8];
        buffer_size_buffer.copy_from_slice(&prefix[1..9]);
        let buffer_size = u64::from_le_bytes(buffer_size_buffer) as usize;

        let mut buffer = self
            .buffers
            .entry(stream_id)
            .or_default()
            .pop_front()
            .unwrap_or_default();

        // Note: it performs a reallocation if necessary
        buffer.resize(buffer_size, 0);

        if let Err(e) = interruptible_read_all(&self.socket, &mut buffer, &self.running) {
            self.valid = false;
            return Err(e);
        }

        Ok((stream_id, buffer))
    }
}
