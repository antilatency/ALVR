use alvr_common::{prelude::*, RelaxedAtomic, ALVR_NAME};
use alvr_sockets::{
    LdcTcpReceiver, LdcTcpSender, RequestPacket, ResponsePacket, SsePacket, CONTROL_PORT,
    HANDSHAKE_PACKET_SIZE_BYTES, LOCAL_IP, REQUEST_STREAM, SSE_STREAM,
};
use futures::stream::Scan;
use serde::{de::DeserializeOwned, Serialize};
use std::{
    io::ErrorKind,
    marker::PhantomData,
    net::{IpAddr, Ipv4Addr, TcpListener, UdpSocket},
    sync::{mpsc, Arc},
    time::Duration,
};

pub struct AnnouncerSocket {
    socket: UdpSocket,
    packet: [u8; 56],
}

impl AnnouncerSocket {
    pub fn new(hostname: &str) -> StrResult<Self> {
        let socket = UdpSocket::bind((LOCAL_IP, CONTROL_PORT)).map_err(err!())?;
        socket.set_broadcast(true).map_err(err!())?;

        let mut packet = [0; 56];
        packet[0..ALVR_NAME.len()].copy_from_slice(ALVR_NAME.as_bytes());
        packet[16..24].copy_from_slice(&alvr_common::protocol_id().to_le_bytes());
        packet[24..hostname.len()].copy_from_slice(hostname.as_bytes());

        Ok(Self { socket, packet })
    }

    pub fn broadcast(&self) -> StrResult {
        self.socket
            .send_to(&self.packet, (Ipv4Addr::BROADCAST, CONTROL_PORT))
            .map_err(err!())?;
        Ok(())
    }
}

pub struct RequestSocket {
    send_socket: LdcTcpSender,
    response_receiver: mpsc::Receiver<Vec<u8>>,
}

impl RequestSocket {
    pub fn request(&mut self, message: RequestPacket) -> IntResult<ResponsePacket> {
        let buffer = bincode::serialize(&message).map_err(to_int_e!())?;

        self.send_socket
            .send(REQUEST_STREAM, &buffer)
            .map_err(int_e!())?;

        // the SSE end will send a None packet to signal failure
        let buffer = self.response_receiver.recv().map_err(to_int_e!())?;
        bincode::deserialize(&buffer).map_err(to_int_e!())
    }
}

pub struct SseSocket {
    receive_socket: LdcTcpReceiver,
    response_sender: mpsc::SyncSender<Vec<u8>>,
}

impl SseSocket {
    pub fn poll(&mut self) -> IntResult<SsePacket> {
        loop {
            let (stream_id, buffer) = self.receive_socket.recv().map_err(int_e!())?;

            if stream_id == SSE_STREAM {
                let message = bincode::deserialize(&buffer).map_err(to_int_e!())?;
                self.receive_socket.push_buffer(stream_id, buffer);

                break Ok(message);
            } else {
                // stream_id == REQUEST_STREAM
                self.response_sender.send(buffer).map_err(to_int_e!())?;
                // Note: no need to push back the buffer, it is consumed

                continue;
            }
        }
    }
}

pub enum ScanResult {
    Connected {
        server_ip: IpAddr,
        request_socket: RequestSocket,
        sse_socket: SseSocket,
    },
    UnrelatedPeer,
    MismatchedVersion,
}

pub struct ControlListenerSocket {
    running: Arc<RelaxedAtomic>,
    inner: TcpListener,
}

impl ControlListenerSocket {
    pub fn new(running: Arc<RelaxedAtomic>) -> StrResult<Self> {
        let inner = TcpListener::bind((LOCAL_IP, CONTROL_PORT)).map_err(err!())?;
        inner.set_nonblocking(true).map_err(err!())?;

        Ok(Self { running, inner })
    }

    pub fn scan(&self) -> IntResult<ScanResult> {
        let (socket, server_address) = match self.inner.accept() {
            Ok(pair) => pair,
            Err(e) => {
                if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::Interrupted {
                    return interrupt();
                } else {
                    return int_fmt_e!("{e}");
                }
            }
        };

        socket.set_nonblocking(true).map_err(to_int_e!())?; // check if necessary

        let send_socket = LdcTcpSender::new(
            socket.try_clone().map_err(to_int_e!())?,
            Arc::clone(&self.running),
        );
        let mut receive_socket = LdcTcpReceiver::new(socket, Arc::clone(&self.running));
        let (response_sender, response_receiver) = mpsc::sync_channel(0);

        // Check server compatibility
        let (_, packet) = receive_socket.recv().map_err(int_e!())?;

        if &packet[0..ALVR_NAME.len()] != ALVR_NAME.as_bytes()
            || !packet[ALVR_NAME.len()..16].iter().all(|b| *b == 0)
        {
            return Ok(ScanResult::UnrelatedPeer);
        } else if packet[16..24] != alvr_common::protocol_id().to_le_bytes() {
            return Ok(ScanResult::MismatchedVersion);
        }

        let request_socket = RequestSocket {
            send_socket,
            response_receiver,
        };
        let sse_socket = SseSocket {
            receive_socket,
            response_sender,
        };

        Ok(ScanResult::Connected {
            server_ip: server_address.ip(),
            request_socket,
            sse_socket,
        })
    }
}
