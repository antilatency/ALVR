#![allow(clippy::if_same_then_else)]

use crate::{
    platform, request,
    sockets::{self, AnnouncerSocket, ControlListenerSocket, RequestSocket, ScanResult, SseSocket},
    statistics::StatisticsManager,
    AlvrEvent, VideoFrame, DECODER_REF, EVENT_BUFFER, IDR_PARSED, IS_ALIVE, IS_RESUMED,
    REQUEST_SOCKET, SHOULD_STREAM, STATISTICS_MANAGER, STATISTICS_SENDER, TRACKING_SENDER,
};
use alvr_audio::{AudioDevice, AudioDeviceType};
use alvr_common::{glam::UVec2, once_cell::sync::Lazy, prelude::*, ALVR_NAME, ALVR_VERSION};
use alvr_session::{AudioDeviceId, CodecType, OculusFovetionLevel, SessionDesc};
use alvr_sockets::{
    spawn_cancelable, Haptics, RequestPacket, ResponsePacket, SsePacket, StreamConfigPacket,
    StreamSocketBuilder, VideoFrameHeaderPacket, VideoStreamingCapabilities, AUDIO_STREAM,
    HAPTICS_STREAM, STATISTICS_STREAM, TRACKING_STREAM, VIDEO_STREAM,
};
use futures::future::BoxFuture;
use glyph_brush_layout::{
    ab_glyph::{Font, FontRef, ScaleFont},
    FontId, GlyphPositioner, HorizontalAlign, Layout, SectionGeometry, SectionText, VerticalAlign,
};
use rand::distributions;
use serde_json as json;
use settings_schema::Switch;
use std::{
    future, mem,
    net::IpAddr,
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc as smpsc, Arc,
    },
    thread,
    time::Duration,
};
use tokio::{
    runtime::Runtime,
    sync::{mpsc as tmpsc, Mutex, Notify},
    task,
    time::{self, Instant},
};

#[cfg(target_os = "android")]
use crate::audio;
#[cfg(not(target_os = "android"))]
use alvr_audio as audio;

const INITIAL_MESSAGE: &str = "Searching for server...\n(open ALVR on your PC)";
const NETWORK_UNREACHABLE_MESSAGE: &str = "Cannot connect to the internet";
const CLIENT_UNTRUSTED_MESSAGE: &str = "On the PC, click \"Trust\"\nnext to the client entry";
const INCOMPATIBLE_VERSIONS_MESSAGE: &str = concat!(
    "Server and client have\n",
    "incompatible types.\n",
    "Please update either the app\n",
    "on the PC or on the headset"
);
const STREAM_STARTING_MESSAGE: &str = "The stream will begin soon\nPlease wait...";
const SERVER_RESTART_MESSAGE: &str = "The server is restarting\nPlease wait...";
const SERVER_DISCONNECTED_MESSAGE: &str = "The server has disconnected.";

const DISCOVERY_RETRY_PAUSE: Duration = Duration::from_millis(500);
const RETRY_CONNECT_MIN_INTERVAL: Duration = Duration::from_secs(1);
const NETWORK_KEEPALIVE_INTERVAL: Duration = Duration::from_secs(1);
const CONNECTION_ERROR_PAUSE: Duration = Duration::from_millis(500);

const LOADING_TEXTURE_WIDTH: usize = 1280;
const LOADING_TEXTURE_HEIGHT: usize = 720;
const FONT_SIZE: f32 = 50_f32;

fn set_lobby_message(message: &str) {
    let hostname = platform::load_config().hostname;

    let message = format!(
        "ALVR v{}\nhostname: {hostname}\n \n{message}",
        *ALVR_VERSION,
    );

    let ubuntu_font =
        FontRef::try_from_slice(include_bytes!("../resources/Ubuntu-Medium.ttf")).unwrap();

    let section_glyphs = Layout::default()
        .h_align(HorizontalAlign::Center)
        .v_align(VerticalAlign::Center)
        .calculate_glyphs(
            &[&ubuntu_font],
            &SectionGeometry {
                screen_position: (
                    LOADING_TEXTURE_WIDTH as f32 / 2_f32,
                    LOADING_TEXTURE_HEIGHT as f32 / 2_f32,
                ),
                ..Default::default()
            },
            &[SectionText {
                text: &message,
                scale: FONT_SIZE.into(),
                font_id: FontId(0),
            }],
        );

    let scaled_font = ubuntu_font.as_scaled(FONT_SIZE);

    let mut buffer = vec![0_u8; LOADING_TEXTURE_WIDTH * LOADING_TEXTURE_HEIGHT * 4];

    for section_glyph in section_glyphs {
        if let Some(outlined) = scaled_font.outline_glyph(section_glyph.glyph) {
            let bounds = outlined.px_bounds();
            outlined.draw(|x, y, alpha| {
                let x = x as usize + bounds.min.x as usize;
                let y = y as usize + bounds.min.y as usize;
                buffer[(y * LOADING_TEXTURE_WIDTH + x) * 4 + 3] = (alpha * 255.0) as u8;
            });
        }
    }

    unsafe { crate::updateLoadingTexuture(buffer.as_ptr()) };
}

pub fn connection_lifecycle_loop(
    default_view_resolution: UVec2,
    supported_refresh_rates: Vec<f32>,
) {
    set_lobby_message(INITIAL_MESSAGE);

    loop {
        match connection_pipeline(default_view_resolution, supported_refresh_rates.clone()) {
            Ok(()) => continue,
            Err(InterruptibleError::Interrupted) => return,
            Err(InterruptibleError::Other(e)) => {
                let message = format!("Connection error:\n{e}\nCheck the PC for more details");
                error!("{message}");
                set_lobby_message(&message);

                // avoid spamming error messages
                thread::sleep(CONNECTION_ERROR_PAUSE);
            }
        }
    }
}

fn connection_pipeline(
    default_view_resolution: UVec2,
    supported_refresh_rates: Vec<f32>,
) -> IntResult {
    let (server_ip, request_socket, mut sse_socket) = {
        let config = platform::load_config();
        let announcer_socket = AnnouncerSocket::new(&config.hostname).map_err(to_int_e!())?;
        let listener_socket =
            ControlListenerSocket::new(Arc::clone(&IS_ALIVE)).map_err(to_int_e!())?;

        loop {
            check_interrupt!(IS_ALIVE.value());

            if let Err(e) = announcer_socket.broadcast() {
                warn!("Broadcast error: {e}");
                set_lobby_message(NETWORK_UNREACHABLE_MESSAGE);

                thread::sleep(RETRY_CONNECT_MIN_INTERVAL);

                set_lobby_message(INITIAL_MESSAGE);

                return Ok(());
            }

            match listener_socket.scan() {
                Ok(ScanResult::Connected {
                    server_ip,
                    request_socket,
                    sse_socket,
                }) => break (server_ip, request_socket, sse_socket),
                Ok(ScanResult::UnrelatedPeer) => info!("Found unrelated peer"),
                Ok(ScanResult::MismatchedVersion) => {
                    warn!("Found server with wrong version");
                    set_lobby_message(INCOMPATIBLE_VERSIONS_MESSAGE);
                }
                Err(InterruptibleError::Interrupted) => (), // Interrupts here are recoverable
                e => (),
            };

            thread::sleep(DISCOVERY_RETRY_PAUSE);
        }
    };
    *REQUEST_SOCKET.lock() = Some(request_socket);

    let microphone_sample_rate =
        AudioDevice::new(None, AudioDeviceId::Default, AudioDeviceType::Input)
            .unwrap()
            .input_sample_rate()
            .unwrap();
    // Advertise this client as streaming-capable
    request(RequestPacket::ClientCapabilities(Some(
        VideoStreamingCapabilities {
            default_view_resolution,
            supported_refresh_rates,
            microphone_sample_rate,
        },
    )))
    .map_err(int_e!())?;

    // the stream socket is still async and requires tokio
    let mut runtime = None;

    loop {
        match sse_socket.poll()? {
            SsePacket::StartStreaming(config_packet) => {
                runtime = None;

                SHOULD_STREAM.set(true);

                let new_runtime = Runtime::new().unwrap();

                new_runtime
                    .spawn(async move { streaming_pipeline(server_ip, config_packet).await });

                runtime = Some(new_runtime);
            }
            SsePacket::StopStreaming => {
                SHOULD_STREAM.set(false);
                drop(runtime.take());
            }
            SsePacket::Event(_) => (), // todo
            SsePacket::ServerRestarting => {
                info!("Server restarting");
                set_lobby_message(SERVER_RESTART_MESSAGE);

                break Ok(());
            }
            _ => (),
        }
        // let fjdks = sse_socket.
    }
}

fn on_server_connected(
    eye_width: i32,
    eye_height: i32,
    fps: f32,
    codec: CodecType,
    realtime_decoder: bool,
    oculus_foveation_level: OculusFovetionLevel,
    dynamic_oculus_foveation: bool,
    extra_latency: bool,
    controller_prediction_multiplier: f32,
) {
    let vm = platform::vm();
    let env = vm.attach_current_thread().unwrap();

    env.call_method(
        platform::context(),
        "onServerConnected",
        "(IIFIZIZZF)V",
        &[
            eye_width.into(),
            eye_height.into(),
            fps.into(),
            (matches!(codec, CodecType::HEVC) as i32).into(),
            realtime_decoder.into(),
            (oculus_foveation_level as i32).into(),
            dynamic_oculus_foveation.into(),
            extra_latency.into(),
            controller_prediction_multiplier.into(),
        ],
    )
    .unwrap();
}

async fn streaming_pipeline(server_ip: IpAddr, config_packet: StreamConfigPacket) -> IntResult {
    let settings = {
        let session_json = match request(RequestPacket::Session).map_err(int_e!())? {
            ResponsePacket::Session(session) => session,
            other => return int_fmt_e!("Unexpected response: {other:?}"),
        };

        let mut session_desc = SessionDesc::default();
        session_desc
            .merge_from_json(&json::from_str(&session_json).map_err(to_int_e!())?)
            .map_err(to_int_e!())?;
        session_desc.to_settings()
    };

    // todo: make event-based
    {
        let mut config = platform::load_config();
        config.dark_mode = settings.extra.client_dark_mode;
        platform::store_config(&config);
    }

    *STATISTICS_MANAGER.lock() = Some(StatisticsManager::new(
        settings.connection.statistics_history_size as _,
    ));

    let stream_socket_builder = StreamSocketBuilder::listen_for_server(
        settings.connection.stream_port,
        settings.connection.stream_protocol,
    )
    .await
    .map_err(to_int_e!())?;

    request(RequestPacket::StreamSocketReady).map_err(int_e!())?;

    let stream_socket = tokio::select! {
        res = stream_socket_builder.accept_from_server(
            server_ip,
            settings.connection.stream_port,
        ) => res.map_err(to_int_e!())?,
        _ = time::sleep(Duration::from_secs(5)) => {
            return int_fmt_e!("Timeout while setting up streams");
        }
    };
    let stream_socket = Arc::new(stream_socket);

    unsafe {
        crate::setStreamConfig(crate::StreamConfigInput {
            eyeWidth: config_packet.view_resolution.x,
            eyeHeight: config_packet.view_resolution.y,
            enableFoveation: matches!(settings.video.foveated_rendering, Switch::Enabled(_)),
            foveationCenterSizeX: if let Switch::Enabled(foveation_vars) =
                &settings.video.foveated_rendering
            {
                foveation_vars.center_size_x
            } else {
                3_f32 / 5_f32
            },
            foveationCenterSizeY: if let Switch::Enabled(foveation_vars) =
                &settings.video.foveated_rendering
            {
                foveation_vars.center_size_y
            } else {
                2_f32 / 5_f32
            },
            foveationCenterShiftX: if let Switch::Enabled(foveation_vars) =
                &settings.video.foveated_rendering
            {
                foveation_vars.center_shift_x
            } else {
                2_f32 / 5_f32
            },
            foveationCenterShiftY: if let Switch::Enabled(foveation_vars) =
                &settings.video.foveated_rendering
            {
                foveation_vars.center_shift_y
            } else {
                1_f32 / 10_f32
            },
            foveationEdgeRatioX: if let Switch::Enabled(foveation_vars) =
                &settings.video.foveated_rendering
            {
                foveation_vars.edge_ratio_x
            } else {
                2_f32
            },
            foveationEdgeRatioY: if let Switch::Enabled(foveation_vars) =
                &settings.video.foveated_rendering
            {
                foveation_vars.edge_ratio_y
            } else {
                2_f32
            },
        });
    }

    on_server_connected(
        config_packet.view_resolution.x as _,
        config_packet.view_resolution.y as _,
        config_packet.fps,
        settings.video.codec,
        settings.video.client_request_realtime_decoder,
        if let Switch::Enabled(foveation_vars) = &settings.video.foveated_rendering {
            foveation_vars.oculus_foveation_level
        } else {
            OculusFovetionLevel::None
        },
        if let Switch::Enabled(foveation_vars) = &settings.video.foveated_rendering {
            foveation_vars.dynamic_oculus_foveation
        } else {
            false
        },
        settings.headset.extra_latency_mode,
        settings
            .headset
            .controllers
            .into_option()
            .map(|c| c.prediction_multiplier)
            .unwrap_or_default(),
    );

    let tracking_send_loop = {
        let mut socket_sender = stream_socket
            .request_stream(TRACKING_STREAM)
            .await
            .map_err(to_int_e!())?;
        async move {
            let (data_sender, mut data_receiver) = tmpsc::unbounded_channel();
            *TRACKING_SENDER.lock() = Some(data_sender);
            while let Some(tracking) = data_receiver.recv().await {
                socket_sender
                    .send_buffer(socket_sender.new_buffer(&tracking, 0)?)
                    .await
                    .ok();

                // Note: this is not the best place to report the acquired input. Instead it should
                // be done as soon as possible (or even just before polling the input). Instead this
                // is reported late to partially compensate for lack of network latency measurement,
                // so the server can just use total_pipeline_latency as the postTimeoffset.
                // This hack will be removed once poseTimeOffset can be calculated more accurately.
                if let Some(stats) = &mut *STATISTICS_MANAGER.lock() {
                    stats.report_input_acquired(tracking.target_timestamp);
                }
            }

            Ok(())
        }
    };

    let statistics_send_loop = {
        let mut socket_sender = stream_socket
            .request_stream(STATISTICS_STREAM)
            .await
            .map_err(to_int_e!())?;
        async move {
            let (data_sender, mut data_receiver) = tmpsc::unbounded_channel();
            *STATISTICS_SENDER.lock() = Some(data_sender);
            while let Some(stats) = data_receiver.recv().await {
                socket_sender
                    .send_buffer(socket_sender.new_buffer(&stats, 0)?)
                    .await
                    .ok();
            }

            Ok(())
        }
    };

    let (legacy_receive_data_sender, legacy_receive_data_receiver) = smpsc::channel();
    let legacy_receive_data_sender = Arc::new(Mutex::new(legacy_receive_data_sender));

    let video_receive_loop = {
        let mut receiver = stream_socket
            .subscribe_to_stream::<VideoFrameHeaderPacket>(VIDEO_STREAM)
            .await
            .map_err(to_int_e!())?;
        let legacy_receive_data_sender = legacy_receive_data_sender.clone();
        async move {
            loop {
                let packet = receiver.recv().await?;

                let mut buffer = vec![0_u8; mem::size_of::<VideoFrame>() + packet.buffer.len()];
                let header = VideoFrame {
                    packetCounter: packet.header.packet_counter,
                    trackingFrameIndex: packet.header.tracking_frame_index,
                    videoFrameIndex: packet.header.video_frame_index,
                    sentTime: packet.header.sent_time,
                    frameByteSize: packet.header.frame_byte_size,
                    fecIndex: packet.header.fec_index,
                    fecPercentage: packet.header.fec_percentage,
                };

                buffer[..mem::size_of::<VideoFrame>()].copy_from_slice(unsafe {
                    &mem::transmute::<_, [u8; mem::size_of::<VideoFrame>()]>(header)
                });
                buffer[mem::size_of::<VideoFrame>()..].copy_from_slice(&packet.buffer);

                if let Some(stats) = &mut *STATISTICS_MANAGER.lock() {
                    stats.report_video_packet_received(Duration::from_nanos(
                        packet.header.tracking_frame_index,
                    ));
                }

                legacy_receive_data_sender.lock().await.send(buffer).ok();
            }
        }
    };

    let haptics_receive_loop = {
        let mut receiver = stream_socket
            .subscribe_to_stream::<Haptics>(HAPTICS_STREAM)
            .await
            .map_err(to_int_e!())?;
        async move {
            loop {
                let packet = receiver.recv().await?.header;

                EVENT_BUFFER.lock().push_back(AlvrEvent::Haptics {
                    device_id: packet.path,
                    duration_s: packet.duration.as_secs_f32(),
                    frequency: packet.frequency,
                    amplitude: packet.amplitude,
                });
            }
        }
    };

    // The main stream loop must be run in a normal thread, because it needs to access the JNI env
    // many times per second. If using a future I'm forced to attach and detach the env continuously.
    // When the parent function exits or gets canceled, this loop will run to finish.
    let legacy_stream_socket_loop = task::spawn_blocking({
        let codec = settings.video.codec;
        let enable_fec = settings.connection.enable_fec;
        move || -> StrResult {
            unsafe {
                // Note: legacyReceive() requires the java context to be attached to the current thread
                // todo: investigate why
                let vm = platform::vm();
                let env = vm.attach_current_thread().unwrap();

                crate::initializeSocket(matches!(codec, CodecType::HEVC) as _, enable_fec);

                let mut idr_request_deadline = None;

                while let Ok(mut data) = legacy_receive_data_receiver.recv() {
                    if !SHOULD_STREAM.value() {
                        break;
                    }

                    // Send again IDR packet every 2s in case it is missed
                    // (due to dropped burst of packets at the start of the stream or otherwise).
                    if !IDR_PARSED.load(Ordering::Relaxed) {
                        if let Some(deadline) = idr_request_deadline {
                            if deadline < Instant::now() {
                                request(RequestPacket::RequestIdr).ok();
                                idr_request_deadline = None;
                            }
                        } else {
                            idr_request_deadline = Some(Instant::now() + Duration::from_secs(2));
                        }
                    }

                    crate::legacyReceive(data.as_mut_ptr(), data.len() as _);
                }

                crate::closeSocket();

                let mut decoder = DECODER_REF.lock();

                if let Some(decoder) = &*decoder {
                    env.call_method(decoder.as_obj(), "onDisconnect", "()V", &[])
                        .unwrap();

                    env.call_method(decoder.as_obj(), "stopAndWait", "()V", &[])
                        .unwrap();
                }

                *decoder = None;
            }

            Ok(())
        }
    });

    let game_audio_loop: BoxFuture<_> = if let Switch::Enabled(desc) = settings.audio.game_audio {
        let device = AudioDevice::new(None, AudioDeviceId::Default, AudioDeviceType::Output)
            .map_err(to_int_e!())?;

        let game_audio_receiver = stream_socket
            .subscribe_to_stream(AUDIO_STREAM)
            .await
            .map_err(to_int_e!())?;
        Box::pin(audio::play_audio_loop(
            device,
            2,
            config_packet.game_audio_sample_rate,
            desc.buffering_config,
            game_audio_receiver,
        ))
    } else {
        Box::pin(future::pending())
    };

    let microphone_loop: BoxFuture<_> = if matches!(settings.audio.microphone, Switch::Enabled(_)) {
        let device = AudioDevice::new(None, AudioDeviceId::Default, AudioDeviceType::Input)
            .map_err(to_int_e!())?;

        let microphone_sender = stream_socket
            .request_stream(AUDIO_STREAM)
            .await
            .map_err(to_int_e!())?;
        Box::pin(audio::record_audio_loop(
            device,
            1,
            false,
            microphone_sender,
        ))
    } else {
        Box::pin(future::pending())
    };

    let keepalive_sender_loop = async move {
        loop {
            let res = request(RequestPacket::KeepAlive);
            if let Err(e) = res {
                info!("Server disconnected. Cause: {e}");
                set_lobby_message(SERVER_DISCONNECTED_MESSAGE);
                break Ok(());
            }

            time::sleep(NETWORK_KEEPALIVE_INTERVAL).await;
        }
    };

    let receive_loop = async move { stream_socket.receive_loop().await };

    // Run many tasks concurrently. Threading is managed by the runtime, for best performance.
    tokio::select! {
        res = spawn_cancelable(receive_loop) => {
            if let Err(e) = res {
                info!("Server disconnected. Cause: {e}");
            }
            set_lobby_message(SERVER_DISCONNECTED_MESSAGE);

            Ok(())
        },
        res = spawn_cancelable(game_audio_loop) => res,
        res = spawn_cancelable(microphone_loop) => res,
        res = spawn_cancelable(tracking_send_loop) => res,
        res = spawn_cancelable(statistics_send_loop) => res,
        res = spawn_cancelable(video_receive_loop) => res,
        res = spawn_cancelable(haptics_receive_loop) => res,
        res = legacy_stream_socket_loop => res.map_err(to_int_e!())?,

        // keep these loops on the current task
        res = keepalive_sender_loop => res,
    }
    .map_err(to_int_e!())
}
