use std::{net::IpAddr, path::PathBuf, time::Duration};

use alvr_common::{
    glam::{Quat, UVec2, Vec2, Vec3},
    semver::Version,
};
use alvr_events::{ButtonValue, Event};
use alvr_session::Fov;
use serde::{Deserialize, Serialize};

pub const HANDSHAKE_STREAM: u8 = 0;
pub const REQUEST_STREAM: u8 = 1;
pub const SSE_STREAM: u8 = 2;

pub const TRACKING_STREAM: u16 = 0;
pub const HAPTICS_STREAM: u16 = 1;
pub const AUDIO_STREAM: u16 = 2;
pub const VIDEO_STREAM: u16 = 3;
pub const STATISTICS_STREAM: u16 = 4;

#[derive(Serialize, Deserialize, Clone)]
pub struct StreamConfigPacket {
    pub view_resolution: UVec2,
    pub fps: f32,
    pub game_audio_sample_rate: u32,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct ViewsConfig {
    // Note: the head-to-eye transform is always a translation along the x axis
    pub ipd_m: f32,
    pub fov: [Fov; 2],
}

#[derive(Serialize, Deserialize, Clone)]
pub struct BatteryPacket {
    pub device_id: u64,
    pub gauge_value: f32, // range [0, 1]
    pub is_plugged: bool,
}

// legacy video packet
#[derive(Serialize, Deserialize, Clone)]
pub struct VideoFrameHeaderPacket {
    pub packet_counter: u32,
    pub tracking_frame_index: u64,
    pub video_frame_index: u64,
    pub sent_time: u64,
    pub frame_byte_size: u32,
    pub fec_index: u32,
    pub fec_percentage: u16,
}

#[derive(Serialize, Deserialize, Clone, Default, Debug)]
pub struct DeviceMotion {
    pub orientation: Quat,
    pub position: Vec3,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
}

#[derive(Serialize, Deserialize)]
pub struct Tracking {
    pub target_timestamp: Duration,
    pub device_motions: Vec<(u64, DeviceMotion)>,
    pub left_hand_skeleton: Option<[Quat; 19]>, // legacy oculus hand
    pub right_hand_skeleton: Option<[Quat; 19]>, // legacy oculus hand
}

#[derive(Serialize, Deserialize)]
pub struct Haptics {
    pub path: u64,
    pub duration: Duration,
    pub frequency: f32,
    pub amplitude: f32,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct AudioDevicesList {
    pub output: Vec<String>,
    pub input: Vec<String>,
}

pub enum GpuVendor {
    Nvidia,
    Amd,
    Other,
}

#[derive(Clone, Debug)]
pub enum PathSegment {
    Name(String),
    Index(usize),
}

#[derive(Serialize, Deserialize, Clone)]
pub enum ClientListAction {
    AddIfMissing,
    SetDisplayName(String),
    Trust,
    AddIp(IpAddr),
    RemoveIp(IpAddr),
    RemoveEntry,
}

#[derive(Serialize, Deserialize, Default, Clone)]
pub struct ClientStatistics {
    pub target_timestamp: Duration, // identifies the frame
    pub frame_interval: Duration,
    pub video_decode: Duration,
    pub rendering: Duration,
    pub vsync_queue: Duration,
    pub total_pipeline_latency: Duration,

    // Note: This is used for the controller prediction.
    // NB: This contains also the tracking packet send latency so it might lead to overprediction
    pub average_total_pipeline_latency: Duration,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct VideoStreamingCapabilities {
    pub default_view_resolution: UVec2,
    pub supported_refresh_rates: Vec<f32>,
    pub microphone_sample_rate: u32,
}

#[derive(Serialize, Deserialize, Clone)]
pub enum RequestPacket {
    Session,
    StoreSession(String),  // json
    StoreSettings(String), // legacy
    RegisterDriver,
    UnregisterDriver,
    RegisteredDrivers,
    AudioDevices,
    GraphicsDevice,
    RestartSteamvr,
    ClientListAction(ClientListAction),
    ServerVersion,
    ServerOs,
    UpdateServer { download_url: String },
    ClientCapabilities(Option<VideoStreamingCapabilities>),
    StreamSocketReady,
    PlayspaceSync(Vec2),
    RequestIdr,
    KeepAlive,
    ViewsConfig(ViewsConfig),
    Battery(BatteryPacket),
    VideoErrorReport, // legacy
    Button { path_id: u64, value: ButtonValue },
    ActiveInteractionProfile { device_id: u64, profile_id: u64 },
    Custom(String), // use json
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum ResponsePacket {
    Ok,
    Err(String),
    Session(String), // json. Not using SessionDesc directly to allow interpolation
    RegisteredDrivers(Vec<PathBuf>),
    AudioDevices(AudioDevicesList),
    GraphicsDevice(String),
    ServerVersion(Version),
    ServerOs(String),
    Custom(String), // use json
}

#[derive(Serialize, Deserialize, Clone)]
pub enum SsePacket {
    StartStreaming(StreamConfigPacket),
    StopStreaming,
    Event(Event),
    ServerRestarting, // legacy
    KeepAlive,
    Custom(String), // use json
}
