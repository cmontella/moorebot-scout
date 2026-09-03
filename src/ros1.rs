//! ROS 1 transport built on raw messages.
//!
//! Raw messages let this crate build on machines that do not have a ROS
//! distribution installed. The Scout itself provides the ROS master and the
//! message publishers/subscribers.

use crate::{
    codec::{write_bytes, write_string, write_u32},
    frame::{ScoutFrame, StreamType},
    motion::ScoutTwist,
};
use rosrust::{Publisher, RawMessage, RawMessageDescription, api::raii::Subscriber};
use std::{
    fmt,
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
    time::{Duration, Instant},
};

const TWIST_MD5: &str = "9f195f881246fdfa2798d1d3eebca84a";
const TWIST_DEFINITION: &str = "geometry_msgs/Vector3 linear\ngeometry_msgs/Vector3 angular\n================================================================================\nMSG: geometry_msgs/Vector3\nfloat64 x\nfloat64 y\nfloat64 z\n";

const COMPRESSED_IMAGE_MD5: &str = "8f7a12909da2c9d3332d540a0977563f";
const COMPRESSED_IMAGE_DEFINITION: &str = "std_msgs/Header header\nstring format\nuint8[] data\n================================================================================\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n";

#[derive(Clone, Debug)]
pub struct Ros1Config {
    pub master_uri: String,
    /// Address on this computer that the Scout can reach.
    pub advertise_address: Option<String>,
    pub node_name: String,
}

impl Default for Ros1Config {
    fn default() -> Self {
        Self {
            master_uri: "http://10.42.0.1:11311".into(),
            advertise_address: None,
            node_name: "moorebot_scout".into(),
        }
    }
}

/// Initializes the process-wide ROS client configuration.
///
/// # Safety
///
/// On platforms where process-environment access is not thread-safe, the
/// caller must ensure no other threads are reading or writing environment
/// variables while this function runs. Call this once, before starting any
/// application threads.
pub unsafe fn init(config: &Ros1Config, capture_sigint: bool) -> Result<(), Ros1Error> {
    // SAFETY: The caller upholds the process-environment synchronization
    // requirement documented above.
    unsafe {
        std::env::set_var("ROS_MASTER_URI", &config.master_uri);
        if let Some(address) = &config.advertise_address {
            // ROS_HOSTNAME has priority over ROS_IP in rosrust, so clear it
            // when the caller explicitly supplies a reachable address.
            std::env::remove_var("ROS_HOSTNAME");
            std::env::set_var("ROS_IP", address);
        }
    }
    rosrust::try_init_with_options(&config.node_name, capture_sigint)
        .map_err(|error| Ros1Error(error.to_string()))
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PublishedTopic {
    pub name: String,
    pub message_type: String,
}

pub fn published_topics() -> Result<Vec<PublishedTopic>, Ros1Error> {
    let topics = rosrust::topics().map_err(|error| Ros1Error(error.to_string()))?;
    Ok(topics
        .into_iter()
        .map(|topic| PublishedTopic {
            name: topic.name,
            message_type: topic.datatype,
        })
        .collect())
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RegisteredService {
    pub name: String,
    pub providers: Vec<String>,
}

pub fn registered_services() -> Result<Vec<RegisteredService>, Ros1Error> {
    let state = rosrust::state().map_err(|error| Ros1Error(error.to_string()))?;
    Ok(state
        .services
        .into_iter()
        .map(|service| RegisteredService {
            name: service.name,
            providers: service.connections,
        })
        .collect())
}

#[derive(Clone)]
pub struct TwistPublisher {
    inner: Publisher<RawMessage>,
}

impl TwistPublisher {
    pub fn new(topic: &str, queue_size: usize) -> Result<Self, Ros1Error> {
        let inner = rosrust::publish_with_description::<RawMessage>(
            topic,
            queue_size,
            RawMessageDescription {
                msg_definition: TWIST_DEFINITION.into(),
                md5sum: TWIST_MD5.into(),
                msg_type: "geometry_msgs/Twist".into(),
            },
        )
        .map_err(|error| Ros1Error(error.to_string()))?;
        Ok(Self { inner })
    }

    pub fn send(&self, twist: ScoutTwist) -> Result<(), Ros1Error> {
        self.inner
            .send(RawMessage(twist.encode_ros()))
            .map_err(|error| Ros1Error(error.to_string()))
    }

    pub fn subscriber_count(&self) -> usize {
        self.inner.subscriber_count()
    }

    pub fn wait_for_a_subscriber(&self, timeout: Duration) -> bool {
        let deadline = Instant::now() + timeout;
        while rosrust::is_ok() && Instant::now() < deadline {
            if self.subscriber_count() > 0 {
                return true;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        self.subscriber_count() > 0
    }
}

pub fn subscribe_raw<F>(
    topic: &str,
    queue_size: usize,
    callback: F,
) -> Result<Subscriber, Ros1Error>
where
    F: Fn(Vec<u8>) + Send + 'static,
{
    rosrust::subscribe::<RawMessage, _>(topic, queue_size, move |message| callback(message.0))
        .map_err(|error| Ros1Error(error.to_string()))
}

pub struct CameraBridge {
    _publisher: Publisher<RawMessage>,
    _subscriber: Subscriber,
    forwarded: Arc<AtomicU64>,
    dropped: Arc<AtomicU64>,
}

impl CameraBridge {
    /// Republish the Scout's custom JPEG frames as
    /// `sensor_msgs/CompressedImage` for standard ROS tooling.
    pub fn start(source_topic: &str, output_topic: &str) -> Result<Self, Ros1Error> {
        let publisher = rosrust::publish_with_description::<RawMessage>(
            output_topic,
            3,
            RawMessageDescription {
                msg_definition: COMPRESSED_IMAGE_DEFINITION.into(),
                md5sum: COMPRESSED_IMAGE_MD5.into(),
                msg_type: "sensor_msgs/CompressedImage".into(),
            },
        )
        .map_err(|error| Ros1Error(error.to_string()))?;

        let forwarded = Arc::new(AtomicU64::new(0));
        let dropped = Arc::new(AtomicU64::new(0));
        let callback_publisher = publisher.clone();
        let callback_forwarded = Arc::clone(&forwarded);
        let callback_dropped = Arc::clone(&dropped);

        // Live video should prefer the newest frame. Keeping only one pending
        // input also limits retained memory when a publisher sends at a rate
        // the bridge cannot sustain.
        let subscriber = subscribe_raw(source_topic, 1, move |bytes| {
            let Ok(frame) = ScoutFrame::decode_ros(&bytes) else {
                callback_dropped.fetch_add(1, Ordering::Relaxed);
                return;
            };
            if frame.stream_type != StreamType::Jpeg || !frame.has_jpeg_markers() {
                callback_dropped.fetch_add(1, Ordering::Relaxed);
                return;
            }

            let now = rosrust::now();
            let body = encode_compressed_image(
                frame.sequence,
                now.sec,
                now.nsec,
                "scout_camera",
                &frame.data,
            );
            if callback_publisher.send(RawMessage(body)).is_ok() {
                callback_forwarded.fetch_add(1, Ordering::Relaxed);
            } else {
                callback_dropped.fetch_add(1, Ordering::Relaxed);
            }
        })?;

        Ok(Self {
            _publisher: publisher,
            _subscriber: subscriber,
            forwarded,
            dropped,
        })
    }

    pub fn forwarded_frames(&self) -> u64 {
        self.forwarded.load(Ordering::Relaxed)
    }

    pub fn dropped_frames(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }
}

fn encode_compressed_image(
    sequence: u32,
    seconds: u32,
    nanoseconds: u32,
    frame_id: &str,
    jpeg: &[u8],
) -> Vec<u8> {
    let mut output = Vec::with_capacity(24 + frame_id.len() + jpeg.len());
    write_u32(&mut output, sequence);
    write_u32(&mut output, seconds);
    write_u32(&mut output, nanoseconds);
    write_string(&mut output, frame_id);
    write_string(&mut output, "jpeg");
    write_bytes(&mut output, jpeg);
    output
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Ros1Error(pub String);

impl fmt::Display for Ros1Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Ros1Error {}

#[cfg(test)]
mod tests {
    use super::encode_compressed_image;

    #[test]
    fn compressed_image_has_ros_field_order() {
        let body = encode_compressed_image(4, 5, 6, "cam", &[0xff, 0xd8, 0xff, 0xd9]);
        assert_eq!(&body[0..4], &4_u32.to_le_bytes());
        assert_eq!(&body[4..8], &5_u32.to_le_bytes());
        assert_eq!(&body[8..12], &6_u32.to_le_bytes());
        assert_eq!(&body[12..16], &3_u32.to_le_bytes());
        assert_eq!(&body[16..19], b"cam");
        assert_eq!(&body[19..23], &4_u32.to_le_bytes());
        assert_eq!(&body[23..27], b"jpeg");
        assert_eq!(&body[27..31], &4_u32.to_le_bytes());
        assert_eq!(&body[31..], &[0xff, 0xd8, 0xff, 0xd9]);
    }
}
