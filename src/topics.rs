pub const CMD_VEL: &str = "/cmd_vel";
pub const JPEG: &str = "/CoreNode/jpg";
pub const H264: &str = "/CoreNode/h264";
pub const AAC: &str = "/CoreNode/aac";
pub const GREY_IMAGE: &str = "/CoreNode/grey_img";
pub const OBJECT_DETECTION: &str = "/CoreNode/obj";
pub const MOTION_DETECTION: &str = "/CoreNode/motion";
pub const GOING_HOME_STATUS: &str = "/CoreNode/going_home_status";
pub const IMU: &str = "/SensorNode/imu";
pub const TOF: &str = "/SensorNode/tof";
pub const LIGHT: &str = "/SensorNode/light";
pub const IBEACON: &str = "/SensorNode/ibeacon";
pub const BATTERY: &str = "/SensorNode/simple_battery_status";
pub const BASE_ODOMETRY: &str = "/baselink_odom_relative";
pub const VIO_ODOMETRY: &str = "/vio_odom_relative";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DriverSupport {
    Read,
    Write,
    Bridge,
    DiscoveredOnly,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct KnownTopic {
    pub name: &'static str,
    pub message_type: &'static str,
    pub capability: &'static str,
    pub support: DriverSupport,
}

pub const KNOWN_TOPICS: &[KnownTopic] = &[
    KnownTopic {
        name: CMD_VEL,
        message_type: "geometry_msgs/Twist",
        capability: "holonomic motion",
        support: DriverSupport::Write,
    },
    KnownTopic {
        name: JPEG,
        message_type: "roller_eye/frame",
        capability: "JPEG camera",
        support: DriverSupport::Bridge,
    },
    KnownTopic {
        name: H264,
        message_type: "roller_eye/frame",
        capability: "H.264 camera",
        support: DriverSupport::DiscoveredOnly,
    },
    KnownTopic {
        name: AAC,
        message_type: "roller_eye/frame",
        capability: "AAC microphone",
        support: DriverSupport::DiscoveredOnly,
    },
    KnownTopic {
        name: GREY_IMAGE,
        message_type: "sensor_msgs/Image",
        capability: "greyscale camera",
        support: DriverSupport::DiscoveredOnly,
    },
    KnownTopic {
        name: OBJECT_DETECTION,
        message_type: "roller_eye/detect",
        capability: "object detection",
        support: DriverSupport::DiscoveredOnly,
    },
    KnownTopic {
        name: MOTION_DETECTION,
        message_type: "roller_eye/detect",
        capability: "motion detection",
        support: DriverSupport::DiscoveredOnly,
    },
    KnownTopic {
        name: IMU,
        message_type: "sensor_msgs/Imu",
        capability: "6-axis IMU",
        support: DriverSupport::Read,
    },
    KnownTopic {
        name: TOF,
        message_type: "sensor_msgs/Range",
        capability: "time-of-flight range",
        support: DriverSupport::Read,
    },
    KnownTopic {
        name: LIGHT,
        message_type: "sensor_msgs/Illuminance",
        capability: "ambient light",
        support: DriverSupport::Read,
    },
    KnownTopic {
        name: IBEACON,
        message_type: "sensor_msgs/Range",
        capability: "charger beacon range",
        support: DriverSupport::DiscoveredOnly,
    },
    KnownTopic {
        name: BATTERY,
        message_type: "roller_eye/status",
        capability: "battery and charging state",
        support: DriverSupport::Read,
    },
    KnownTopic {
        name: BASE_ODOMETRY,
        message_type: "nav_msgs/Odometry",
        capability: "wheel odometry",
        support: DriverSupport::DiscoveredOnly,
    },
    KnownTopic {
        name: VIO_ODOMETRY,
        message_type: "nav_msgs/Odometry",
        capability: "visual-inertial odometry",
        support: DriverSupport::DiscoveredOnly,
    },
];

pub fn known_topic(name: &str) -> Option<&'static KnownTopic> {
    KNOWN_TOPICS.iter().find(|topic| topic.name == name)
}
