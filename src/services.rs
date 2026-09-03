//! Known services found in the first-party Scout source release.
//!
//! These are intentionally discovery-only in this first version. Their exact
//! behavior and firmware compatibility need to be verified on physical
//! hardware before the driver makes them callable.

pub const ADJUST_LIGHT: &str = "/CoreNode/adjust_light";
pub const NIGHT_STATUS: &str = "/CoreNode/night_get";
pub const VIDEO_RESOLUTION: &str = "/CoreNode/video_set_resolution";
pub const NAV_PATROL: &str = "/NavPathNode/nav_patrol";
pub const NAV_PATROL_STOP: &str = "/NavPathNode/nav_patrol_stop";
pub const NAV_STATUS: &str = "/NavPathNode/nav_get_status";
pub const NAV_LIST_PATHS: &str = "/NavPathNode/nav_list_path";
pub const IMU_CALIBRATE: &str = "/imu_calib";
pub const ALGO_ACTION: &str = "/UtilNode/algo_action";
pub const ALGO_MOVE: &str = "/UtilNode/algo_move";
pub const ALGO_ROLL: &str = "/UtilNode/algo_roll";
pub const RECORD_START: &str = "/RecorderAgentNode/record_start";
pub const RECORD_STOP: &str = "/RecorderAgentNode/record_stop";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct KnownService {
    pub name: &'static str,
    pub service_type: &'static str,
    pub capability: &'static str,
}

pub const KNOWN_SERVICES: &[KnownService] = &[
    KnownService {
        name: ADJUST_LIGHT,
        service_type: "roller_eye/adjust_ligth",
        capability: "IR light brightness/mode",
    },
    KnownService {
        name: NIGHT_STATUS,
        service_type: "roller_eye/night_get",
        capability: "night mode and IR brightness",
    },
    KnownService {
        name: VIDEO_RESOLUTION,
        service_type: "roller_eye/video_set_resolution",
        capability: "camera resolution",
    },
    KnownService {
        name: NAV_PATROL,
        service_type: "roller_eye/nav_patrol",
        capability: "patrol or return to dock",
    },
    KnownService {
        name: NAV_PATROL_STOP,
        service_type: "roller_eye/nav_patrol_stop",
        capability: "stop patrol",
    },
    KnownService {
        name: NAV_STATUS,
        service_type: "roller_eye/nav_get_status",
        capability: "navigation state",
    },
    KnownService {
        name: NAV_LIST_PATHS,
        service_type: "roller_eye/nav_list_path",
        capability: "saved patrol paths",
    },
    KnownService {
        name: IMU_CALIBRATE,
        service_type: "roller_eye/imu_calib",
        capability: "IMU calibration",
    },
    KnownService {
        name: ALGO_ACTION,
        service_type: "roller_eye/algo_action",
        capability: "timed velocity action",
    },
    KnownService {
        name: ALGO_MOVE,
        service_type: "roller_eye/algo_move",
        capability: "distance move",
    },
    KnownService {
        name: ALGO_ROLL,
        service_type: "roller_eye/algo_roll",
        capability: "angle rotation",
    },
    KnownService {
        name: RECORD_START,
        service_type: "roller_eye/record_start",
        capability: "onboard media recording",
    },
    KnownService {
        name: RECORD_STOP,
        service_type: "roller_eye/record_stop",
        capability: "stop onboard recording",
    },
];

pub fn known_service(name: &str) -> Option<&'static KnownService> {
    KNOWN_SERVICES.iter().find(|service| service.name == name)
}
