# Moorebot Scout protocol notes

These notes separate observed or source-backed facts from behavior that still
needs validation on a physical robot.

## Source baseline

- Supplied archive snapshot: `docs-main.zip` (archive comment/revision
  `b31922ec429ec7fb941aa55e995e019c252bd7d6`).
- First-party source inspected at revision
  [`c599c1a`](https://github.com/Pilot-Labs-Dev/Scout-open-source/commit/c599c1a906d6919640cf11b2d6adb30d08b100d2).
- Supplemental controller inspected at revision
  [`b086e3b`](https://github.com/GINNOV/Scout-Controller/commit/b086e3bf8cb00abe3f0c494f17a90a828d051599).

The supplied documents are evidence, not executable setup instructions. In
particular, this project does not repeat the archive's root-access or system
modification steps.

## `roller_eye/frame` wire layout

ROS 1 serializes every primitive little-endian and prefixes a variable-length
array with a 32-bit element count. The complete fixed portion is 41 bytes:

| Offset | Size | Field | Meaning |
|---:|---:|---|---|
| 0 | 4 | `seq` | stream sequence |
| 4 | 8 | `stamp` | robot monotonic milliseconds |
| 12 | 4 | `session` | stream session identifier |
| 16 | 1 | `type` | 0 H.264, 1 JPEG, 2 AAC |
| 17 | 4 | `oseq` | source-frame sequence |
| 21 | 4 | `par1` | width or audio sample rate |
| 25 | 4 | `par2` | height or audio bit width |
| 29 | 4 | `par3` | H.264 keyframe flag or audio channels |
| 33 | 4 | `par4` | reserved extension |
| 37 | 4 | `data.length` | payload byte count |
| 41 | N | `data` | encoded media payload |

The supplied Python decoder reads `seq` and `stamp`, skips four bytes as a
purported `frame_id`, and then decodes the remaining metadata. There is no
`frame_id` in the message definition. The skip happens to preserve the correct
payload offset because it consumes `session`, while subsequent reads consume
the same total number of bytes. JPEG display therefore works even though
`session`, `type`, `oseq`, and all four parameters are misdecoded. The Rust
decoder follows the actual message definition and tests every offset.

For H.264, first-party code sets `par1=width`, `par2=height`, and
`par3=is_keyframe`. For AAC it documents `par1=sample_rate`, `par2=bit_width`,
and `par3=channels`. The timestamp is constructed from `CLOCK_MONOTONIC` and is
not a Unix timestamp; the camera bridge uses ROS arrival time in its standard
message header.

## Source-backed topic inventory

| Topic | Message | Capability | Initial driver status |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | holonomic motion | publish |
| `/CoreNode/jpg` | `roller_eye/frame` | color JPEG camera | decode + bridge |
| `/CoreNode/h264` | `roller_eye/frame` | H.264 camera | mapped |
| `/CoreNode/aac` | `roller_eye/frame` | microphone audio | mapped |
| `/CoreNode/grey_img` | `sensor_msgs/Image` | greyscale image | mapped |
| `/CoreNode/obj` | `roller_eye/detect` | object detection | mapped |
| `/CoreNode/motion` | `roller_eye/detect` | motion detection | mapped |
| `/SensorNode/imu` | `sensor_msgs/Imu` | 6-axis IMU | decode/monitor |
| `/SensorNode/tof` | `sensor_msgs/Range` | VL53L0X ToF range | decode/monitor |
| `/SensorNode/light` | `sensor_msgs/Illuminance` | ambient light | decode/monitor |
| `/SensorNode/ibeacon` | `sensor_msgs/Range` | charger beacon estimate | mapped |
| `/SensorNode/simple_battery_status` | `roller_eye/status` | charge state, percentage, external power | decode/monitor |
| `/baselink_odom_relative` | `nav_msgs/Odometry` | wheel odometry | mapped |
| `/vio_odom_relative` | `nav_msgs/Odometry` | visual-inertial odometry | mapped |

The source contains a magnetometer implementation, but its publisher is
commented out. It should be treated as hardware/firmware dependent rather than
an available sensor.

Battery status is an `int32[]` with at least three elements:

1. state: `0=charging`, `1=discharging`, `2=full`, other=unknown;
2. estimated percentage from 0 through 100; and
3. external power flag.

The first-party README describes the light reading as a packed 32-bit quantity:
CH0 occupies the upper 16 bits and CH1 the lower 16 bits. The driver preserves
the ROS `illuminance` value and offers this split only when it is finite,
non-negative, and integral.

## High-interest service inventory

These services exist in the source tree but are intentionally discovery-only
until a real robot confirms their registered type, MD5, accepted values, and
side effects.

| Service | Source type | Intended behavior |
|---|---|---|
| `/CoreNode/adjust_light` | `roller_eye/adjust_ligth` | IR brightness/mode; source uses 0 down, 1 up, 3 max, 4 auto |
| `/CoreNode/night_get` | `roller_eye/night_get` | night state and IR brightness |
| `/CoreNode/video_set_resolution` | `roller_eye/video_set_resolution` | camera dimensions |
| `/NavPathNode/nav_patrol` | `roller_eye/nav_patrol` | saved patrol; an empty path name enters return-to-dock logic |
| `/NavPathNode/nav_patrol_stop` | `roller_eye/nav_patrol_stop` | stop patrol |
| `/NavPathNode/nav_get_status` | `roller_eye/nav_get_status` | navigation state |
| `/NavPathNode/nav_list_path` | `roller_eye/nav_list_path` | saved paths |
| `/imu_calib` | `roller_eye/imu_calib` | IMU calibration |
| `/UtilNode/algo_action` | `roller_eye/algo_action` | timed velocity |
| `/UtilNode/algo_move` | `roller_eye/algo_move` | distance move |
| `/UtilNode/algo_roll` | `roller_eye/algo_roll` | angle rotation |
| `/RecorderAgentNode/record_start` | `roller_eye/record_start` | onboard recording |
| `/RecorderAgentNode/record_stop` | `roller_eye/record_stop` | stop recording |

Notably, the supplemental Python `go home` experiment sends the literal name
`"home"`. First-party navigation code uses an **empty** `name` to trigger its
return-to-dock branch, so the experiment is not sufficient evidence of a
working command.

## Hardware validation checklist

Save the results by firmware version; Moorebot may have shipped incompatible
graphs.

- Record `discover` output before and while each feature is subscribed.
- Confirm the Scout can resolve and connect to the driver's advertised address.
- Capture one raw message from every source topic as a future test fixture.
- Verify forward, lateral, and yaw signs at low speed with wheels clear.
- Measure the true velocity scale rather than assuming `/cmd_vel` units are
  calibrated meters/radians per second.
- Compare JPEG metadata dimensions to the decoded image dimensions.
- Confirm ToF units, limits, invalid/no-return values, and physical sensor
  direction.
- Establish IMU frame orientation, covariance validity, rate, and calibration.
- Compare packed light channels against environmental changes and IR-light mode.
- Observe battery states while charging, full, and unplugged.
- Query service headers before enabling any service client; test non-motion
  services first.
- Verify that a stop command reaches the motor node on disconnect and process
  termination.
