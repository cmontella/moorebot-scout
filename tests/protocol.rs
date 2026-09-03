use moorebot_scout::{
    frame::{AudioFormat, ScoutFrame, StreamType},
    motion::{MotionLimits, Velocity},
    sensors::{BatteryState, BatteryStatus, IlluminanceSample, ImuSample, RangeSample},
    DecodeError,
};

fn u32_field(output: &mut Vec<u8>, value: u32) {
    output.extend_from_slice(&value.to_le_bytes());
}

fn i32_field(output: &mut Vec<u8>, value: i32) {
    output.extend_from_slice(&value.to_le_bytes());
}

fn u64_field(output: &mut Vec<u8>, value: u64) {
    output.extend_from_slice(&value.to_le_bytes());
}

fn f32_field(output: &mut Vec<u8>, value: f32) {
    output.extend_from_slice(&value.to_le_bytes());
}

fn f64_field(output: &mut Vec<u8>, value: f64) {
    output.extend_from_slice(&value.to_le_bytes());
}

fn string_field(output: &mut Vec<u8>, value: &str) {
    u32_field(output, value.len() as u32);
    output.extend_from_slice(value.as_bytes());
}

fn header(output: &mut Vec<u8>, sequence: u32, frame_id: &str) {
    u32_field(output, sequence);
    u32_field(output, 12);
    u32_field(output, 34);
    string_field(output, frame_id);
}

fn frame_body(stream_type: i8, parameters: [i32; 4], payload: &[u8]) -> Vec<u8> {
    let mut output = Vec::new();
    u32_field(&mut output, 7);
    u64_field(&mut output, 123_456);
    u32_field(&mut output, 3_001);
    output.push(stream_type as u8);
    u32_field(&mut output, 6);
    for value in parameters {
        i32_field(&mut output, value);
    }
    u32_field(&mut output, payload.len() as u32);
    output.extend_from_slice(payload);
    output
}

#[test]
fn decodes_scout_frame_at_the_documented_offsets() {
    let jpeg = [0xff, 0xd8, 1, 2, 3, 0xff, 0xd9];
    let decoded = ScoutFrame::decode_ros(&frame_body(1, [1920, 1080, 0, 9], &jpeg)).unwrap();

    assert_eq!(decoded.sequence, 7);
    assert_eq!(decoded.timestamp_ms, 123_456);
    assert_eq!(decoded.session, 3_001);
    assert_eq!(decoded.stream_type, StreamType::Jpeg);
    assert_eq!(decoded.original_sequence, 6);
    assert_eq!(decoded.parameters, [1920, 1080, 0, 9]);
    assert_eq!(decoded.video_dimensions(), Some((1920, 1080)));
    assert!(decoded.has_jpeg_markers());
}

#[test]
fn interprets_h264_and_aac_parameters() {
    let h264 = ScoutFrame::decode_ros(&frame_body(0, [1280, 720, 1, 0], &[1])).unwrap();
    assert_eq!(h264.is_keyframe(), Some(true));

    let aac = ScoutFrame::decode_ros(&frame_body(2, [48_000, 16, 2, 0], &[1])).unwrap();
    assert_eq!(
        aac.audio_format(),
        Some(AudioFormat {
            sample_rate_hz: 48_000,
            bit_width: 16,
            channels: 2,
        })
    );
}

#[test]
fn rejects_aac_parameters_that_do_not_fit_the_public_format() {
    let oversized_bit_width =
        ScoutFrame::decode_ros(&frame_body(2, [48_000, 65_536, 2, 0], &[1])).unwrap();
    assert_eq!(oversized_bit_width.audio_format(), None);

    let oversized_channels =
        ScoutFrame::decode_ros(&frame_body(2, [48_000, 16, 65_537, 0], &[1])).unwrap();
    assert_eq!(oversized_channels.audio_format(), None);

    let zero_sample_rate = ScoutFrame::decode_ros(&frame_body(2, [0, 16, 2, 0], &[1])).unwrap();
    assert_eq!(zero_sample_rate.audio_format(), None);
}

#[test]
fn rejects_truncated_and_overlong_frames() {
    let mut truncated = frame_body(1, [1, 1, 0, 0], &[1, 2, 3]);
    truncated.pop();
    assert!(matches!(
        ScoutFrame::decode_ros(&truncated),
        Err(DecodeError::UnexpectedEnd { .. })
    ));

    let mut overlong = frame_body(1, [1, 1, 0, 0], &[1, 2, 3]);
    overlong.push(4);
    assert_eq!(
        ScoutFrame::decode_ros(&overlong),
        Err(DecodeError::TrailingBytes(1))
    );
}

#[test]
fn maps_standard_axes_to_scout_axes_and_clamps() {
    let twist = Velocity {
        forward_mps: 2.0,
        lateral_mps: -0.5,
        yaw_rps: 4.0,
    }
    .to_scout_twist(MotionLimits::default())
    .unwrap();

    assert_eq!(twist.linear_x, -0.2);
    assert_eq!(twist.linear_y, 0.47);
    assert_eq!(twist.angular_z, 2.9);
    assert_eq!(twist.encode_ros().len(), 48);
}

#[test]
fn rejects_non_finite_motion() {
    let error = Velocity {
        forward_mps: f64::NAN,
        ..Velocity::default()
    }
    .to_scout_twist(MotionLimits::default())
    .unwrap_err();
    assert_eq!(error.to_string(), "forward velocity must be finite");
}

#[test]
fn decodes_range_and_light_messages() {
    let mut range = Vec::new();
    header(&mut range, 4, "tof_link");
    range.push(1);
    f32_field(&mut range, 0.4);
    f32_field(&mut range, 0.02);
    f32_field(&mut range, 2.0);
    f32_field(&mut range, 0.375);
    let range = RangeSample::decode_ros(&range).unwrap();
    assert_eq!(range.header.frame_id, "tof_link");
    assert_eq!(range.radiation_type, 1);
    assert_eq!(range.range_m, 0.375);

    let mut light = Vec::new();
    header(&mut light, 5, "");
    f64_field(&mut light, f64::from((123_u32 << 16) | 456));
    f64_field(&mut light, 0.0);
    let light = IlluminanceSample::decode_ros(&light).unwrap();
    assert_eq!(light.packed_channels(), Some((123, 456)));
}

#[test]
fn decodes_scout_battery_status_vector() {
    let mut body = Vec::new();
    u32_field(&mut body, 3);
    i32_field(&mut body, 0);
    i32_field(&mut body, 87);
    i32_field(&mut body, 1);

    let battery = BatteryStatus::decode_ros(&body).unwrap();
    assert_eq!(battery.state, BatteryState::Charging);
    assert_eq!(battery.percentage, 87);
    assert!(battery.externally_powered);
}

#[test]
fn rejects_battery_length_larger_than_the_message() {
    let mut body = Vec::new();
    u32_field(&mut body, u32::MAX);
    assert!(matches!(
        BatteryStatus::decode_ros(&body),
        Err(DecodeError::UnexpectedEnd { .. })
    ));
}

#[test]
fn decodes_complete_imu_message() {
    let mut body = Vec::new();
    header(&mut body, 9, "imu_link");
    for value in [0.0, 0.0, 0.0, 1.0] {
        f64_field(&mut body, value);
    }
    for value in 0..9 {
        f64_field(&mut body, value as f64);
    }
    for value in [0.1, 0.2, 0.3] {
        f64_field(&mut body, value);
    }
    for value in 10..19 {
        f64_field(&mut body, value as f64);
    }
    for value in [1.1, 1.2, 9.8] {
        f64_field(&mut body, value);
    }
    for value in 20..29 {
        f64_field(&mut body, value as f64);
    }

    let imu = ImuSample::decode_ros(&body).unwrap();
    assert_eq!(imu.header.frame_id, "imu_link");
    assert_eq!(imu.orientation.w, 1.0);
    assert_eq!(imu.angular_velocity.z, 0.3);
    assert_eq!(imu.linear_acceleration.z, 9.8);
    assert_eq!(imu.linear_acceleration_covariance[8], 28.0);
}
