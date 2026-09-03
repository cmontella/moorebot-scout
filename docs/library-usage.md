# Using `moorebot-scout` from Rust

The command-line program is the easiest starting point. This page is for a
student who wants to use the protocol types in another Rust project.

The crate is not on crates.io yet. Until the first release, add the Git
repository to your project's `Cargo.toml`. Disable default features when you
only need protocol parsing and do not need live ROS transport:

```toml
[dependencies]
moorebot-scout = { git = "https://github.com/cmontella/moorebot-scout", default-features = false }
```

After issue #7 is complete and a release exists, this will be replaced by a
normal crates.io version requirement.

## Example: map a safe motion request

The public `Velocity` type uses the meanings students normally expect:
positive forward velocity goes forward, positive lateral velocity goes left,
and positive yaw turns counter-clockwise. The conversion applies limits and
maps those values to the Scout's unusual axes.

```rust
use moorebot_scout::motion::{MotionLimits, Velocity};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let requested = Velocity {
        forward_mps: 0.10,
        lateral_mps: 0.0,
        yaw_rps: 0.0,
    };

    let scout = requested.to_scout_twist(MotionLimits::default())?;
    println!("Scout linear.x: {}", scout.linear_x);
    println!("Scout linear.y: {}", scout.linear_y);
    println!("Scout angular.z: {}", scout.angular_z);
    Ok(())
}
```

The repository contains this as a build-checked example:

```text
cargo run --example motion_mapping --no-default-features
```

Converting a velocity does not contact a robot. Publishing motion is a separate
operation and should preserve the CLI's connection checks, deadline, Ctrl-C
handling, and final zero-velocity command.

## Example: inspect known interfaces

`KNOWN_TOPICS` and `KNOWN_SERVICES` distinguish interfaces implemented by the
driver from interfaces that have only been found in source code:

```rust
use moorebot_scout::{services::KNOWN_SERVICES, topics::KNOWN_TOPICS};

fn main() {
    for topic in KNOWN_TOPICS {
        println!("{}: {} ({:?})", topic.name, topic.capability, topic.support);
    }
    for service in KNOWN_SERVICES {
        println!("{}: {}", service.name, service.capability);
    }
}
```

Run the checked version with:

```text
cargo run --example list_known_interfaces --no-default-features
```

## Example: decode a Scout media message

`ScoutFrame::decode_ros` expects the complete ROS-serialized
`roller_eye/frame` message body—not a standalone JPEG file:

```rust
use moorebot_scout::frame::{ScoutFrame, StreamType};

fn inspect_message(message_body: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
    let frame = ScoutFrame::decode_ros(message_body)?;

    match frame.stream_type {
        StreamType::Jpeg => println!("JPEG dimensions: {:?}", frame.video_dimensions()),
        StreamType::H264 => println!("H.264 keyframe: {:?}", frame.is_keyframe()),
        StreamType::Aac => println!("AAC format: {:?}", frame.audio_format()),
        StreamType::Unknown(value) => println!("unknown stream type: {value}"),
    }
    Ok(())
}
```

Malformed, truncated, overlong, and oversized messages return `DecodeError`.
Media payloads larger than `MAX_FRAME_DATA_BYTES` are rejected before the
decoder copies them.

## Feature selection

| Dependency form | What it builds |
|---|---|
| Default features | Protocol modules plus the ROS 1 transport and CLI |
| `default-features = false` | Protocol modules only; no live ROS transport |

The protocol-only configuration is useful for offline analysis, recorded
fixtures, teaching serialization, and programs that provide their own
transport. Both configurations are tested on Linux, macOS, and Windows.

## API stability

Version `0.1.0` is an initial hardware-unvalidated API. Expect types and method
names to change as real Scout captures reveal firmware differences. Pin a Git
revision for classroom assignments that must remain reproducible.
