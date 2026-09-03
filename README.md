# moorebot-scout

[![CI](https://github.com/cmontella/moorebot-scout/actions/workflows/ci.yml/badge.svg)](https://github.com/cmontella/moorebot-scout/actions/workflows/ci.yml)

An independent Rust driver and protocol library for the Moorebot Scout. It
connects directly to the ROS 1 master already running on the robot, so building
the crate does **not** require a local ROS installation.

> Status: the wire formats and control mapping are covered by offline tests,
> but this version has not yet been exercised against a physical Scout. Put the
> robot on blocks for the first motion test and keep a hand on its power button.

## Start here

New to Rust, ROS, or networked robots? Follow the
[student getting-started guide](docs/getting-started.md). It explains every
prerequisite, how to find the correct network address on Windows, macOS, and
Linux, what output to expect, and how to perform a cautious first hardware test.

If you want to use the crate from another Rust program, see the
[library examples](docs/library-usage.md).

## What works in this first slice

- Enumerate the live ROS topics and services, with annotations for known Scout
  interfaces.
- Publish bounded motion commands to `/cmd_vel`, including the Scout's unusual
  linear-axis mapping and a zero-velocity command on normal, error, or Ctrl-C
  exit.
- Decode the Scout's 6-axis IMU, time-of-flight range sensor, ambient-light
  sensor, and custom battery status.
- Decode the custom `roller_eye/frame` media message correctly.
- Republish `/CoreNode/jpg` as standard `sensor_msgs/CompressedImage`, which
  makes the color camera usable by normal ROS image tools.
- Build the protocol-only library with no ROS transport dependencies.

The H.264 camera, AAC microphone, object/motion detection, iBeacon, odometry,
IR light controls, autonomous navigation, docking, and onboard recording are
mapped but deliberately not presented as working APIs until hardware testing.

## Build

```sh
cd moorebot-scout
cargo build --release
cargo test --all-targets
```

CI verifies the default ROS 1 build and the protocol-only build on Linux,
macOS, and Windows using both Rust 1.98 and the current stable toolchain.

To build only the protocol library:

```sh
cargo build --no-default-features
```

Two examples work without a robot or ROS installation:

```sh
cargo run --example motion_mapping --no-default-features
cargo run --example list_known_interfaces --no-default-features
```

The CLI is available at `target/release/moorebot-scout` (`.exe` on Windows), or
can be run through Cargo as shown below.

## Connect to a Scout

1. Connect the computer to the same network as the Scout.
2. Determine the computer address that the Scout can reach. In the Scout's
   direct-connect mode this is usually a `10.42.0.x` address.
3. If the robot advertises ROS nodes as `linaro-alip`, make that hostname resolve
   to the robot's address (commonly `10.42.0.1`).
4. Pass the computer's reachable address with `--advertise-address`. Do not use
   `127.0.0.1`; ROS 1 peers need to connect back to this process.

List everything the firmware currently exposes:

```sh
cargo run --release -- \
  --master http://10.42.0.1:11311 \
  --advertise-address 10.42.0.124 \
  discover
```

The address values are examples; use the addresses assigned to your robot and
computer.

### Monitor the undocumented sensors

```sh
cargo run --release -- \
  --master http://10.42.0.1:11311 \
  --advertise-address 10.42.0.124 \
  monitor --seconds 30
```

This subscribes to:

- `/SensorNode/imu` (`sensor_msgs/Imu`)
- `/SensorNode/tof` (`sensor_msgs/Range`)
- `/SensorNode/light` (`sensor_msgs/Illuminance`)
- `/SensorNode/simple_battery_status` (`roller_eye/status`)

The Scout starts several sensor publishers only when a subscriber connects, so
their absence from an idle topic stream does not necessarily mean the hardware
is disabled.

### Bridge the color camera

```sh
cargo run --release -- \
  --master http://10.42.0.1:11311 \
  --advertise-address 10.42.0.124 \
  camera-bridge
```

The bridge publishes standard compressed images on
`/moorebot_scout/camera/image/compressed`. A ROS 1 image viewer can subscribe to
that topic without knowing about `roller_eye/frame`. Because the output type is
standard, it is also a cleaner boundary for a later ROS 1-to-ROS 2 bridge.

### Send a short motion command

This example asks for 0.1 m/s forward motion for 500 ms, then sends a stop:

```sh
cargo run --release -- \
  --master http://10.42.0.1:11311 \
  --advertise-address 10.42.0.124 \
  drive --forward 0.1 --duration-ms 500
```

The public API uses standard mobile-base semantics, while the Scout firmware
swaps the two linear axes:

| Meaning | Driver input | Scout `/cmd_vel` |
|---|---:|---:|
| Forward/backward | `forward_mps` | `linear.y` |
| Left/right strafe | `lateral_mps` | `linear.x` |
| Counter-clockwise rotation | `yaw_rps` | `angular.z` |

Commands are clamped to 0.47 m/s forward, 0.2 m/s lateral, and 2.9 rad/s yaw.
The first-party motor source defines approximately 0.47 m/s as its linear
ceiling; the lower lateral and yaw values follow the supplied controller. A
command is refused if no `/cmd_vel` subscriber connects within three seconds.
This initial CLI also limits a single command to 60 seconds and its update rate
to 1–100 Hz.

## Architecture

The crate deliberately separates protocol work from transport:

```text
Rust application
  ├─ motion + media + sensor codecs (no ROS installation required)
  └─ ROS 1 transport (`rosrust`, raw wire messages)
          ↕ TCPROS/XML-RPC over the local network
      ROS master and nodes running on the Scout
```

Using raw ROS messages avoids build-time dependence on an obsolete ROS Melodic
installation while retaining normal ROS 1 interoperability.

## Evidence and limitations

The initial implementation was derived independently from:

- the supplied Python/MATLAB archive, especially its `/cmd_vel` publisher,
  `/CoreNode/jpg` subscriber, and `frame.msg`;
- Moorebot/Pilot Labs' [first-party Scout source
  release](https://github.com/Pilot-Labs-Dev/Scout-open-source), which exposes
  additional sensor topics and internal service definitions; and
- the pure-Rust [rosrust](https://github.com/adnanademovic/rosrust) ROS 1 client.

The live transport pins the exact revision from [upstream `rosrust` PR
#221](https://github.com/adnanademovic/rosrust/pull/221), which rejects oversized
TCPROS bodies and connection headers before allocating them. Publishing to
crates.io is disabled in `Cargo.toml` until that fix is available from a
published dependency; the remaining work is tracked in [issue
#9](https://github.com/cmontella/moorebot-scout/issues/9).

No first-party source was copied into this crate. See
[`docs/protocol.md`](docs/protocol.md) for the decoded layout, discovered feature
map, security/resource-limit audit, and hardware-validation checklist.

This crate is not affiliated with or endorsed by Moorebot or Pilot Labs.

## Security

ROS 1 peers are unauthenticated. Use the driver only with a trusted Scout and
ROS master on an isolated robot network. See the [security policy and threat
model](SECURITY.md) for input limits, known `rosrust` transport and dependency
risks, and private reporting instructions.

## Roadmap

1. [Capture a `discover` report and sample messages from a real
   Scout](https://github.com/cmontella/moorebot-scout/issues/1).
2. [Validate motion, camera, sensor units, and firmware
   behavior](https://github.com/cmontella/moorebot-scout/issues/2).
3. [Add recorded-message fixtures and hardware-gated integration
   tests](https://github.com/cmontella/moorebot-scout/issues/3).
4. [Implement typed clients for Scout control and navigation
   services](https://github.com/cmontella/moorebot-scout/issues/4).
5. [Add H.264, AAC, and detection stream
   support](https://github.com/cmontella/moorebot-scout/issues/5).
6. [Add a ROS 2 integration
   path](https://github.com/cmontella/moorebot-scout/issues/6).
7. [Prepare the crate for its first crates.io
   release](https://github.com/cmontella/moorebot-scout/issues/7).

## License

Apache License 2.0. See [`LICENSE`](LICENSE).
