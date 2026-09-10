# moorebot-scout

[![CI](https://github.com/cmontella/moorebot-scout/actions/workflows/ci.yml/badge.svg)](https://github.com/cmontella/moorebot-scout/actions/workflows/ci.yml)

An independent Rust driver and protocol library for the Moorebot Scout. It
connects directly to the ROS 1 master already running on the robot, so building
the crate does **not** require a local ROS installation.

> Discovery, sensor monitoring, camera capture, and interactive motion have been
> tested against a physical Scout. As with any mobile robot, use a clear testing
> area and keep the power button within reach.

## Start here

New to Rust, ROS, or networked robots? Follow the
[student getting-started guide](docs/getting-started.md). It explains every
prerequisite, how to connect directly or through a home router on Windows,
macOS, and Linux, what output to expect, and how to drive safely.

For normal use, download the package for your computer from [GitHub
Releases](https://github.com/cmontella/moorebot-scout/releases). Each package
contains one executable—no Rust, ROS, Python, or MATLAB installation is needed.
After extracting it, connect to the Scout network and run the executable:

```text
./moorebot-scout
```

On Windows, use `./moorebot-scout.exe` in PowerShell. With no command the
program finds the Scout and starts keyboard control: W/S drives, A/D strafes,
Q/E turns, Up/Down changes speed and update rate, Space saves the latest camera
image to the Desktop, and Escape stops and exits. Release binaries are currently
unsigned; read the safety and download notes in the student guide before
running one.

If you want to use the crate from another Rust program, see the
[library examples](docs/library-usage.md).

## What works today

- Enumerate the live ROS topics and services, with annotations for known Scout
  interfaces.
- Publish bounded motion commands to `/cmd_vel`, including the Scout's unusual
  linear-axis mapping and a zero-velocity command on normal, error, or Ctrl-C
  exit.
- Drive interactively with WASD and save the latest valid camera JPEG with
  Space. Key-release handling and input deadman timers stop stale movement.
- Decode the Scout's 6-axis IMU, time-of-flight range sensor, ambient-light
  sensor, and custom battery status.
- Decode the custom `roller_eye/frame` media message correctly.
- Republish `/CoreNode/jpg` as standard `sensor_msgs/CompressedImage`, which
  makes the color camera usable by normal ROS image tools.
- Build the protocol-only library with no ROS transport dependencies.

The H.264 camera, AAC microphone, object/motion detection, iBeacon, odometry,
IR light controls, autonomous navigation, docking, and onboard recording are
mapped but are not yet exposed as driver commands.

## Build from source

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

1. Connect the computer to the Scout's own Wi-Fi, or put the Scout and computer
   on the same trusted home network using the [home-network
   guide](docs/home-network.md).
2. Run the executable. It confirms that the ROS master is reachable, asks the
   operating system which local address routes to it, and handles the Scout's
   internal `linaro-alip` hostname inside the process. No hosts-file edit is
   needed.
3. When possible, it matches the current Wi-Fi or ARP MAC address to the
   classroom robot list and prints the robot's name. Identity detection is
   informational and does not affect connectivity or authorize motion.

List everything the firmware currently exposes:

```sh
moorebot-scout discover
```

The default ROS master is `http://10.42.0.1:11311`, which is the normal address
in Wi-Fi Direct mode. A home router assigns the Scout a different address; pass
it with `--master`, for example:

```sh
moorebot-scout --master http://192.168.1.73:11311
```

If automatic route selection cannot work in an unusual network setup,
`--advertise-address` remains available as a manual troubleshooting override.

### Monitor the undocumented sensors

```sh
moorebot-scout monitor
```

This subscribes to:

- `/SensorNode/imu` (`sensor_msgs/Imu`)
- `/SensorNode/tof` (`sensor_msgs/Range`)
- `/SensorNode/light` (`sensor_msgs/Illuminance`)
- `/SensorNode/simple_battery_status` (`roller_eye/status`)

The Scout starts several sensor publishers only when a subscriber connects, so
their absence from an idle topic stream does not necessarily mean the hardware
is disabled. Monitoring continues until Ctrl-C by default; use
`monitor --seconds 30` for a fixed-duration sample.

### Bridge the color camera

```sh
moorebot-scout camera-bridge
```

The bridge publishes standard compressed images on
`/moorebot_scout/camera/image/compressed`. A ROS 1 image viewer can subscribe to
that topic without knowing about `roller_eye/frame`. Because the output type is
standard, it is also a cleaner boundary for a later ROS 1-to-ROS 2 bridge.

### Send a short motion command

This example asks for 0.1 m/s forward motion for 500 ms, then sends a stop:

```sh
moorebot-scout drive --forward 0.1 --duration-ms 500
```

`drive` is a one-shot timed command and requires at least one of `--forward`,
`--lateral`, or `--yaw`. Components can be combined for diagonal motion while
turning.

The public API uses standard mobile-base semantics, while the Scout firmware
swaps the two linear axes:

| Meaning | Driver input | Scout `/cmd_vel` |
|---|---:|---:|
| Forward/backward | `forward_mps` | `linear.y` |
| Left/right strafe | `lateral_mps` | negated `linear.x` (Scout X points right) |
| Counter-clockwise rotation | `yaw_rps` | `angular.z` |

Commands are clamped to 0.47 m/s forward, 0.2 m/s lateral, and 2.9 rad/s yaw.
The first-party motor source defines approximately 0.47 m/s as its linear
ceiling; the lower lateral and yaw values follow the supplied controller. A
command is refused if no `/cmd_vel` subscriber connects within three seconds.
This initial CLI also limits a single command to 60 seconds and its update rate
to 1–100 Hz.

### Drive with the keyboard and capture pictures

First put the Scout on a stable stand with every wheel clear, then run:

```sh
moorebot-scout
```

The explicit `moorebot-scout teleop` command behaves the same way and accepts
the advanced options below.

| Key | Action |
|---|---|
| W / S | forward / backward |
| A / D | strafe left / right |
| Q / E | turn left / right |
| Up / Down | increase / decrease speed and control rate |
| Space | save the latest JPEG to the Desktop |
| Esc or Ctrl-C | stop and exit |

The Scout motor controller mixes forward, lateral, and yaw commands across all
four Mecanum wheels. Teleop starts at 40 Hz. Up/Down changes both velocity and
the sampling/publication rate in 25% steps, within the Scout's velocity limits
and a 100 Hz maximum; the new values are printed after each adjustment.

The default turn rate is 2.0 rad/s. Lower yaw values can fall below the physical
motor driver's minimum reliable duty threshold, which can make only one wheel
polarity appear to start even though the firmware's pure-yaw mix addresses all
four wheels.

The terminal must remain focused. A key-event deadman stops stale motion, and a
zero-velocity command is sent on normal exit and input/publisher errors. Space
stops motion before writing the newest valid `/CoreNode/jpg` frame. Use
`teleop --picture-directory PATH` (or `--screenshot-dir PATH`) to override the
Desktop, and `teleop --help` for every speed and control option.

## Architecture

The crate deliberately separates protocol work from transport:

```text
Rust application
  ├─ keyboard control + Desktop picture saving
  ├─ motion + media + sensor codecs (no ROS installation required)
  └─ ROS 1 transport (`rosrust`, bounded raw wire messages)
          ├─ selects the computer's route automatically
          └─ maps the Scout-only `linaro-alip` name internally
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

The live transport pins an exact `rosrust` fork revision. Its allocation limits
were submitted in [upstream `rosrust` PR
#221](https://github.com/adnanademovic/rosrust/pull/221); the revision also adds
the narrow peer-hostname alias needed for Scout firmware. Publishing to
crates.io is disabled in `Cargo.toml` until the transport fixes are available
from a published dependency; the remaining work is tracked in [issue
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
