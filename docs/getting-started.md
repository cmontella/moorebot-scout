# Student getting-started guide

This guide assumes you are new to both Rust and ROS. By the end, you will have
built and tested the driver, connected it to a Moorebot Scout, inspected the
robot's interfaces, and optionally read sensors, viewed the camera, or sent one
short motion command.

## What is the Moorebot Scout?

The Scout is a small mobile monitoring robot built on Linux and ROS. It has four
Mecanum wheels, a 1080p camera with infrared night vision, a microphone and
speaker, an IMU motion sensor, an ambient-light sensor, a forward
time-of-flight distance sensor, and a charging dock. Its normal phone app can
drive it, show video, and run saved patrols. Moorebot presents it as both a
home-monitoring device and an educational/open-source robotics platform; see
the [official product overview](https://www.moorebot.com/products/moorebot-scout)
and [official open-source control
repository](https://github.com/Pilot-Labs-Dev/Scout-open-source).

This project replaces the difficult Python/MATLAB computer setup with one Rust
program. The first version can inspect the robot, read several sensors, bridge
the color camera into a standard ROS message, and send short bounded movement
commands. Other capabilities found in the Scout source—such as night mode,
patrols, docking, recording, audio, and onboard detections—are listed for future
work but are not yet exposed as working commands.

The driver talks to ROS 1 already running on the Scout. You do **not** need to
install ROS, Python, MATLAB, or custom Moorebot message packages to build it or
use its command-line tools. A separate ROS installation is useful only if you
want graphical tools such as `rqt_image_view`.

## How the computer and Scout connect

```mermaid
flowchart LR
    subgraph Computer[Student computer: Windows, macOS, or Linux]
        CLI[Rust driver CLI]
        Viewer[Optional ROS image viewer]
        SSH[Optional SSH client]
    end

    WiFi[Scout Wi-Fi or trusted local network]

    subgraph Robot[Moorebot Scout at 10.42.0.1]
        Master[ROS master on port 11311]
        Motors[Motor node: /cmd_vel]
        Sensors[IMU, ToF, light, battery nodes]
        Camera[Camera node: /CoreNode/jpg]
        Shell[Linux SSH server on port 22]
    end

    CLI -->|XML-RPC discovery| Master
    CLI -->|bounded velocity messages| Motors
    Sensors -->|TCPROS sensor messages| CLI
    Camera -->|Scout JPEG messages| CLI
    CLI -->|standard CompressedImage| Viewer
    SSH -.->|optional diagnostics only| Shell
    Computer --- WiFi --- Robot
```

The Rust driver does **not** run commands through SSH. It first asks the ROS
master where topics live, and then ROS nodes open direct TCPROS connections to
exchange messages. That is why the command needs both the robot's master
address and your computer's advertised address. SSH is a separate, optional
way to open a Linux terminal on the robot for diagnostics.

## Safety first

The driver has offline tests, but its motion directions and sensor units have
not yet been validated on a physical Scout.

- Do discovery and sensor exercises before motion.
- For the first motion test, place the Scout on a stable stand with every wheel
  clear of the table or floor.
- Keep hands, hair, cables, and loose clothing away from the wheels.
- Keep one hand ready at the robot's physical power control.
- Use the low speed and short duration shown in this guide.
- Do not test near stairs, table edges, people, or animals.

You do not need root access to the Scout, and this guide does not ask you to
change software on the robot.

## Four terms you need

| Term | Plain-language meaning |
|---|---|
| ROS master | The directory service on the Scout. Its usual address is `http://10.42.0.1:11311`. |
| Node | One program participating in ROS, such as the Scout motor controller or this driver. |
| Topic | A named stream of messages. `/cmd_vel` carries movement commands. |
| Advertised address | Your computer's address on the Scout network. The robot uses it to connect back to the driver. |

The robot address and computer address are different. In direct-connect mode,
the robot is commonly `10.42.0.1`; your computer will normally have another
`10.42.0.x` address. Never advertise `127.0.0.1`, because that means "this same
machine" to whichever computer reads it.

## 1. Install the development tools

Install these tools on your computer:

1. [Git](https://git-scm.com/downloads), used to download the project.
2. [Rust through rustup](https://rustup.rs/), which installs the Rust compiler
   and Cargo build tool.

Open PowerShell on Windows or Terminal on macOS/Linux and verify the installs:

```text
git --version
rustc --version
cargo --version
```

Each command should print a version. If `rustc` is older than 1.98, update it:

```text
rustup update stable
```

## 2. Download, build, and test the driver

These commands are the same in PowerShell and in a macOS/Linux terminal:

```text
git clone https://github.com/cmontella/moorebot-scout.git
cd moorebot-scout
cargo build --release
cargo test --all-targets
```

The first build downloads Rust packages and can take several minutes. A
successful test run ends with lines containing `test result: ok`. Compiler
warnings about future incompatibility in `buf_redux` or `multipart` come from
upstream ROS dependencies and do not indicate a failed build.

Before connecting a robot, try the offline examples:

```text
cargo run --example motion_mapping --no-default-features
cargo run --example list_known_interfaces --no-default-features
```

You can also inspect every command and option:

```text
cargo run -- --help
cargo run -- drive --help
```

## 3. Join the Scout network

1. Power on the Scout.
2. Connect your computer to the Wi-Fi network created by the Scout. The
   factory-default name is `robot_scout_xxxxxx` and the factory-default Wi-Fi
   password is `r0123456`, according to the [official Scout
   FAQ](https://www.moorebot.com/pages/faq-for-moorebot-scout-2). The FAQ says
   to change this default after the first login. A Scout already configured in
   access-point mode may instead be on the same trusted local network as your
   computer.
3. Find the computer's address on that network.

On Windows, run `ipconfig` and find the `IPv4 Address` under the active Wi-Fi
adapter. On macOS, first try `ipconfig getifaddr en0`; if that prints nothing,
use `ifconfig` and find the active Wi-Fi interface. On Linux, use `ip -4 addr`.

In direct-connect mode, choose the address beginning with `10.42.0.`. Ignore
loopback (`127.0.0.1`), disconnected adapters, Docker/virtual-machine adapters,
and addresses from a different network. The examples below use
`10.42.0.124`; replace it with your computer's actual value.

Check whether the robot responds:

- Windows: `ping -n 1 10.42.0.1`
- macOS/Linux: `ping -c 1 10.42.0.1`

A blocked ping does not always mean the robot is unreachable, but a successful
reply confirms the basic route.

## 4. Optional: open an SSH diagnostic shell

**Skip this section for normal driver use.** ROS connections do not require
SSH, and none of the driver commands below depend on it.

The supplied course archive reports this factory/lab SSH login:

| Setting | Supplied value |
|---|---|
| Address | `10.42.0.1` |
| Username | `linaro` |
| Password | `linaro` |

Firmware and classroom configuration may differ. From PowerShell or a
macOS/Linux terminal, try:

```text
ssh linaro@10.42.0.1
```

On the first connection, SSH asks whether you trust the host key. Verify that
you are connected directly to the expected Scout before accepting it. The
password is not displayed while you type. Once connected, safe read-only checks
include:

```text
hostname
ip addr
pgrep -a rosmaster
```

Type `exit` to close the shell. Do not use `sudo`, enable root access, delete
robot files, or change startup services for these exercises. If SSH is disabled
but the ROS master responds on port 11311, the Rust driver can still work.

Factory credentials are public and should not remain enabled on a robot placed
on a shared network. Change them using the supported Moorebot setup process,
record classroom-specific credentials somewhere private, and never put real
passwords into an issue or discovery capture.

## 5. Discover the robot

Run this from the project directory as one line, replacing the example
computer address:

```text
cargo run --release -- --master http://10.42.0.1:11311 --advertise-address 10.42.0.124 discover
```

The command asks the Scout's ROS master for every published topic and
registered service. Part of a successful result should resemble this:

```text
ROS master: http://10.42.0.1:11311
Published topics:
  /CoreNode/jpg                              roller_eye/frame                 JPEG camera [Bridge]
  /SensorNode/imu                            sensor_msgs/Imu                  6-axis IMU [Read]
Registered services:
  ...
```

Your list may differ with firmware version and robot state. `Read`, `Write`, or
`Bridge` means this version of the driver implements that interface.
`DiscoveredOnly` means the driver recognizes it but does not yet claim that it
works.

If the output mentions `linaro-alip` or a later command cannot resolve that
name, add this single mapping to your computer's hosts file:

```text
10.42.0.1 linaro-alip
```

- Windows hosts file: `C:\Windows\System32\drivers\etc\hosts` (open the editor
  as Administrator).
- macOS/Linux hosts file: `/etc/hosts` (editing requires administrator access
  on your computer, not on the robot).

Run `discover` again after saving the file.

## 6. Read the sensors

This example listens for 30 seconds:

```text
cargo run --release -- --master http://10.42.0.1:11311 --advertise-address 10.42.0.124 monitor --seconds 30
```

Representative output looks like this; your numbers will change continuously:

```text
imu    accel=(+0.012, -0.031, +9.801) m/s² gyro=(+0.001, +0.000, -0.002) rad/s
tof    range=0.375 m
light  raw=8061384 channels=(CH0=123, CH1=456)
battery 87% Discharging external_power=false
```

The program listens for the IMU, time-of-flight distance sensor, ambient-light
sensor, and battery state. Some Scout nodes start publishing only after a
subscriber appears, so `waiting for sensor publishers...` can be normal for a
few seconds.

Use `--seconds 0` to run until Ctrl-C:

```text
cargo run --release -- --master http://10.42.0.1:11311 --advertise-address 10.42.0.124 monitor --seconds 0
```

The displayed units follow the ROS message definitions, but physical scaling,
orientation, and firmware differences still require hardware validation.

## 7. Perform the first motion test

Complete this checklist first:

- The Scout is stable with all wheels off the surface.
- The area around every wheel is clear.
- `discover` completes successfully, confirming basic ROS connectivity.
- You know where the physical power control is.
- Another person nearby knows that the wheels may move.

Then request only 0.05 m/s forward for 250 milliseconds:

```text
cargo run --release -- --master http://10.42.0.1:11311 --advertise-address 10.42.0.124 drive --forward 0.05 --duration-ms 250
```

The driver refuses to move if no `/cmd_vel` subscriber connects within three
seconds. It caps requested speeds, limits a command to 60 seconds, and sends a
zero-velocity command after normal completion, an error, or Ctrl-C. These are
software safeguards, not a replacement for the physical precautions above.

Once the first command is understood, these are separate low-speed examples:

```text
# Ask for backward motion
cargo run --release -- --master http://10.42.0.1:11311 --advertise-address 10.42.0.124 drive --forward -0.05 --duration-ms 250

# Ask for leftward motion
cargo run --release -- --master http://10.42.0.1:11311 --advertise-address 10.42.0.124 drive --lateral 0.05 --duration-ms 250

# Ask for a counter-clockwise turn
cargo run --release -- --master http://10.42.0.1:11311 --advertise-address 10.42.0.124 drive --yaw 0.3 --duration-ms 250
```

Those direction names are the driver's intended standard coordinate semantics.
Confirm the real wheel directions while the robot is still on the stand and
report discrepancies in the hardware-validation issue.

## 8. Bridge the color camera

Start the bridge in one terminal:

```text
cargo run --release -- --master http://10.42.0.1:11311 --advertise-address 10.42.0.124 camera-bridge
```

It converts the Scout-specific `/CoreNode/jpg` messages into standard ROS 1
`sensor_msgs/CompressedImage` messages on:

```text
/moorebot_scout/camera/image/compressed
```

The terminal reports how many frames were forwarded or dropped when the bridge
stops. To display the stream, a second computer program must subscribe to the
output topic. For example, on a computer with ROS 1 desktop tools installed:

```text
rqt_image_view /moorebot_scout/camera/image/compressed
```

The Rust bridge itself still does not require a local ROS installation. Media
payloads above 16 MiB are rejected, and the bridge should be used only on a
trusted, isolated robot network because ROS 1 does not authenticate publishers.

## Troubleshooting

### `cargo`, `rustc`, or `git` is not recognized

Close and reopen the terminal after installation. If that does not help,
re-run the relevant installer and allow it to update your PATH.

### The driver cannot contact `10.42.0.1:11311`

Confirm that the Scout is powered on and that the computer is connected to the
correct network. Recheck the robot address. A VPN, firewall, campus network
policy, or virtual-machine network can block ROS connections.

On Windows, `Test-NetConnection 10.42.0.1 -Port 11311` checks the ROS master
port. On macOS/Linux, `nc -vz 10.42.0.1 11311` performs the same check when
Netcat is installed.

### The master responds, but subscriptions fail

Recheck `--advertise-address`. It must be your computer's address on the Scout
network, not the robot address and not `127.0.0.1`. Add the `linaro-alip` hosts
entry described above if the error names that host.

### `no subscriber connected to /cmd_vel`

The driver deliberately refused to send motion because it could not confirm a
motor-node subscriber. Run `discover`, verify the robot is fully booted, and
check address/firewall settings. Do not bypass this guard.

### Sensor monitoring keeps waiting

Allow several seconds for subscription-driven Scout nodes to start. If nothing
arrives, save the `discover` output, note the Scout model and firmware version,
and attach them to [hardware capture issue
#1](https://github.com/cmontella/moorebot-scout/issues/1).

### How to capture useful diagnostic output

The following saves discovery output to a file on every supported platform:

```text
cargo run --release -- --master http://10.42.0.1:11311 --advertise-address 10.42.0.124 discover > scout-discover.txt
```

Before sharing the file, remove private network names, addresses you do not
want public, credentials, and other identifying information. Never post Wi-Fi
passwords or private keys.
