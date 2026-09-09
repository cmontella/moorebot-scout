use clap::{Args, Parser, Subcommand};
use crossterm::{
    event::{self, Event, KeyCode, KeyEvent, KeyEventKind, KeyModifiers},
    terminal::{disable_raw_mode, enable_raw_mode},
};
use moorebot_scout::{
    frame::{ScoutFrame, StreamType},
    motion::{MotionLimits, ScoutTwist, Velocity},
    ros1::{self, CameraBridge, Ros1Config, TwistPublisher},
    sensors::{BatteryStatus, IlluminanceSample, ImuSample, RangeSample},
    services, topics,
};
use std::{
    error::Error,
    fs,
    io::{self, Write},
    path::{Path, PathBuf},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

#[derive(Parser)]
#[command(version, about = "Rust driver tools for the Moorebot Scout")]
struct Cli {
    /// URI of the ROS 1 master running on the Scout.
    #[arg(long, global = true, default_value = "http://10.42.0.1:11311")]
    master: String,

    /// Address on this computer that the Scout can reach.
    #[arg(long, global = true)]
    advertise_address: Option<String>,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// List the live ROS graph and annotate known Scout capabilities.
    Discover,
    /// Send a bounded velocity command, followed by an unconditional stop.
    Drive(DriveArgs),
    /// Print decoded IMU, range, light, and battery samples.
    Monitor(MonitorArgs),
    /// Convert `/CoreNode/jpg` into standard `sensor_msgs/CompressedImage`.
    CameraBridge(CameraBridgeArgs),
    /// Drive interactively with WASD and save JPEGs with Space.
    Teleop(TeleopArgs),
}

#[derive(Args)]
struct DriveArgs {
    /// Forward speed in meters per second.
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    forward: f64,
    /// Leftward speed in meters per second.
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    lateral: f64,
    /// Counter-clockwise rotation in radians per second.
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    yaw: f64,
    /// Duration of the command. The driver sends zero velocity afterward.
    #[arg(long, default_value_t = 500)]
    duration_ms: u64,
    /// Command publication frequency.
    #[arg(long, default_value_t = 10.0)]
    rate_hz: f64,
}

#[derive(Args)]
struct MonitorArgs {
    /// Stop after this many seconds; use zero to run until Ctrl-C.
    #[arg(long, default_value_t = 10)]
    seconds: u64,
}

#[derive(Args)]
struct CameraBridgeArgs {
    #[arg(long, default_value = topics::JPEG)]
    source_topic: String,
    #[arg(long, default_value = "/moorebot_scout/camera/image/compressed")]
    output_topic: String,
}

#[derive(Args)]
struct TeleopArgs {
    /// Forward/backward speed in meters per second.
    #[arg(long, default_value_t = 0.10)]
    speed: f64,
    /// Rotation speed in radians per second.
    #[arg(long, default_value_t = 0.80)]
    turn_speed: f64,
    /// Velocity publication frequency.
    #[arg(long, default_value_t = 20.0)]
    rate_hz: f64,
    /// Stop this many milliseconds after the last movement key event.
    #[arg(long, default_value_t = 250)]
    deadman_ms: u64,
    /// Folder for Space-bar JPEG captures; defaults to the Windows Desktop.
    #[arg(long)]
    screenshot_dir: Option<PathBuf>,
}

#[derive(Clone, Debug)]
struct CameraSnapshot {
    sequence: u32,
    jpeg: Vec<u8>,
}

#[derive(Default)]
struct TeleopKeys {
    forward: bool,
    backward: bool,
    left: bool,
    right: bool,
    last_motion_event: Option<Instant>,
}

impl TeleopKeys {
    fn handle(&mut self, key: KeyEvent, now: Instant) {
        let pressed = !matches!(key.kind, KeyEventKind::Release);
        let changed = match key.code {
            KeyCode::Char('w') | KeyCode::Char('W') => {
                self.forward = pressed;
                true
            }
            KeyCode::Char('s') | KeyCode::Char('S') => {
                self.backward = pressed;
                true
            }
            KeyCode::Char('a') | KeyCode::Char('A') => {
                self.left = pressed;
                true
            }
            KeyCode::Char('d') | KeyCode::Char('D') => {
                self.right = pressed;
                true
            }
            _ => false,
        };
        if changed && pressed {
            self.last_motion_event = Some(now);
        }
    }

    fn velocity(&self, args: &TeleopArgs, now: Instant) -> Velocity {
        let active = self.last_motion_event.is_some_and(|last| {
            now.saturating_duration_since(last) <= Duration::from_millis(args.deadman_ms)
        });
        if !active {
            return Velocity::default();
        }

        Velocity {
            forward_mps: axis(self.forward, self.backward) * args.speed,
            lateral_mps: 0.0,
            yaw_rps: axis(self.left, self.right) * args.turn_speed,
        }
    }

    fn expire_stale(&mut self, now: Instant, deadman: Duration) {
        if self
            .last_motion_event
            .is_some_and(|last| now.saturating_duration_since(last) > deadman)
        {
            self.forward = false;
            self.backward = false;
            self.left = false;
            self.right = false;
            self.last_motion_event = None;
        }
    }
}

fn axis(positive: bool, negative: bool) -> f64 {
    f64::from(i8::from(positive) - i8::from(negative))
}

struct RawModeGuard;

impl RawModeGuard {
    fn enable() -> io::Result<Self> {
        enable_raw_mode()?;
        Ok(Self)
    }
}

impl Drop for RawModeGuard {
    fn drop(&mut self) {
        let _ = disable_raw_mode();
    }
}

#[derive(Default)]
struct SensorSnapshot {
    imu: Option<ImuSample>,
    tof: Option<RangeSample>,
    light: Option<IlluminanceSample>,
    battery: Option<BatteryStatus>,
}

fn main() {
    env_logger::init();
    if let Err(error) = run(Cli::parse()) {
        eprintln!("error: {error}");
        std::process::exit(1);
    }
}

fn run(cli: Cli) -> Result<(), Box<dyn Error>> {
    let config = Ros1Config {
        master_uri: cli.master,
        advertise_address: cli.advertise_address,
        node_name: match &cli.command {
            Command::Discover => "moorebot_scout_discover",
            Command::Drive(_) => "moorebot_scout_drive",
            Command::Monitor(_) => "moorebot_scout_monitor",
            Command::CameraBridge(_) => "moorebot_scout_camera_bridge",
            Command::Teleop(_) => "moorebot_scout_teleop",
        }
        .into(),
    };

    match cli.command {
        Command::Discover => discover(&config),
        Command::Drive(args) => drive(&config, args),
        Command::Monitor(args) => monitor(&config, args),
        Command::CameraBridge(args) => camera_bridge(&config, args),
        Command::Teleop(args) => teleop(&config, args),
    }
}

fn discover(config: &Ros1Config) -> Result<(), Box<dyn Error>> {
    // SAFETY: This command initializes ROS before starting application threads.
    unsafe { ros1::init(config, true)? };
    let mut published = ros1::published_topics()?;
    published.sort_by(|left, right| left.name.cmp(&right.name));

    println!("ROS master: {}", config.master_uri);
    println!("Published topics:");
    for topic in published {
        if let Some(known) = topics::known_topic(&topic.name) {
            println!(
                "  {:<42} {:<32} {} [{:?}]",
                topic.name, topic.message_type, known.capability, known.support
            );
        } else {
            println!("  {:<42} {}", topic.name, topic.message_type);
        }
    }

    let mut registered_services = ros1::registered_services()?;
    registered_services.sort_by(|left, right| left.name.cmp(&right.name));
    println!("Registered services:");
    for service in registered_services {
        if let Some(known) = services::known_service(&service.name) {
            println!(
                "  {:<42} {} [research-only: {}]",
                service.name, known.service_type, known.capability
            );
        } else {
            println!(
                "  {:<42} provider(s): {}",
                service.name,
                service.providers.join(", ")
            );
        }
    }
    Ok(())
}

fn next_motion_wait(period: Duration, now: Instant, deadline: Instant) -> Option<Duration> {
    deadline
        .checked_duration_since(now)
        .filter(|remaining| !remaining.is_zero())
        .map(|remaining| period.min(remaining))
}

fn drive(config: &Ros1Config, args: DriveArgs) -> Result<(), Box<dyn Error>> {
    if args.duration_ms == 0 {
        return Err("--duration-ms must be greater than zero".into());
    }
    if args.duration_ms > 60_000 {
        return Err("--duration-ms may not exceed 60000 in this safety-oriented CLI".into());
    }
    if !args.rate_hz.is_finite() || !(1.0..=100.0).contains(&args.rate_hz) {
        return Err("--rate-hz must be finite and between 1 and 100".into());
    }

    let command = Velocity {
        forward_mps: args.forward,
        lateral_mps: args.lateral,
        yaw_rps: args.yaw,
    }
    .to_scout_twist(MotionLimits::default())?;

    // Own Ctrl-C handling so a zero command can be queued before shutdown.
    // SAFETY: This command initializes ROS before installing the signal handler
    // or starting any other application threads.
    unsafe { ros1::init(config, false)? };
    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    let drive_thread = thread::current();
    ctrlc::set_handler(move || {
        signal_running.store(false, Ordering::SeqCst);
        drive_thread.unpark();
    })?;

    let publisher = TwistPublisher::new(topics::CMD_VEL, 2)?;
    if !publisher.wait_for_a_subscriber(Duration::from_secs(3)) {
        return Err("no subscriber connected to /cmd_vel; refusing to send motion".into());
    }

    println!(
        "Scout command: forward={:.3} m/s lateral={:.3} m/s yaw={:.3} rad/s for {} ms",
        command.linear_y, command.linear_x, command.angular_z, args.duration_ms
    );

    let period = Duration::from_secs_f64(1.0 / args.rate_hz);
    let deadline = Instant::now() + Duration::from_millis(args.duration_ms);
    let mut send_error = None;
    while running.load(Ordering::SeqCst) && Instant::now() < deadline {
        if let Err(error) = publisher.send(command) {
            send_error = Some(error);
            break;
        }
        let Some(wait) = next_motion_wait(period, Instant::now(), deadline) else {
            break;
        };
        thread::park_timeout(wait);
    }

    // Queue stop before shutting down the ROS transport, including error and
    // Ctrl-C paths. A short flush interval gives TCPROS time to transmit it.
    let stop_result = publisher.send(ScoutTwist::zero());
    thread::sleep(Duration::from_millis(100));
    rosrust::shutdown();

    if let Some(error) = send_error {
        return Err(error.into());
    }
    stop_result?;
    println!("Stop command sent.");
    Ok(())
}

fn monitor(config: &Ros1Config, args: MonitorArgs) -> Result<(), Box<dyn Error>> {
    // SAFETY: This command initializes ROS before creating subscriptions and
    // their worker threads.
    unsafe { ros1::init(config, true)? };
    let snapshot = Arc::new(Mutex::new(SensorSnapshot::default()));
    let mut subscriptions = Vec::new();

    let imu_snapshot = Arc::clone(&snapshot);
    subscriptions.push(ros1::subscribe_raw(topics::IMU, 20, move |bytes| {
        if let Ok(value) = ImuSample::decode_ros(&bytes) {
            imu_snapshot.lock().expect("sensor snapshot poisoned").imu = Some(value);
        }
    })?);

    let tof_snapshot = Arc::clone(&snapshot);
    subscriptions.push(ros1::subscribe_raw(topics::TOF, 5, move |bytes| {
        if let Ok(value) = RangeSample::decode_ros(&bytes) {
            tof_snapshot.lock().expect("sensor snapshot poisoned").tof = Some(value);
        }
    })?);

    let light_snapshot = Arc::clone(&snapshot);
    subscriptions.push(ros1::subscribe_raw(topics::LIGHT, 5, move |bytes| {
        if let Ok(value) = IlluminanceSample::decode_ros(&bytes) {
            light_snapshot
                .lock()
                .expect("sensor snapshot poisoned")
                .light = Some(value);
        }
    })?);

    let battery_snapshot = Arc::clone(&snapshot);
    subscriptions.push(ros1::subscribe_raw(topics::BATTERY, 5, move |bytes| {
        if let Ok(value) = BatteryStatus::decode_ros(&bytes) {
            battery_snapshot
                .lock()
                .expect("sensor snapshot poisoned")
                .battery = Some(value);
        }
    })?);

    println!("Monitoring Scout sensors from {}...", config.master_uri);
    let started = Instant::now();
    while rosrust::is_ok()
        && (args.seconds == 0 || started.elapsed() < Duration::from_secs(args.seconds))
    {
        thread::sleep(Duration::from_secs(1));
        let current = snapshot.lock().expect("sensor snapshot poisoned");
        if let Some(imu) = &current.imu {
            println!(
                "imu    accel=({:+.3}, {:+.3}, {:+.3}) m/s² gyro=({:+.3}, {:+.3}, {:+.3}) rad/s",
                imu.linear_acceleration.x,
                imu.linear_acceleration.y,
                imu.linear_acceleration.z,
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z,
            );
        }
        if let Some(tof) = &current.tof {
            println!("tof    range={:.3} m", tof.range_m);
        }
        if let Some(light) = &current.light {
            match light.packed_channels() {
                Some((ch0, ch1)) => println!(
                    "light  raw={:.0} channels=(CH0={}, CH1={})",
                    light.illuminance, ch0, ch1
                ),
                None => println!("light  illuminance={:.3}", light.illuminance),
            }
        }
        if let Some(battery) = &current.battery {
            println!(
                "battery {}% {:?} external_power={}",
                battery.percentage, battery.state, battery.externally_powered
            );
        }
        if current.imu.is_none()
            && current.tof.is_none()
            && current.light.is_none()
            && current.battery.is_none()
        {
            println!("waiting for sensor publishers...");
        }
    }

    drop(subscriptions);
    Ok(())
}

fn camera_bridge(config: &Ros1Config, args: CameraBridgeArgs) -> Result<(), Box<dyn Error>> {
    // SAFETY: This command initializes ROS before starting the camera bridge.
    unsafe { ros1::init(config, true)? };
    let bridge = CameraBridge::start(&args.source_topic, &args.output_topic)?;
    println!(
        "Bridging {} -> {} as sensor_msgs/CompressedImage (Ctrl-C to stop)",
        args.source_topic, args.output_topic
    );
    rosrust::spin();
    println!(
        "Forwarded {} frame(s); dropped {} malformed/non-JPEG frame(s).",
        bridge.forwarded_frames(),
        bridge.dropped_frames()
    );
    Ok(())
}

fn teleop(config: &Ros1Config, args: TeleopArgs) -> Result<(), Box<dyn Error>> {
    if !args.speed.is_finite() || args.speed <= 0.0 {
        return Err("--speed must be finite and greater than zero".into());
    }
    if !args.turn_speed.is_finite() || args.turn_speed <= 0.0 {
        return Err("--turn-speed must be finite and greater than zero".into());
    }
    if !args.rate_hz.is_finite() || !(1.0..=100.0).contains(&args.rate_hz) {
        return Err("--rate-hz must be finite and between 1 and 100".into());
    }
    if !(100..=2_000).contains(&args.deadman_ms) {
        return Err("--deadman-ms must be between 100 and 2000".into());
    }

    let limits = MotionLimits::default();
    Velocity {
        forward_mps: args.speed,
        lateral_mps: 0.0,
        yaw_rps: args.turn_speed,
    }
    .to_scout_twist(limits)?;

    // Own Ctrl-C handling so the terminal is restored and zero velocity is sent.
    // SAFETY: ROS is initialized before any application threads are started.
    unsafe { ros1::init(config, false)? };
    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;

    let publisher = TwistPublisher::new(topics::CMD_VEL, 2)?;
    if !publisher.wait_for_a_subscriber(Duration::from_secs(5)) {
        return Err(
            "no subscriber connected to /cmd_vel; pass --advertise-address with this computer's 10.42.0.x Wi-Fi address"
                .into(),
        );
    }

    let latest_frame = Arc::new(Mutex::new(None::<CameraSnapshot>));
    let callback_frame = Arc::clone(&latest_frame);
    let _camera_subscription = ros1::subscribe_raw(topics::JPEG, 1, move |bytes| {
        let Ok(frame) = ScoutFrame::decode_ros(&bytes) else {
            return;
        };
        if frame.stream_type == StreamType::Jpeg && frame.has_jpeg_markers() {
            *callback_frame.lock().expect("camera snapshot poisoned") = Some(CameraSnapshot {
                sequence: frame.sequence,
                jpeg: frame.data,
            });
        }
    })?;

    let screenshot_dir = args
        .screenshot_dir
        .clone()
        .unwrap_or_else(default_screenshot_dir);
    fs::create_dir_all(&screenshot_dir)?;
    let _raw_mode = RawModeGuard::enable()?;
    println!("W/S forward/back, A/D turn, Space capture, Esc or Ctrl-C quit");
    println!("Screenshots: {}", screenshot_dir.display());
    io::stdout().flush()?;

    let period = Duration::from_secs_f64(1.0 / args.rate_hz);
    let deadman = Duration::from_millis(args.deadman_ms);
    let mut keys = TeleopKeys::default();
    let loop_result: Result<(), Box<dyn Error>> = (|| {
        while running.load(Ordering::SeqCst) && rosrust::is_ok() {
            let iteration_started = Instant::now();
            keys.expire_stale(iteration_started, deadman);
            while event::poll(Duration::ZERO)? {
                let Event::Key(key) = event::read()? else {
                    continue;
                };
                if key.code == KeyCode::Esc
                    || (key.modifiers.contains(KeyModifiers::CONTROL)
                        && matches!(key.code, KeyCode::Char('c') | KeyCode::Char('C')))
                {
                    running.store(false, Ordering::SeqCst);
                    break;
                }
                if key.code == KeyCode::Char(' ') && matches!(key.kind, KeyEventKind::Press) {
                    match save_latest_screenshot(&latest_frame, &screenshot_dir) {
                        Ok(path) => println!("Captured {}", path.display()),
                        Err(error) => eprintln!("capture failed: {error}"),
                    }
                    io::stdout().flush()?;
                }
                keys.handle(key, Instant::now());
            }

            let command = keys
                .velocity(&args, Instant::now())
                .to_scout_twist(limits)?;
            publisher.send(command)?;
            if let Some(wait) = period.checked_sub(iteration_started.elapsed()) {
                thread::sleep(wait);
            }
        }
        Ok(())
    })();

    // Always attempt a stop, including terminal-input and publisher error paths.
    let stop_result = publisher.send(ScoutTwist::zero());
    thread::sleep(Duration::from_millis(100));
    rosrust::shutdown();
    loop_result?;
    stop_result?;
    println!("\nStopped.");
    Ok(())
}

fn default_screenshot_dir() -> PathBuf {
    std::env::var_os("USERPROFILE")
        .map(PathBuf::from)
        .map(|profile| profile.join("Desktop"))
        .filter(|desktop| desktop.is_dir())
        .or_else(|| std::env::current_dir().ok())
        .unwrap_or_else(|| PathBuf::from("."))
}

fn save_latest_screenshot(
    latest_frame: &Mutex<Option<CameraSnapshot>>,
    directory: &Path,
) -> Result<PathBuf, Box<dyn Error>> {
    let snapshot = latest_frame
        .lock()
        .map_err(|_| "camera snapshot lock is poisoned")?
        .clone()
        .ok_or("no camera frame has arrived yet")?;
    let timestamp_ms = SystemTime::now().duration_since(UNIX_EPOCH)?.as_millis();
    let path = directory.join(format!(
        "scout-{timestamp_ms}-frame-{}.jpg",
        snapshot.sequence
    ));
    fs::write(&path, snapshot.jpeg)?;
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motion_wait_does_not_extend_past_deadline() {
        let now = Instant::now();
        let deadline = now + Duration::from_millis(500);

        assert_eq!(
            next_motion_wait(Duration::from_secs(1), now, deadline),
            Some(Duration::from_millis(500))
        );
        assert_eq!(
            next_motion_wait(Duration::from_millis(100), now, deadline),
            Some(Duration::from_millis(100))
        );
        assert_eq!(
            next_motion_wait(Duration::from_secs(1), deadline, deadline),
            None
        );
    }

    #[test]
    fn teleop_opposite_keys_cancel_each_other() {
        assert_eq!(axis(false, false), 0.0);
        assert_eq!(axis(true, false), 1.0);
        assert_eq!(axis(false, true), -1.0);
        assert_eq!(axis(true, true), 0.0);
    }

    #[test]
    fn teleop_deadman_clears_stale_key_state() {
        let now = Instant::now();
        let mut keys = TeleopKeys {
            forward: true,
            last_motion_event: Some(now),
            ..TeleopKeys::default()
        };

        keys.expire_stale(now + Duration::from_millis(251), Duration::from_millis(250));

        assert!(!keys.forward);
        assert!(keys.last_motion_event.is_none());
    }
}
