use clap::{Args, Parser, Subcommand};
use crossterm::{
    event::{
        self, Event, KeyCode, KeyEvent, KeyEventKind, KeyModifiers, KeyboardEnhancementFlags,
        PopKeyboardEnhancementFlags, PushKeyboardEnhancementFlags,
    },
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, supports_keyboard_enhancement},
};
use moorebot_scout::{
    frame::{ScoutFrame, StreamType},
    motion::{MotionLimits, ScoutTwist, Velocity},
    robot::{KnownScout, known_scout_in_text},
    ros1::{
        self, CameraBridge, ConnectionInfo, MAX_BATTERY_MESSAGE_BYTES, MAX_SENSOR_MESSAGE_BYTES,
        Ros1Config, TwistPublisher,
    },
    sensors::{BatteryStatus, IlluminanceSample, ImuSample, RangeSample},
    services, topics,
};
use std::{
    error::Error,
    fs::{self, OpenOptions},
    io::{self, Read, Write},
    path::{Path, PathBuf},
    process::{Command as ProcessCommand, Stdio},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

const TELEOP_INITIAL_REPEAT_GRACE: Duration = Duration::from_millis(1_100);
const TELEOP_REPEAT_DEADMAN: Duration = Duration::from_millis(350);
const SPEED_SCALE_STEP: f64 = 0.25;
const MIN_SPEED_SCALE: f64 = 0.25;
const MAX_SYSTEM_COMMAND_OUTPUT_BYTES: u64 = 64 * 1024;
const TELEOP_HELP: &str = "Controls:\n  W / S       forward / backward\n  A / D       strafe left / right\n  Q / E       turn left / right\n  Up / Down   increase / decrease speed and control rate\n  Space       save the latest camera frame to the Desktop\n  Esc/Ctrl-C  stop and exit";

#[derive(Parser)]
#[command(version, about = "Rust driver tools for the Moorebot Scout")]
struct Cli {
    /// URI of the ROS 1 master running on the Scout.
    #[arg(long, global = true, default_value = "http://10.42.0.1:11311")]
    master: String,

    /// Override the local address automatically selected for the Scout route.
    #[arg(long, global = true)]
    advertise_address: Option<String>,

    /// Show detailed ROS transport diagnostics.
    #[arg(short, long, global = true)]
    verbose: bool,

    #[command(subcommand)]
    command: Option<Command>,
}

#[derive(Subcommand)]
enum Command {
    /// List the live ROS graph and annotate known Scout capabilities.
    Discover,
    /// Send a bounded velocity command, followed by an unconditional stop.
    Drive(DriveArgs),
    /// Drive with WASD and save the latest camera picture with Space.
    Teleop(TeleopArgs),
    /// Print decoded IMU, range, light, and battery samples.
    Monitor(MonitorArgs),
    /// Convert `/CoreNode/jpg` into standard `sensor_msgs/CompressedImage`.
    CameraBridge(CameraBridgeArgs),
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
#[command(after_help = TELEOP_HELP)]
struct TeleopArgs {
    /// Forward and reverse speed in meters per second.
    #[arg(long, default_value_t = 0.10)]
    speed: f64,
    /// Left/right strafe speed in meters per second.
    #[arg(long, default_value_t = 0.10)]
    strafe_speed: f64,
    /// Rotation speed in radians per second.
    #[arg(long, default_value_t = 2.0)]
    turn_speed: f64,
    /// Base velocity publication frequency; Up/Down adjusts it with speed.
    #[arg(long, default_value_t = 40.0)]
    rate_hz: f64,
    /// Directory in which Space saves JPEG pictures; defaults to the Desktop.
    #[arg(long, visible_alias = "screenshot-dir")]
    picture_directory: Option<PathBuf>,
    /// Scout JPEG camera topic.
    #[arg(long, default_value = topics::JPEG)]
    camera_topic: String,
}

impl Default for TeleopArgs {
    fn default() -> Self {
        Self {
            speed: 0.10,
            strafe_speed: 0.10,
            turn_speed: 2.0,
            rate_hz: 40.0,
            picture_directory: None,
            camera_topic: topics::JPEG.into(),
        }
    }
}

#[derive(Args)]
struct MonitorArgs {
    /// Stop after this many seconds; use zero to run until Ctrl-C.
    #[arg(long, default_value_t = 0)]
    seconds: u64,
}

#[derive(Args)]
struct CameraBridgeArgs {
    #[arg(long, default_value = topics::JPEG)]
    source_topic: String,
    #[arg(long, default_value = "/moorebot_scout/camera/image/compressed")]
    output_topic: String,
}

#[derive(Default)]
struct SensorSnapshot {
    imu: Option<ImuSample>,
    tof: Option<RangeSample>,
    light: Option<IlluminanceSample>,
    battery: Option<BatteryStatus>,
}

fn main() {
    let cli = Cli::parse();
    let default_log_filter = if cli.verbose {
        "info"
    } else {
        "warn,rosrust=off"
    };
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(default_log_filter))
        .init();
    if let Err(error) = run(cli) {
        eprintln!("\nCould not start the Moorebot Scout driver:\n  {error}");
        eprintln!("\nRun with --verbose if an instructor needs the ROS diagnostics.");
        std::process::exit(1);
    }
}

fn run(cli: Cli) -> Result<(), Box<dyn Error>> {
    let command = cli
        .command
        .unwrap_or_else(|| Command::Teleop(TeleopArgs::default()));
    let config = Ros1Config {
        master_uri: cli.master,
        advertise_address: cli.advertise_address,
        node_name: match &command {
            Command::Discover => "moorebot_scout_discover",
            Command::Drive(_) => "moorebot_scout_drive",
            Command::Teleop(_) => "moorebot_scout_teleop",
            Command::Monitor(_) => "moorebot_scout_monitor",
            Command::CameraBridge(_) => "moorebot_scout_camera_bridge",
        }
        .into(),
    };

    println!("Looking for a Moorebot Scout...");
    match command {
        Command::Discover => discover(&config),
        Command::Drive(args) => drive(&config, args),
        Command::Teleop(args) => teleop(&config, args),
        Command::Monitor(args) => monitor(&config, args),
        Command::CameraBridge(args) => camera_bridge(&config, args),
    }
}

fn discover(config: &Ros1Config) -> Result<(), Box<dyn Error>> {
    // SAFETY: This command initializes ROS before starting application threads.
    let connection = unsafe { ros1::init(config, true, MAX_SENSOR_MESSAGE_BYTES)? };
    announce_connection(connection);
    let mut published = ros1::published_topics()?;
    published.sort_by(|left, right| left.name.cmp(&right.name));

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
    if args.forward == 0.0 && args.lateral == 0.0 && args.yaw == 0.0 {
        return Err(
            "drive needs movement: use --forward, --lateral, or --yaw (for example: moorebot-scout drive --forward 0.1)"
                .into(),
        );
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
    let connection = unsafe { ros1::init(config, false, MAX_SENSOR_MESSAGE_BYTES)? };
    announce_connection(connection);
    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    let drive_thread = thread::current();
    ctrlc::set_handler(move || {
        signal_running.store(false, Ordering::SeqCst);
        drive_thread.unpark();
    })?;

    let publisher = TwistPublisher::new(topics::CMD_VEL, 2)?;
    if !publisher.wait_for_a_subscriber(Duration::from_secs(3)) {
        return Err(motor_connection_error().into());
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum TeleopAction {
    None,
    Capture,
    Faster,
    Slower,
    Exit,
}

#[derive(Default)]
struct TeleopControl {
    forward_until: Option<Instant>,
    reverse_until: Option<Instant>,
    strafe_left_until: Option<Instant>,
    strafe_right_until: Option<Instant>,
    turn_left_until: Option<Instant>,
    turn_right_until: Option<Instant>,
}

impl TeleopControl {
    fn handle_key(&mut self, key: KeyEvent, now: Instant) -> TeleopAction {
        if key.modifiers.contains(KeyModifiers::CONTROL)
            && matches!(key.code, KeyCode::Char('c' | 'C'))
        {
            return TeleopAction::Exit;
        }

        if matches!(key.kind, KeyEventKind::Press | KeyEventKind::Repeat) {
            match key.code {
                KeyCode::Esc => return TeleopAction::Exit,
                KeyCode::Char(' ') if key.kind == KeyEventKind::Press => {
                    return TeleopAction::Capture;
                }
                KeyCode::Up if key.kind == KeyEventKind::Press => {
                    return TeleopAction::Faster;
                }
                KeyCode::Down if key.kind == KeyEventKind::Press => {
                    return TeleopAction::Slower;
                }
                KeyCode::Char(character) => match character.to_ascii_lowercase() {
                    'w' => Self::refresh(&mut self.forward_until, key.kind, now),
                    's' => Self::refresh(&mut self.reverse_until, key.kind, now),
                    'a' => Self::refresh(&mut self.strafe_left_until, key.kind, now),
                    'd' => Self::refresh(&mut self.strafe_right_until, key.kind, now),
                    'q' => Self::refresh(&mut self.turn_left_until, key.kind, now),
                    'e' => Self::refresh(&mut self.turn_right_until, key.kind, now),
                    _ => {}
                },
                _ => {}
            }
        } else if key.kind == KeyEventKind::Release {
            match key.code {
                KeyCode::Char('w' | 'W') => self.forward_until = None,
                KeyCode::Char('s' | 'S') => self.reverse_until = None,
                KeyCode::Char('a' | 'A') => self.strafe_left_until = None,
                KeyCode::Char('d' | 'D') => self.strafe_right_until = None,
                KeyCode::Char('q' | 'Q') => self.turn_left_until = None,
                KeyCode::Char('e' | 'E') => self.turn_right_until = None,
                _ => {}
            }
        }

        TeleopAction::None
    }

    fn refresh(deadline: &mut Option<Instant>, kind: KeyEventKind, now: Instant) {
        // Legacy terminals report every auto-repeat as another Press. Treat a
        // Press received while this key is still active as a repeat, while the
        // first Press gets enough grace for normal desktop repeat delays.
        let is_repeat = kind == KeyEventKind::Repeat
            || deadline.is_some_and(|current_deadline| current_deadline > now);
        let timeout = if is_repeat {
            TELEOP_REPEAT_DEADMAN
        } else {
            TELEOP_INITIAL_REPEAT_GRACE
        };
        *deadline = Some(now + timeout);
    }

    fn velocity(
        &mut self,
        now: Instant,
        speed: f64,
        strafe_speed: f64,
        turn_speed: f64,
        speed_scale: f64,
    ) -> Velocity {
        let forward = Self::active(&mut self.forward_until, now) as i8
            - Self::active(&mut self.reverse_until, now) as i8;
        let strafe = Self::active(&mut self.strafe_left_until, now) as i8
            - Self::active(&mut self.strafe_right_until, now) as i8;
        let turn = Self::active(&mut self.turn_left_until, now) as i8
            - Self::active(&mut self.turn_right_until, now) as i8;

        Velocity {
            forward_mps: f64::from(forward) * speed * speed_scale,
            lateral_mps: f64::from(strafe) * strafe_speed * speed_scale,
            yaw_rps: f64::from(turn) * turn_speed * speed_scale,
        }
    }

    fn active(deadline: &mut Option<Instant>, now: Instant) -> bool {
        if deadline.is_some_and(|deadline| deadline > now) {
            true
        } else {
            *deadline = None;
            false
        }
    }

    fn clear(&mut self) {
        *self = Self::default();
    }
}

fn maximum_speed_scale(args: &TeleopArgs, limits: MotionLimits) -> f64 {
    [
        limits.max_forward_mps / args.speed,
        limits.max_lateral_mps / args.strafe_speed,
        limits.max_yaw_rps / args.turn_speed,
        100.0 / args.rate_hz,
    ]
    .into_iter()
    .fold(f64::INFINITY, f64::min)
    .max(MIN_SPEED_SCALE)
}

fn adjusted_speed_scale(current: f64, increase: bool, maximum: f64) -> f64 {
    let change = if increase {
        SPEED_SCALE_STEP
    } else {
        -SPEED_SCALE_STEP
    };
    (current + change).clamp(MIN_SPEED_SCALE, maximum)
}

struct RawTerminal {
    keyboard_enhancements: bool,
}

impl RawTerminal {
    fn enter() -> io::Result<Self> {
        enable_raw_mode()?;
        let keyboard_enhancements = supports_keyboard_enhancement().unwrap_or(false);
        if keyboard_enhancements
            && let Err(error) = execute!(
                io::stdout(),
                PushKeyboardEnhancementFlags(
                    KeyboardEnhancementFlags::DISAMBIGUATE_ESCAPE_CODES
                        | KeyboardEnhancementFlags::REPORT_EVENT_TYPES
                )
            )
        {
            let _ = disable_raw_mode();
            return Err(error);
        }

        Ok(Self {
            keyboard_enhancements,
        })
    }
}

impl Drop for RawTerminal {
    fn drop(&mut self) {
        if self.keyboard_enhancements {
            let _ = execute!(io::stdout(), PopKeyboardEnhancementFlags);
        }
        let _ = disable_raw_mode();
    }
}

fn validate_teleop_speed(name: &str, value: f64, maximum: f64) -> Result<(), Box<dyn Error>> {
    if !value.is_finite() || value <= 0.0 || value > maximum {
        return Err(format!(
            "{name} must be finite, greater than zero, and no more than {maximum}"
        )
        .into());
    }
    Ok(())
}

fn save_latest_picture(
    latest_frame: &Mutex<Option<ScoutFrame>>,
    directory: &Path,
) -> Result<Option<PathBuf>, Box<dyn Error>> {
    let frame = latest_frame
        .lock()
        .map_err(|_| "camera frame storage was poisoned")?;
    let Some(frame) = frame.as_ref() else {
        return Ok(None);
    };

    let timestamp_ms = SystemTime::now().duration_since(UNIX_EPOCH)?.as_millis();
    for suffix in 0..100_u8 {
        let suffix = if suffix == 0 {
            String::new()
        } else {
            format!("-{suffix}")
        };
        let path = directory.join(format!(
            "scout-{timestamp_ms}-frame-{}{}.jpg",
            frame.sequence, suffix
        ));
        match OpenOptions::new().write(true).create_new(true).open(&path) {
            Ok(mut file) => {
                file.write_all(&frame.data)?;
                return Ok(Some(path));
            }
            Err(error) if error.kind() == io::ErrorKind::AlreadyExists => continue,
            Err(error) => return Err(error.into()),
        }
    }

    Err("could not choose a unique picture filename".into())
}

fn run_teleop_session(
    publisher: &TwistPublisher,
    latest_frame: &Mutex<Option<ScoutFrame>>,
    running: &AtomicBool,
    args: &TeleopArgs,
    picture_directory: &Path,
) -> Result<(), Box<dyn Error>> {
    let _terminal = RawTerminal::enter()?;
    let mut control = TeleopControl::default();
    let mut next_publish = Instant::now();
    let maximum_scale = maximum_speed_scale(args, MotionLimits::default());
    let mut speed_scale = 1.0_f64.min(maximum_scale);

    publisher.send(ScoutTwist::zero())?;
    while running.load(Ordering::SeqCst) && rosrust::is_ok() {
        let now = Instant::now();
        let period = Duration::from_secs_f64(1.0 / (args.rate_hz * speed_scale));
        let wait = next_publish
            .checked_duration_since(now)
            .unwrap_or(Duration::ZERO)
            .min(period);

        if event::poll(wait)?
            && let Event::Key(key) = event::read()?
        {
            match control.handle_key(key, Instant::now()) {
                TeleopAction::None => {}
                TeleopAction::Exit => break,
                TeleopAction::Capture => {
                    // Stop before touching the disk so a slow or failed write
                    // cannot leave the previous motion command active.
                    control.clear();
                    publisher.send(ScoutTwist::zero())?;
                    match save_latest_picture(latest_frame, picture_directory)? {
                        Some(path) => println!("\r\nSaved {}", path.display()),
                        None => println!("\r\nNo JPEG camera frame has arrived yet."),
                    }
                    next_publish = Instant::now() + period;
                }
                action @ (TeleopAction::Faster | TeleopAction::Slower) => {
                    speed_scale = adjusted_speed_scale(
                        speed_scale,
                        matches!(action, TeleopAction::Faster),
                        maximum_scale,
                    );
                    println!(
                        "\r\nSpeed: {:.0}% | control rate: {:.0} Hz",
                        speed_scale * 100.0,
                        args.rate_hz * speed_scale
                    );
                    io::stdout().flush()?;
                    next_publish = Instant::now();
                }
            }
        }

        let now = Instant::now();
        if now >= next_publish {
            let command = control
                .velocity(
                    now,
                    args.speed,
                    args.strafe_speed,
                    args.turn_speed,
                    speed_scale,
                )
                .to_scout_twist(MotionLimits::default())?;
            publisher.send(command)?;
            next_publish = now + Duration::from_secs_f64(1.0 / (args.rate_hz * speed_scale));
        }
    }

    Ok(())
}

fn teleop(config: &Ros1Config, args: TeleopArgs) -> Result<(), Box<dyn Error>> {
    let limits = MotionLimits::default();
    validate_teleop_speed("--speed", args.speed, limits.max_forward_mps)?;
    validate_teleop_speed("--strafe-speed", args.strafe_speed, limits.max_lateral_mps)?;
    validate_teleop_speed("--turn-speed", args.turn_speed, limits.max_yaw_rps)?;
    if !args.rate_hz.is_finite() || !(1.0..=100.0).contains(&args.rate_hz) {
        return Err("--rate-hz must be finite and between 1 and 100".into());
    }
    let picture_directory = args
        .picture_directory
        .clone()
        .unwrap_or_else(default_picture_directory);
    fs::create_dir_all(&picture_directory)?;

    // SAFETY: This command initializes ROS before starting any ROS-owned
    // publishers, subscriptions, signal handlers, or worker threads.
    let connection = unsafe { ros1::init(config, false, ros1::MAX_MEDIA_MESSAGE_BYTES)? };
    announce_connection(connection);
    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;

    let latest_frame = Arc::new(Mutex::new(None));
    let callback_frame = Arc::clone(&latest_frame);
    let camera_subscription = ros1::subscribe_raw::<{ ros1::MAX_MEDIA_MESSAGE_BYTES }, _>(
        &args.camera_topic,
        1,
        move |bytes| {
            let Ok(frame) = ScoutFrame::decode_ros(&bytes) else {
                return;
            };
            if frame.stream_type != StreamType::Jpeg || !frame.has_jpeg_markers() {
                return;
            }
            if let Ok(mut current) = callback_frame.lock() {
                *current = Some(frame);
            }
        },
    )?;
    let publisher = TwistPublisher::new(topics::CMD_VEL, 2)?;
    if !publisher.wait_for_a_subscriber(Duration::from_secs(3)) {
        drop(camera_subscription);
        rosrust::shutdown();
        return Err(motor_connection_error().into());
    }

    if camera_subscription.publisher_count() == 0 {
        println!(
            "Camera: still waiting for {}. Driving is available; Space will work after a frame arrives.",
            args.camera_topic
        );
    }
    println!("{TELEOP_HELP}");
    let initial_scale = 1.0_f64.min(maximum_speed_scale(&args, limits));
    println!(
        "Pictures: {} | Speed: {:.0}% | control rate: {:.0} Hz\nKeep this terminal focused.",
        picture_directory.display(),
        initial_scale * 100.0,
        args.rate_hz * initial_scale
    );

    let session_result = run_teleop_session(
        &publisher,
        &latest_frame,
        &running,
        &args,
        &picture_directory,
    );
    let stop_result = publisher.send(ScoutTwist::zero());
    thread::sleep(Duration::from_millis(100));
    drop(camera_subscription);
    rosrust::shutdown();

    session_result?;
    stop_result?;
    println!("Stop command sent.");
    Ok(())
}

fn default_picture_directory() -> PathBuf {
    std::env::var_os("USERPROFILE")
        .map(PathBuf::from)
        .map(|profile| profile.join("Desktop"))
        .filter(|desktop| desktop.is_dir())
        .or_else(|| {
            std::env::var_os("HOME")
                .map(PathBuf::from)
                .map(|home| home.join("Desktop"))
        })
        .filter(|desktop| desktop.is_dir())
        .or_else(|| std::env::current_dir().ok())
        .unwrap_or_else(|| PathBuf::from("."))
}

fn monitor(config: &Ros1Config, args: MonitorArgs) -> Result<(), Box<dyn Error>> {
    // SAFETY: This command initializes ROS before creating subscriptions and
    // their worker threads.
    let connection = unsafe { ros1::init(config, true, MAX_SENSOR_MESSAGE_BYTES)? };
    announce_connection(connection);
    let snapshot = Arc::new(Mutex::new(SensorSnapshot::default()));
    let mut subscriptions = Vec::new();

    let imu_snapshot = Arc::clone(&snapshot);
    subscriptions.push(ros1::subscribe_raw::<MAX_SENSOR_MESSAGE_BYTES, _>(
        topics::IMU,
        1,
        move |bytes| {
            if let Ok(value) = ImuSample::decode_ros(&bytes) {
                imu_snapshot.lock().expect("sensor snapshot poisoned").imu = Some(value);
            }
        },
    )?);

    let tof_snapshot = Arc::clone(&snapshot);
    subscriptions.push(ros1::subscribe_raw::<MAX_SENSOR_MESSAGE_BYTES, _>(
        topics::TOF,
        1,
        move |bytes| {
            if let Ok(value) = RangeSample::decode_ros(&bytes) {
                tof_snapshot.lock().expect("sensor snapshot poisoned").tof = Some(value);
            }
        },
    )?);

    let light_snapshot = Arc::clone(&snapshot);
    subscriptions.push(ros1::subscribe_raw::<MAX_SENSOR_MESSAGE_BYTES, _>(
        topics::LIGHT,
        1,
        move |bytes| {
            if let Ok(value) = IlluminanceSample::decode_ros(&bytes) {
                light_snapshot
                    .lock()
                    .expect("sensor snapshot poisoned")
                    .light = Some(value);
            }
        },
    )?);

    let battery_snapshot = Arc::clone(&snapshot);
    subscriptions.push(ros1::subscribe_raw::<MAX_BATTERY_MESSAGE_BYTES, _>(
        topics::BATTERY,
        1,
        move |bytes| {
            if let Ok(value) = BatteryStatus::decode_ros(&bytes) {
                battery_snapshot
                    .lock()
                    .expect("sensor snapshot poisoned")
                    .battery = Some(value);
            }
        },
    )?);

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
    let connection = unsafe { ros1::init(config, true, ros1::MAX_MEDIA_MESSAGE_BYTES)? };
    announce_connection(connection);
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

fn motor_connection_error() -> &'static str {
    "the Scout motor controller did not connect, so no movement was sent. On Windows, allow moorebot-scout.exe through the firewall on Private networks; on every platform, verify that the Scout is fully started and this computer is still connected to its Wi-Fi"
}

fn announce_connection(connection: ConnectionInfo) {
    println!(
        "Connected to the Scout at {} (this computer: {}).",
        connection.master_address, connection.advertise_address
    );
    match detect_scout(connection.master_address) {
        Some(scout) => println!("Robot: {} ({})", scout.name, scout.mac_address),
        None => println!("Robot: connected; its classroom name was not detected."),
    }
}

fn detect_scout(master_address: std::net::SocketAddr) -> Option<KnownScout> {
    #[cfg(windows)]
    {
        let netsh = windows_system_program("netsh.exe")?;
        if let Some(scout) = command_output(&netsh, &["wlan", "show", "interfaces"])
            .as_deref()
            .and_then(known_scout_in_text)
        {
            return Some(scout);
        }
    }

    let address = master_address.ip().to_string();
    #[cfg(windows)]
    let program = windows_system_program("arp.exe")?;
    #[cfg(not(windows))]
    let program = ["/usr/sbin/arp", "/sbin/arp", "/usr/bin/arp"]
        .into_iter()
        .map(Path::new)
        .find(|path| path.is_file())?
        .to_owned();
    #[cfg(windows)]
    let arguments = ["-a", address.as_str()];
    #[cfg(not(windows))]
    let arguments = ["-n", address.as_str()];
    command_output(&program, &arguments)
        .as_deref()
        .and_then(known_scout_in_text)
}

#[cfg(windows)]
fn windows_system_program(filename: &str) -> Option<PathBuf> {
    use std::{ffi::OsString, os::windows::ffi::OsStringExt};
    use windows_sys::Win32::System::SystemInformation::GetSystemDirectoryW;

    if !matches!(filename, "netsh.exe" | "arp.exe") {
        return None;
    }
    let mut buffer = [0_u16; 32_768];
    // SAFETY: `buffer` is writable for the supplied element count. A zero
    // result or a required length outside the buffer is rejected below.
    let length = unsafe { GetSystemDirectoryW(buffer.as_mut_ptr(), buffer.len() as u32) } as usize;
    if length == 0 || length >= buffer.len() {
        return None;
    }
    let path = PathBuf::from(OsString::from_wide(&buffer[..length])).join(filename);
    path.is_file().then_some(path)
}

fn command_output(program: &Path, arguments: &[&str]) -> Option<String> {
    let mut child = ProcessCommand::new(program)
        .args(arguments)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .ok()?;
    let mut bytes = Vec::new();
    child
        .stdout
        .take()?
        .take(MAX_SYSTEM_COMMAND_OUTPUT_BYTES + 1)
        .read_to_end(&mut bytes)
        .ok()?;
    if bytes.len() as u64 > MAX_SYSTEM_COMMAND_OUTPUT_BYTES {
        let _ = child.kill();
        bytes.truncate(MAX_SYSTEM_COMMAND_OUTPUT_BYTES as usize);
    }
    let _ = child.wait();
    Some(String::from_utf8_lossy(&bytes).into_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_subcommand_selects_student_teleop_defaults() {
        let cli = Cli::try_parse_from(["moorebot-scout"]).unwrap();
        assert!(cli.command.is_none());
        let command = cli
            .command
            .unwrap_or_else(|| Command::Teleop(TeleopArgs::default()));
        let Command::Teleop(args) = command else {
            panic!("zero arguments must select teleop");
        };
        assert_eq!(args.speed, 0.10);
        assert_eq!(args.strafe_speed, 0.10);
        assert_eq!(args.turn_speed, 2.0);
        assert_eq!(args.rate_hz, 40.0);
        assert_eq!(args.camera_topic, topics::JPEG);
    }

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
    fn teleop_keys_move_release_and_expire() {
        let now = Instant::now();
        let mut control = TeleopControl::default();

        assert_eq!(
            control.handle_key(
                KeyEvent::new_with_kind(
                    KeyCode::Char('w'),
                    KeyModifiers::NONE,
                    KeyEventKind::Press
                ),
                now,
            ),
            TeleopAction::None
        );
        assert_eq!(control.velocity(now, 0.08, 0.1, 2.0, 1.0).forward_mps, 0.08);

        control.handle_key(
            KeyEvent::new_with_kind(
                KeyCode::Char('w'),
                KeyModifiers::NONE,
                KeyEventKind::Release,
            ),
            now,
        );
        assert_eq!(
            control.velocity(now, 0.08, 0.1, 2.0, 1.0),
            Velocity::default()
        );

        control.handle_key(
            KeyEvent::new_with_kind(KeyCode::Char('a'), KeyModifiers::NONE, KeyEventKind::Press),
            now,
        );
        assert_eq!(control.velocity(now, 0.08, 0.1, 2.0, 1.0).lateral_mps, 0.1);
        assert_eq!(control.velocity(now, 0.08, 0.1, 2.0, 1.0).yaw_rps, 0.0);
        assert_eq!(
            control
                .velocity(now + Duration::from_millis(500), 0.08, 0.1, 2.0, 1.0)
                .lateral_mps,
            0.1
        );
        assert_eq!(
            control.velocity(now + TELEOP_INITIAL_REPEAT_GRACE, 0.08, 0.1, 2.0, 1.0,),
            Velocity::default()
        );

        control.handle_key(
            KeyEvent::new_with_kind(KeyCode::Char('q'), KeyModifiers::NONE, KeyEventKind::Press),
            now,
        );
        assert_eq!(control.velocity(now, 0.08, 0.1, 2.0, 1.0).yaw_rps, 2.0);
    }

    #[test]
    fn teleop_repeat_switches_to_the_short_deadman() {
        let now = Instant::now();
        let mut control = TeleopControl::default();
        let press =
            KeyEvent::new_with_kind(KeyCode::Char('w'), KeyModifiers::NONE, KeyEventKind::Press);

        control.handle_key(press, now);
        let repeated_at = now + Duration::from_millis(500);
        // This covers legacy terminals, which report repeats as Press rather
        // than KeyEventKind::Repeat.
        control.handle_key(press, repeated_at);
        assert_eq!(
            control.velocity(repeated_at + TELEOP_REPEAT_DEADMAN, 0.08, 0.1, 2.0, 1.0,),
            Velocity::default()
        );
    }

    #[test]
    fn teleop_space_captures_once_and_escape_exits() {
        let now = Instant::now();
        let mut control = TeleopControl::default();

        assert_eq!(
            control.handle_key(
                KeyEvent::new_with_kind(
                    KeyCode::Char(' '),
                    KeyModifiers::NONE,
                    KeyEventKind::Press
                ),
                now,
            ),
            TeleopAction::Capture
        );
        assert_eq!(
            control.handle_key(
                KeyEvent::new_with_kind(
                    KeyCode::Char(' '),
                    KeyModifiers::NONE,
                    KeyEventKind::Repeat,
                ),
                now,
            ),
            TeleopAction::None
        );
        assert_eq!(
            control.handle_key(
                KeyEvent::new_with_kind(KeyCode::Esc, KeyModifiers::NONE, KeyEventKind::Press),
                now,
            ),
            TeleopAction::Exit
        );

        assert_eq!(
            control.handle_key(
                KeyEvent::new_with_kind(KeyCode::Up, KeyModifiers::NONE, KeyEventKind::Press),
                now,
            ),
            TeleopAction::Faster
        );
        assert_eq!(
            control.handle_key(
                KeyEvent::new_with_kind(KeyCode::Down, KeyModifiers::NONE, KeyEventKind::Press),
                now,
            ),
            TeleopAction::Slower
        );
    }

    #[test]
    fn teleop_speed_adjustment_is_bounded() {
        assert_eq!(adjusted_speed_scale(1.0, true, 2.0), 1.25);
        assert_eq!(adjusted_speed_scale(2.0, true, 2.0), 2.0);
        assert_eq!(adjusted_speed_scale(0.25, false, 2.0), 0.25);
    }

    #[test]
    fn picture_capture_writes_the_validated_jpeg_without_overwriting() {
        let unique = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let directory = std::env::temp_dir().join(format!(
            "moorebot-scout-picture-test-{}-{unique}",
            std::process::id()
        ));
        fs::create_dir(&directory).unwrap();
        let latest = Mutex::new(Some(ScoutFrame {
            sequence: 42,
            timestamp_ms: 1,
            session: 2,
            stream_type: StreamType::Jpeg,
            original_sequence: 42,
            parameters: [640, 480, 0, 0],
            data: vec![0xff, 0xd8, 1, 2, 3, 0xff, 0xd9],
        }));

        let first = save_latest_picture(&latest, &directory)
            .unwrap()
            .expect("picture should be available");
        let second = save_latest_picture(&latest, &directory)
            .unwrap()
            .expect("picture should be available");
        assert_ne!(first, second);
        assert_eq!(fs::read(&first).unwrap(), [0xff, 0xd8, 1, 2, 3, 0xff, 0xd9]);
        assert_eq!(
            fs::read(&second).unwrap(),
            [0xff, 0xd8, 1, 2, 3, 0xff, 0xd9]
        );

        fs::remove_file(first).unwrap();
        fs::remove_file(second).unwrap();
        fs::remove_dir(directory).unwrap();
    }
}
