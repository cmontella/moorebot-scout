use clap::{Args, Parser, Subcommand};
use moorebot_scout::{
    motion::{MotionLimits, ScoutTwist, Velocity},
    ros1::{self, CameraBridge, Ros1Config, TwistPublisher},
    sensors::{BatteryStatus, IlluminanceSample, ImuSample, RangeSample},
    services, topics,
};
use std::{
    error::Error,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread,
    time::{Duration, Instant},
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
        }
        .into(),
    };

    match cli.command {
        Command::Discover => discover(&config),
        Command::Drive(args) => drive(&config, args),
        Command::Monitor(args) => monitor(&config, args),
        Command::CameraBridge(args) => camera_bridge(&config, args),
    }
}

fn discover(config: &Ros1Config) -> Result<(), Box<dyn Error>> {
    ros1::init(config, true)?;
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
    ros1::init(config, false)?;
    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;

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
        thread::sleep(period);
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
    ros1::init(config, true)?;
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
    ros1::init(config, true)?;
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
