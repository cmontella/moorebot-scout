use moorebot_scout::motion::{MotionLimits, ScoutTwist, Velocity};
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    io::{self, Read, Write},
    net::{Shutdown, SocketAddr, TcpListener, TcpStream},
    sync::{
        Arc, Condvar, Mutex,
        atomic::{AtomicBool, AtomicUsize, Ordering},
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

const PROTOCOL_VERSION: u8 = 1;
const MAX_CONTROL_LINE_BYTES: u64 = 4 * 1024;
const MAX_CAMERA_CLIENTS: usize = 4;
const NETWORK_POLL_INTERVAL: Duration = Duration::from_millis(100);
const CAMERA_WRITE_TIMEOUT: Duration = Duration::from_secs(2);

#[derive(Clone, Copy, Debug)]
pub struct BridgeOptions {
    pub control_address: SocketAddr,
    pub camera_address: SocketAddr,
    pub maximum_command_timeout: Duration,
}

#[derive(Clone, Copy, Debug)]
struct MotionState {
    command: ScoutTwist,
    deadline: Option<Instant>,
}

impl Default for MotionState {
    fn default() -> Self {
        Self {
            command: ScoutTwist::zero(),
            deadline: None,
        }
    }
}

impl MotionState {
    fn current(&mut self, now: Instant) -> ScoutTwist {
        if self.deadline.is_some_and(|deadline| deadline > now) {
            self.command
        } else {
            self.stop();
            ScoutTwist::zero()
        }
    }

    fn stop(&mut self) {
        self.command = ScoutTwist::zero();
        self.deadline = None;
    }
}

#[derive(Default)]
struct LatestFrame {
    generation: u64,
    jpeg: Option<Vec<u8>>,
}

type SharedFrame = Arc<(Mutex<LatestFrame>, Condvar)>;

#[derive(Clone)]
pub struct FrameSink {
    frame: SharedFrame,
}

impl FrameSink {
    pub fn publish(&self, jpeg: Vec<u8>) {
        let (lock, available) = &*self.frame;
        if let Ok(mut frame) = lock.lock() {
            frame.generation = frame.generation.wrapping_add(1);
            frame.jpeg = Some(jpeg);
            available.notify_all();
        }
    }
}

pub struct IntegrationBridge {
    control_address: SocketAddr,
    camera_address: SocketAddr,
    motion: Arc<Mutex<MotionState>>,
    frame: SharedFrame,
    running: Arc<AtomicBool>,
    threads: Vec<JoinHandle<()>>,
}

impl IntegrationBridge {
    pub fn start(options: BridgeOptions, running: Arc<AtomicBool>) -> io::Result<Self> {
        if !options.control_address.ip().is_loopback() || !options.camera_address.ip().is_loopback()
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "bridge addresses must use localhost/loopback; the bridge is not a network authentication boundary",
            ));
        }
        if options.control_address == options.camera_address && options.control_address.port() != 0
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "control and camera addresses must use different ports",
            ));
        }

        let control_listener = TcpListener::bind(options.control_address)?;
        let camera_listener = TcpListener::bind(options.camera_address)?;
        control_listener.set_nonblocking(true)?;
        camera_listener.set_nonblocking(true)?;
        let control_address = control_listener.local_addr()?;
        let camera_address = camera_listener.local_addr()?;

        let motion = Arc::new(Mutex::new(MotionState::default()));
        let frame = Arc::new((Mutex::new(LatestFrame::default()), Condvar::new()));
        let control_thread = {
            let running = Arc::clone(&running);
            let motion = Arc::clone(&motion);
            thread::Builder::new()
                .name("scout-bridge-control".into())
                .spawn(move || {
                    run_control_listener(
                        control_listener,
                        motion,
                        running,
                        options.maximum_command_timeout,
                    )
                })?
        };
        let camera_thread = {
            let running = Arc::clone(&running);
            let frame = Arc::clone(&frame);
            thread::Builder::new()
                .name("scout-bridge-camera".into())
                .spawn(move || run_camera_listener(camera_listener, frame, running))?
        };

        Ok(Self {
            control_address,
            camera_address,
            motion,
            frame,
            running,
            threads: vec![control_thread, camera_thread],
        })
    }

    pub fn control_address(&self) -> SocketAddr {
        self.control_address
    }

    pub fn camera_address(&self) -> SocketAddr {
        self.camera_address
    }

    pub fn frame_sink(&self) -> FrameSink {
        FrameSink {
            frame: Arc::clone(&self.frame),
        }
    }

    pub fn current_command(&self, now: Instant) -> ScoutTwist {
        self.motion
            .lock()
            .map(|mut motion| motion.current(now))
            .unwrap_or_else(|_| ScoutTwist::zero())
    }

    pub fn shutdown(mut self) {
        self.running.store(false, Ordering::SeqCst);
        if let Ok(mut motion) = self.motion.lock() {
            motion.stop();
        }
        self.frame.1.notify_all();
        for handle in self.threads.drain(..) {
            let _ = handle.join();
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
enum ControlRequest {
    Velocity {
        vx: f64,
        vy: f64,
        vtheta: f64,
        timeout_ms: u64,
    },
    Stop,
    Ping,
}

fn run_control_listener(
    listener: TcpListener,
    motion: Arc<Mutex<MotionState>>,
    running: Arc<AtomicBool>,
    maximum_timeout: Duration,
) {
    while running.load(Ordering::SeqCst) {
        match listener.accept() {
            Ok((stream, _)) => {
                if let Ok(mut state) = motion.lock() {
                    state.stop();
                }
                let _ = serve_control_client(stream, &motion, &running, maximum_timeout);
                if let Ok(mut state) = motion.lock() {
                    state.stop();
                }
            }
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                thread::sleep(NETWORK_POLL_INTERVAL);
            }
            Err(_) => thread::sleep(NETWORK_POLL_INTERVAL),
        }
    }
}

fn serve_control_client(
    mut stream: TcpStream,
    motion: &Mutex<MotionState>,
    running: &AtomicBool,
    maximum_timeout: Duration,
) -> io::Result<()> {
    stream.set_nodelay(true)?;
    stream.set_read_timeout(Some(NETWORK_POLL_INTERVAL))?;
    write_json_line(
        &mut stream,
        &json!({
            "ok": true,
            "type": "hello",
            "protocol": PROTOCOL_VERSION,
            "max_command_timeout_ms": duration_milliseconds(maximum_timeout),
        }),
    )?;

    let mut reader = stream.try_clone()?;
    let mut pending = Vec::new();
    let mut input = [0_u8; 1024];
    while running.load(Ordering::SeqCst) {
        let read = match reader.read(&mut input) {
            Ok(read) => read,
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                continue;
            }
            Err(error) => return Err(error),
        };
        if read == 0 {
            return Ok(());
        }
        pending.extend_from_slice(&input[..read]);
        if pending.len() as u64 > MAX_CONTROL_LINE_BYTES && !pending.contains(&b'\n') {
            write_json_line(
                &mut stream,
                &error_response("control message is too long or is missing a newline"),
            )?;
            return Ok(());
        }

        while let Some(newline) = pending.iter().position(|byte| *byte == b'\n') {
            let line = pending.drain(..=newline).collect::<Vec<_>>();
            if line.len() as u64 > MAX_CONTROL_LINE_BYTES {
                write_json_line(&mut stream, &error_response("control message is too long"))?;
                return Ok(());
            }
            let response = match serde_json::from_slice::<ControlRequest>(&line) {
                Ok(request) => handle_request(request, motion, maximum_timeout),
                Err(error) => error_response(&format!("invalid request: {error}")),
            };
            write_json_line(&mut stream, &response)?;
        }
        if pending.len() as u64 > MAX_CONTROL_LINE_BYTES {
            write_json_line(&mut stream, &error_response("control message is too long"))?;
            return Ok(());
        }
    }
    Ok(())
}

fn handle_request(
    request: ControlRequest,
    motion: &Mutex<MotionState>,
    maximum_timeout: Duration,
) -> Value {
    match request {
        ControlRequest::Velocity {
            vx,
            vy,
            vtheta,
            timeout_ms,
        } => {
            if timeout_ms == 0 || Duration::from_millis(timeout_ms) > maximum_timeout {
                return error_response(&format!(
                    "timeout_ms must be between 1 and {}",
                    duration_milliseconds(maximum_timeout)
                ));
            }
            let requested = Velocity::new(vx, vy, vtheta);
            let command = match requested.to_scout_twist(MotionLimits::default()) {
                Ok(command) => command,
                Err(error) => return error_response(&error.to_string()),
            };
            let applied = [command.linear_y, -command.linear_x, command.angular_z];
            match motion.lock() {
                Ok(mut state) => {
                    state.command = command;
                    state.deadline = Some(Instant::now() + Duration::from_millis(timeout_ms));
                    json!({
                        "ok": true,
                        "type": "velocity",
                        "applied": applied,
                        "timeout_ms": timeout_ms,
                    })
                }
                Err(_) => error_response("motion state is unavailable"),
            }
        }
        ControlRequest::Stop => match motion.lock() {
            Ok(mut state) => {
                state.stop();
                json!({"ok": true, "type": "stop"})
            }
            Err(_) => error_response("motion state is unavailable"),
        },
        ControlRequest::Ping => json!({
            "ok": true,
            "type": "pong",
            "protocol": PROTOCOL_VERSION,
        }),
    }
}

fn duration_milliseconds(duration: Duration) -> u64 {
    u64::try_from(duration.as_millis()).unwrap_or(u64::MAX)
}

fn error_response(message: &str) -> Value {
    json!({"ok": false, "type": "error", "message": message})
}

fn write_json_line(stream: &mut TcpStream, value: &Value) -> io::Result<()> {
    serde_json::to_writer(&mut *stream, value).map_err(io::Error::other)?;
    stream.write_all(b"\n")?;
    stream.flush()
}

fn run_camera_listener(listener: TcpListener, frame: SharedFrame, running: Arc<AtomicBool>) {
    let clients = Arc::new(AtomicUsize::new(0));
    while running.load(Ordering::SeqCst) {
        match listener.accept() {
            Ok((stream, _)) => {
                if clients.fetch_add(1, Ordering::SeqCst) >= MAX_CAMERA_CLIENTS {
                    clients.fetch_sub(1, Ordering::SeqCst);
                    let _ = stream.shutdown(Shutdown::Both);
                    continue;
                }
                let frame = Arc::clone(&frame);
                let running = Arc::clone(&running);
                let worker_clients = Arc::clone(&clients);
                if thread::Builder::new()
                    .name("scout-bridge-camera-client".into())
                    .spawn(move || {
                        let _ = serve_camera_client(stream, &frame, &running);
                        worker_clients.fetch_sub(1, Ordering::SeqCst);
                    })
                    .is_err()
                {
                    // The client count was incremented before attempting to
                    // create its worker.
                    clients.fetch_sub(1, Ordering::SeqCst);
                }
            }
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                thread::sleep(NETWORK_POLL_INTERVAL);
            }
            Err(_) => thread::sleep(NETWORK_POLL_INTERVAL),
        }
    }
}

fn serve_camera_client(
    mut stream: TcpStream,
    shared: &SharedFrame,
    running: &AtomicBool,
) -> io::Result<()> {
    stream.set_nodelay(true)?;
    stream.set_read_timeout(Some(NETWORK_POLL_INTERVAL))?;
    stream.set_write_timeout(Some(CAMERA_WRITE_TIMEOUT))?;
    let (lock, available) = &**shared;
    let mut seen_generation = 0;

    while running.load(Ordering::SeqCst) {
        let mut request = [0_u8; 1];
        match stream.read_exact(&mut request) {
            Ok(()) if request[0] == 1 => {}
            Ok(()) => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "camera request must be byte 1",
                ));
            }
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                continue;
            }
            Err(error) => return Err(error),
        }

        let mut frame = lock
            .lock()
            .map_err(|_| io::Error::other("camera frame state is unavailable"))?;
        while frame.generation == seen_generation && running.load(Ordering::SeqCst) {
            let (next, _) = available
                .wait_timeout(frame, NETWORK_POLL_INTERVAL)
                .map_err(|_| io::Error::other("camera frame state is unavailable"))?;
            frame = next;
        }
        if !running.load(Ordering::SeqCst) {
            return Ok(());
        }
        let Some(jpeg) = frame.jpeg.clone() else {
            continue;
        };
        seen_generation = frame.generation;
        drop(frame);

        let length = u32::try_from(jpeg.len())
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "JPEG frame is too large"))?;
        stream.write_all(&length.to_be_bytes())?;
        stream.write_all(&jpeg)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read_json_line(stream: &mut TcpStream) -> Value {
        let mut line = Vec::new();
        loop {
            let mut byte = [0_u8; 1];
            stream.read_exact(&mut byte).unwrap();
            line.push(byte[0]);
            if byte[0] == b'\n' {
                return serde_json::from_slice(&line).unwrap();
            }
        }
    }

    #[test]
    fn motion_deadman_expires_to_zero() {
        let now = Instant::now();
        let mut state = MotionState {
            command: Velocity::new(0.1, 0.0, 0.2)
                .to_scout_twist(MotionLimits::default())
                .unwrap(),
            deadline: Some(now + Duration::from_millis(250)),
        };
        assert_ne!(state.current(now), ScoutTwist::zero());
        assert_eq!(
            state.current(now + Duration::from_millis(250)),
            ScoutTwist::zero()
        );
        assert!(state.deadline.is_none());
    }

    #[test]
    fn velocity_request_is_clamped_and_armed() {
        let state = Mutex::new(MotionState::default());
        let response = handle_request(
            ControlRequest::Velocity {
                vx: 10.0,
                vy: -10.0,
                vtheta: 10.0,
                timeout_ms: 200,
            },
            &state,
            Duration::from_secs(1),
        );
        assert_eq!(response["ok"], true);
        assert_eq!(response["applied"], json!([0.47, -0.2, 2.9]));
        assert!(state.lock().unwrap().deadline.is_some());
    }

    #[test]
    fn rejects_timeout_above_bridge_limit() {
        let state = Mutex::new(MotionState::default());
        let response = handle_request(
            ControlRequest::Velocity {
                vx: 0.1,
                vy: 0.0,
                vtheta: 0.0,
                timeout_ms: 1_001,
            },
            &state,
            Duration::from_secs(1),
        );
        assert_eq!(response["ok"], false);
        assert!(state.lock().unwrap().deadline.is_none());
    }

    #[test]
    fn serves_control_and_latest_camera_frame() {
        let running = Arc::new(AtomicBool::new(true));
        let bridge = IntegrationBridge::start(
            BridgeOptions {
                control_address: "127.0.0.1:0".parse().unwrap(),
                camera_address: "127.0.0.1:0".parse().unwrap(),
                maximum_command_timeout: Duration::from_secs(1),
            },
            Arc::clone(&running),
        )
        .unwrap();

        let mut control = TcpStream::connect(bridge.control_address()).unwrap();
        control
            .set_read_timeout(Some(Duration::from_secs(1)))
            .unwrap();
        assert_eq!(read_json_line(&mut control)["type"], "hello");
        control
            .write_all(
                b"{\"type\":\"velocity\",\"vx\":0.1,\"vy\":0.02,\"vtheta\":0.3,\"timeout_ms\":500}\n",
            )
            .unwrap();
        let response = read_json_line(&mut control);
        assert_eq!(response["ok"], true);
        assert_ne!(bridge.current_command(Instant::now()), ScoutTwist::zero());

        let jpeg = vec![0xff, 0xd8, 1, 2, 3, 0xff, 0xd9];
        bridge.frame_sink().publish(jpeg.clone());
        let mut camera = TcpStream::connect(bridge.camera_address()).unwrap();
        camera
            .set_read_timeout(Some(Duration::from_secs(1)))
            .unwrap();
        camera.write_all(&[1]).unwrap();
        let mut length = [0_u8; 4];
        camera.read_exact(&mut length).unwrap();
        assert_eq!(u32::from_be_bytes(length) as usize, jpeg.len());
        let mut received = vec![0_u8; jpeg.len()];
        camera.read_exact(&mut received).unwrap();
        assert_eq!(received, jpeg);

        bridge.shutdown();
    }
}
