# Python and MATLAB integration

The `moorebot-scout bridge` command keeps one ROS connection to the Scout open
and presents a small local interface to Python and MATLAB. The language clients
do not need ROS or the Scout's custom ROS messages.

```text
Python or MATLAB
  |-- newline-delimited JSON velocity commands --> 127.0.0.1:43000
  `-- request one latest JPEG frame -----------> 127.0.0.1:43001
                 moorebot-scout bridge
                       | TCPROS
                       v
                 Moorebot Scout
```

The two local ports are deliberately separate. Image processing can be slow
without delaying velocity updates. A camera client requests each frame, and the
bridge returns the newest frame available rather than building an old-frame
backlog.

## 1. Start the bridge

Connect the computer to the Scout, open a terminal in the extracted release,
and leave this running:

```text
# Windows PowerShell
./moorebot-scout.exe bridge

# macOS or Linux
./moorebot-scout bridge
```

If the Scout is on a router rather than its own Wi-Fi, put the global option
before `bridge`:

```text
./moorebot-scout --master http://192.168.1.73:11311 bridge
```

The bridge listens only on this computer (`127.0.0.1`). It continuously
publishes the last valid command at 40 Hz. Every command has a short deadline;
if the client disconnects, crashes, or fails to refresh it, the bridge switches
to zero velocity. Ctrl-C in the bridge terminal also sends a stop.

Wait for `Integration bridge ready. Commands: 127.0.0.1:43000 | camera:
127.0.0.1:43001` before connecting a client. Only one MATLAB or Python client
may own control at a time. An additional client receives an immediate
`Another client already controls the bridge` error. Use the same client for
images and movement, or close it before starting another example.

## 2. Use Python

Python 3.10 or newer is recommended. From the extracted release directory:

```text
python -m pip install ./integrations/python
```

For the live-camera example, install the optional OpenCV and NumPy
dependencies:

```text
python -m pip install "./integrations/python[vision]"
python integrations/python/examples/live_camera.py
```

To see the command-refresh pattern separately, first put the Scout on a stand
with all wheels clear, then run
`python integrations/python/examples/continuous_commands.py`.

The core package itself uses only the Python standard library. A control loop
looks like this:

```python
from moorebot_scout import ScoutClient

with ScoutClient() as scout:
    while True:
        jpeg = scout.read_jpeg()       # newest frame, as bytes
        # image = scout.read_image()   # decoded OpenCV BGR array
        scout.set_velocity(0.08, 0.0, 0.0, timeout=0.5)
```

Always use a context manager or call `stop()` in `finally`. The bridge's
deadman is a backup, not a replacement for an explicit stop.

## 3. Use MATLAB

The MATLAB client uses `tcpclient`, introduced in R2020b. In MATLAB, change to
the extracted release directory and run:

```matlab
function runScout
    addpath("integrations/matlab")
    scout = MoorebotScout();
    cleanup = onCleanup(@() delete(scout));
    while true
        frame = scout.readImage();
        % Segment or analyze frame here.
        scout.setVelocity(0.08, 0.0, 0.0, 0.5);
    end
end
```

Save this in `runScout.m`. A function scope makes `onCleanup` close the client
when the function returns, errors, or is interrupted. In a script or the
Command Window, `scout` and `cleanup` remain in the workspace. Before replacing
an existing connection, run `delete(scout); clear scout cleanup`. Merely
assigning `scout = MoorebotScout()` again constructs the new connection while
the previous one is still open.

The ready-to-run live-camera example is in `integrations/matlab/examples`:

```matlab
run("integrations/matlab/examples/liveCamera.m")
```

`continuousCommands.m` shows the same command-refresh pattern. Its first run
must also be made with the wheels clear.

`readImage` decodes JPEG bytes in memory with the Java image library bundled
with desktop MATLAB, so it does not write a temporary image file for every
frame. The sample segmentation code uses ordinary MATLAB array operations.

### Diagnose connection and camera delays

If the constructor reports no bridge greeting, confirm the Rust terminal has
reached the ready message and use its **commands** port (normally 43000), not
the camera port (43001). Close other Scout clients. Older bridge versions
leave a second client waiting silently; update the executable together with
the integration files. If a fresh bridge still fails, capture its startup
output and the MATLAB connection call instead of increasing the timeout.

With an existing, connected `scout`, check control responsiveness without
moving the robot and then measure camera receipt and decoding separately:

```matlab
tic; scout.ping(); toc
samples = measureScoutCamera(scout, 30);
```

The timing helper sends no movement commands and opens no extra control
connection. Frame receipt includes waiting for Scout to publish the next
image; it is not pure network latency. The reported throughput excludes
display and image analysis. A slow camera does not block the bridge's
control socket, but a MATLAB loop that waits for each image before sending
velocity still runs at the image-processing rate. If that interval exceeds
the requested command timeout, the bridge intentionally stops movement.

## Coordinate convention

Both clients use `[vx, vy, vtheta]`:

| Value | Positive direction | Unit |
|---|---|---|
| `vx` | forward | m/s |
| `vy` | left strafe | m/s |
| `vtheta` | counter-clockwise turn | rad/s |

The Rust bridge clamps values to the driver's Scout limits. A command's
`timeout` is how long the bridge may keep using it; continuous control loops
should send a fresh command for every processed frame.

## Local protocol

The control port starts with one JSON `hello` line for its sole owner, or an
error line with `code: "busy"` if another owner is connected. Rejection never
changes the existing owner's movement command. Requests and responses are
newline-delimited JSON. Protocol version 1 accepts:

```json
{"type":"velocity","vx":0.08,"vy":0.0,"vtheta":-0.4,"timeout_ms":500}
{"type":"stop"}
{"type":"ping"}
```

For each camera frame, send the byte `0x01` to the camera port. The reply is a
four-byte unsigned big-endian JPEG length followed by exactly that many JPEG
bytes. The supplied clients implement these details.
