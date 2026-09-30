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
addpath("integrations/matlab")
scout = MoorebotScout();
cleanup = onCleanup(@() delete(scout));

while true
    frame = scout.readImage();
    % Segment or analyze frame here.
    scout.setVelocity(0.08, 0.0, 0.0, 0.5);
end
```

The ready-to-run live-camera example is in `integrations/matlab/examples`:

```matlab
run("integrations/matlab/examples/liveCamera.m")
```

`continuousCommands.m` shows the same command-refresh pattern. Its first run
must also be made with the wheels clear.

`readImage` decodes JPEG bytes in memory with the Java image library bundled
with desktop MATLAB, so it does not write a temporary image file for every
frame. The sample segmentation code uses ordinary MATLAB array operations.

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

The control port starts with one JSON `hello` line. Requests and responses are
newline-delimited JSON. Protocol version 1 accepts:

```json
{"type":"velocity","vx":0.08,"vy":0.0,"vtheta":-0.4,"timeout_ms":500}
{"type":"stop"}
{"type":"ping"}
```

For each camera frame, send the byte `0x01` to the camera port. The reply is a
four-byte unsigned big-endian JPEG length followed by exactly that many JPEG
bytes. The supplied clients implement these details.
