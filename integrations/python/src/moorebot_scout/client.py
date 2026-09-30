"""Persistent command and camera client for ``moorebot-scout bridge``."""

from __future__ import annotations

import json
import math
import socket
import struct
import threading
from collections.abc import Iterator
from types import TracebackType
from typing import Any, Self

_MAX_CONTROL_LINE = 4096
_MAX_JPEG_BYTES = 16 * 1024 * 1024


class BridgeError(RuntimeError):
    """The local bridge rejected a request or returned invalid data."""


class ScoutClient:
    """Connect Python to one running ``moorebot-scout bridge`` process.

    The control and camera sockets are independent, so image processing cannot
    block command messages. Call :meth:`set_velocity` repeatedly from a control
    loop. If the command deadline passes before the next update, the Rust
    bridge commands zero velocity.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        control_port: int = 43000,
        camera_port: int = 43001,
        *,
        timeout: float = 5.0,
    ) -> None:
        self._host = host
        self._camera_port = camera_port
        self._timeout = timeout
        self._control = socket.create_connection((host, control_port), timeout)
        self._control.settimeout(timeout)
        self._control_reader = self._control.makefile("rb")
        self._control_lock = threading.Lock()
        self._camera: socket.socket | None = None
        self._closed = False

        hello = self._read_response()
        if hello.get("type") != "hello" or hello.get("protocol") != 1:
            self.close(stop=False)
            raise BridgeError(f"unsupported bridge greeting: {hello!r}")
        self.max_command_timeout = float(hello["max_command_timeout_ms"]) / 1000.0

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()

    def close(self, *, stop: bool = True) -> None:
        """Stop the robot if possible and close both sockets."""

        if self._closed:
            return
        if stop:
            try:
                self.stop()
            except (BridgeError, OSError):
                pass
        self._closed = True
        if self._camera is not None:
            self._camera.close()
            self._camera = None
        self._control_reader.close()
        self._control.close()

    def ping(self) -> None:
        """Check that the control bridge is responsive."""

        response = self._request({"type": "ping"})
        if response.get("type") != "pong":
            raise BridgeError(f"unexpected ping response: {response!r}")

    def set_velocity(
        self,
        vx: float,
        vy: float,
        vtheta: float,
        *,
        timeout: float = 0.5,
    ) -> tuple[float, float, float]:
        """Set ``[vx, vy, vtheta]`` until refreshed or ``timeout`` expires.

        ``vx`` is forward m/s, ``vy`` is leftward m/s, and ``vtheta`` is
        counter-clockwise rad/s. The returned tuple contains the values after
        the Rust driver applies the Scout's safety limits.
        """

        values = (float(vx), float(vy), float(vtheta), float(timeout))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("velocity and timeout values must be finite")
        if timeout <= 0.0 or timeout > self.max_command_timeout:
            raise ValueError(
                f"timeout must be greater than zero and no more than "
                f"{self.max_command_timeout:g} seconds"
            )
        timeout_ms = max(1, round(timeout * 1000.0))
        response = self._request(
            {
                "type": "velocity",
                "vx": vx,
                "vy": vy,
                "vtheta": vtheta,
                "timeout_ms": timeout_ms,
            }
        )
        applied = response.get("applied")
        if not isinstance(applied, list) or len(applied) != 3:
            raise BridgeError(f"invalid velocity response: {response!r}")
        return (float(applied[0]), float(applied[1]), float(applied[2]))

    def stop(self) -> None:
        """Immediately replace the active command with zero velocity."""

        response = self._request({"type": "stop"})
        if response.get("type") != "stop":
            raise BridgeError(f"unexpected stop response: {response!r}")

    def read_jpeg(self) -> bytes:
        """Request and return the newest available camera frame as JPEG bytes."""

        if self._closed:
            raise BridgeError("client is closed")
        if self._camera is None:
            self._camera = socket.create_connection(
                (self._host, self._camera_port), self._timeout
            )
            self._camera.settimeout(self._timeout)
        self._camera.sendall(b"\x01")
        (length,) = struct.unpack(">I", self._receive_exact(self._camera, 4))
        if length == 0 or length > _MAX_JPEG_BYTES:
            raise BridgeError(f"invalid JPEG length from bridge: {length}")
        jpeg = self._receive_exact(self._camera, length)
        if not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
            raise BridgeError("bridge returned data without JPEG markers")
        return jpeg

    def jpeg_frames(self) -> Iterator[bytes]:
        """Yield newly requested JPEG frames until the caller stops iteration."""

        while not self._closed:
            yield self.read_jpeg()

    def read_image(self) -> Any:
        """Return the newest frame as an OpenCV BGR array.

        Install the optional dependencies with
        ``pip install 'moorebot-scout-client[vision]'``.
        """

        try:
            import cv2
            import numpy as np
        except ImportError as error:
            raise BridgeError(
                "read_image requires NumPy and OpenCV; install the 'vision' extra"
            ) from error
        encoded = np.frombuffer(self.read_jpeg(), dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            raise BridgeError("OpenCV could not decode the Scout JPEG")
        return image

    def _request(self, request: dict[str, object]) -> dict[str, Any]:
        if self._closed:
            raise BridgeError("client is closed")
        encoded = json.dumps(request, separators=(",", ":")).encode("ascii") + b"\n"
        if len(encoded) > _MAX_CONTROL_LINE:
            raise ValueError("control request is too large")
        with self._control_lock:
            self._control.sendall(encoded)
            response = self._read_response()
        if response.get("ok") is not True:
            raise BridgeError(str(response.get("message", "bridge rejected request")))
        return response

    def _read_response(self) -> dict[str, Any]:
        line = self._control_reader.readline(_MAX_CONTROL_LINE + 1)
        if not line:
            raise BridgeError("bridge closed the control connection")
        if len(line) > _MAX_CONTROL_LINE or not line.endswith(b"\n"):
            raise BridgeError("invalid control response from bridge")
        try:
            response = json.loads(line)
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise BridgeError("bridge returned invalid JSON") from error
        if not isinstance(response, dict):
            raise BridgeError("bridge returned a non-object JSON response")
        if response.get("ok") is not True:
            raise BridgeError(str(response.get("message", "bridge reported an error")))
        return response

    @staticmethod
    def _receive_exact(connection: socket.socket, length: int) -> bytes:
        output = bytearray(length)
        view = memoryview(output)
        received = 0
        while received < length:
            count = connection.recv_into(view[received:])
            if count == 0:
                raise BridgeError("bridge closed the camera connection")
            received += count
        return bytes(output)
