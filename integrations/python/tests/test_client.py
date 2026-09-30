import json
import socket
import struct
import threading
import unittest

from moorebot_scout import BridgeError, ScoutClient

JPEG = b"\xff\xd8test-frame\xff\xd9"


class FakeBridge:
    def __init__(self) -> None:
        self.control = socket.socket()
        self.control.bind(("127.0.0.1", 0))
        self.control.listen()
        self.camera = socket.socket()
        self.camera.bind(("127.0.0.1", 0))
        self.camera.listen()
        self.control_port = self.control.getsockname()[1]
        self.camera_port = self.camera.getsockname()[1]
        self.requests: list[dict[str, object]] = []
        self.threads = [
            threading.Thread(target=self._control, daemon=True),
            threading.Thread(target=self._camera, daemon=True),
        ]
        for thread in self.threads:
            thread.start()

    def _control(self) -> None:
        try:
            connection, _ = self.control.accept()
        except OSError:
            return
        with connection, connection.makefile("rb") as reader:
            connection.sendall(
                b'{"ok":true,"type":"hello","protocol":1,'
                b'"max_command_timeout_ms":1000}\n'
            )
            for line in reader:
                request = json.loads(line)
                self.requests.append(request)
                if request["type"] == "velocity":
                    response = {
                        "ok": True,
                        "type": "velocity",
                        "applied": [request["vx"], request["vy"], request["vtheta"]],
                    }
                else:
                    response = {"ok": True, "type": request["type"]}
                    if request["type"] == "ping":
                        response["type"] = "pong"
                connection.sendall(json.dumps(response).encode() + b"\n")

    def _camera(self) -> None:
        try:
            connection, _ = self.camera.accept()
        except OSError:
            return
        with connection:
            while connection.recv(1) == b"\x01":
                connection.sendall(struct.pack(">I", len(JPEG)) + JPEG)

    def close(self) -> None:
        self.control.close()
        self.camera.close()
        for thread in self.threads:
            thread.join(timeout=1.0)


class ScoutClientTests(unittest.TestCase):
    def setUp(self) -> None:
        self.bridge = FakeBridge()
        self.client = ScoutClient(
            control_port=self.bridge.control_port,
            camera_port=self.bridge.camera_port,
        )

    def tearDown(self) -> None:
        self.client.close()
        self.bridge.close()

    def test_velocity_and_camera(self) -> None:
        applied = self.client.set_velocity(0.1, -0.02, 0.3, timeout=0.4)
        self.assertEqual(applied, (0.1, -0.02, 0.3))
        self.assertEqual(self.client.read_jpeg(), JPEG)
        self.assertEqual(self.client.read_jpeg(), JPEG)

    def test_rejects_timeout_before_sending(self) -> None:
        with self.assertRaises(ValueError):
            self.client.set_velocity(0.1, 0.0, 0.0, timeout=2.0)

    def test_closed_client_rejects_requests(self) -> None:
        self.client.close()
        with self.assertRaises(BridgeError):
            self.client.ping()


if __name__ == "__main__":
    unittest.main()
