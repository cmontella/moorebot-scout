import json
import socket
import struct
import threading
import unittest
from unittest.mock import Mock, patch

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


class ScoutClientGreetingTests(unittest.TestCase):
    def assert_greeting_failure(
        self,
        response: bytes | BaseException,
        exception_type: type[BaseException],
        message: str,
    ) -> BaseException:
        connection = Mock(spec=socket.socket)
        reader = Mock()
        connection.makefile.return_value = reader
        if isinstance(response, BaseException):
            reader.readline.side_effect = response
        else:
            reader.readline.return_value = response
        with patch(
            "moorebot_scout.client.socket.create_connection", return_value=connection
        ):
            # Keep the exception and its traceback alive so garbage collection
            # cannot hide a constructor that failed to close its resources.
            try:
                ScoutClient()
            except exception_type as error:
                failure = error
            else:
                self.fail("the invalid greeting was accepted")
        self.assertIn(message, str(failure))
        self.assertIsNotNone(failure.__traceback__)
        reader.close.assert_called_once_with()
        connection.close.assert_called_once_with()
        connection.sendall.assert_not_called()
        return failure

    def test_busy_greeting_closes_without_sending_stop(self) -> None:
        self.assert_greeting_failure(
            b'{"ok":false,"code":"busy",'
            b'"message":"another control client is already connected"}\n',
            BridgeError,
            "another control client is already connected",
        )

    def test_greeting_timeout_closes_without_sending_stop(self) -> None:
        timeout = TimeoutError("greeting timed out")
        failure = self.assert_greeting_failure(
            timeout, TimeoutError, "greeting timed out"
        )
        self.assertIs(failure, timeout)

    def test_invalid_greetings_close_without_sending_stop(self) -> None:
        cases = [
            (b"", "bridge closed the control connection"),
            (b"not json\n", "bridge returned invalid JSON"),
            (b"[]\n", "non-object JSON response"),
            (b'{"ok":true,"type":"hello","protocol":2}\n', "unsupported"),
            (b'{"ok":true,"type":"hello","protocol":1}\n', "invalid command timeout"),
        ]
        for response, message in cases:
            with self.subTest(response=response):
                self.assert_greeting_failure(response, BridgeError, message)

    def test_invalid_command_timeouts_close_without_sending_stop(self) -> None:
        for timeout in (None, "bad", {}, 0, -1, float("nan"), float("inf")):
            with self.subTest(timeout=timeout):
                response = json.dumps(
                    {
                        "ok": True,
                        "type": "hello",
                        "protocol": 1,
                        "max_command_timeout_ms": timeout,
                    }
                ).encode() + b"\n"
                self.assert_greeting_failure(
                    response, BridgeError, "invalid command timeout"
                )


if __name__ == "__main__":
    unittest.main()
