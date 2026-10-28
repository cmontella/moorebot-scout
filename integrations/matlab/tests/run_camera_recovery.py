"""Run MATLAB camera recovery tests against ephemeral localhost mock sockets.

Requires Python 3.10+ and MATLAB with tcpclient. No robot or running bridge is
used. Invoke with `python run_camera_recovery.py --matlab /path/to/matlab`, or
omit --matlab when matlab is on PATH. All fixtures use the standard library.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import struct
import subprocess
import threading

JPEG = b"\xff\xd8\xff\xd9"
FRAMED_JPEG = struct.pack(">I", len(JPEG)) + JPEG
FAILURES = (b"\x00\x00", struct.pack(">I", 6) + b"\xff\xd8", b"\x00" * 4,
            struct.pack(">I", 4) + b"bad!")


def listener() -> socket.socket:
    result = socket.socket()
    result.bind(("127.0.0.1", 0))
    result.listen(4)
    result.settimeout(180)
    return result


def control_server(server: socket.socket) -> None:
    connection, _ = server.accept()
    with connection:
        connection.settimeout(15)
        connection.sendall(json.dumps({"ok": True, "type": "hello", "protocol": 1,
                                       "max_command_timeout_ms": 1000}).encode() + b"\n")
        reader = connection.makefile("rb")
        with reader:
            commands = []
            while line := reader.readline():
                command = json.loads(line)["type"]
                assert command in ("ping", "stop"), command
                commands.append(command)
                response = "pong" if command == "ping" else "stop"
                connection.sendall(json.dumps({"ok": True, "type": response}).encode() + b"\n")
        assert commands == ["ping"] * 8 + ["stop"], commands


def camera_server(server: socket.socket) -> None:
    # The first connection fails. Each replacement serves one good frame and
    # then the next failure, verifying success keeps the same camera socket.
    for index in range(len(FAILURES) + 1):
        connection, _ = server.accept()
        with connection:
            connection.settimeout(15)
            if index:
                assert connection.recv(1) == b"\x01"
                connection.sendall(FRAMED_JPEG)
            if index < len(FAILURES):
                assert connection.recv(1) == b"\x01"
                connection.sendall(FAILURES[index])
            # Partial frames remain connected until the client times out.
            # It must close here, not retry against the damaged byte stream.
            assert connection.recv(1) == b"", "Failed camera socket was reused"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matlab", default=shutil.which("matlab"))
    args = parser.parse_args()
    if not args.matlab:
        parser.error("MATLAB is not on PATH; pass --matlab with its executable path")
    failures: list[BaseException] = []

    def checked(target, server):
        try:
            target(server)
        except BaseException as error:
            failures.append(error)

    with listener() as control, listener() as camera:
        ports = (control.getsockname()[1], camera.getsockname()[1])
        assert not set(ports) & {43000, 43001}, "Mock must not use normal bridge ports"
        workers = [threading.Thread(target=checked, args=(target, server), daemon=True)
                   for target, server in ((control_server, control), (camera_server, camera))]
        for worker in workers:
            worker.start()
        test_path = str(Path(__file__).resolve().parent).replace("'", "''")
        expression = f"addpath('{test_path}'); testCameraRecovery({ports[0]}, {ports[1]})"
        options = {"creationflags": subprocess.CREATE_NO_WINDOW} if os.name == "nt" else {"start_new_session": True}
        print(f"Testing camera recovery on mock ports {ports}", flush=True)
        process = subprocess.Popen([args.matlab, "-batch", expression], **options)
        try:
            returncode = process.wait(timeout=180)
        except subprocess.TimeoutExpired:
            # Terminate only the MATLAB process tree created by this runner.
            if os.name == "nt":
                subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                               check=False, stdout=subprocess.DEVNULL)
            else:
                os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=10)
            raise
        if returncode:
            raise RuntimeError(f"MATLAB exited with status {returncode}")
        for worker in workers:
            worker.join(timeout=5)
            assert not worker.is_alive(), "Mock socket did not close after MATLAB returned"
        if failures:
            raise RuntimeError(f"Mock bridge assertions failed: {failures!r}")
    print("PASS: all camera sockets closed; the original control session remained usable")


if __name__ == "__main__":
    main()
