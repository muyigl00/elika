"""Bounded line protocol for the persistent Gazebo transport helper."""

from __future__ import annotations

import select
import subprocess
import time


class GazeboServiceClient:
    def __init__(self, executable, environment, log) -> None:
        self.process = subprocess.Popen(
            [str(executable)], env=environment, stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=log, bufsize=0, start_new_session=True
        )
        try:
            if self._read() != "ready":
                raise RuntimeError("Gazebo service helper did not initialize")
        except BaseException:
            self.close()
            raise

    def _read(self) -> str:
        deadline = time.monotonic() + 8.0
        data = bytearray()
        while time.monotonic() < deadline:
            timeout = max(0.0, deadline - time.monotonic())
            if not select.select([self.process.stdout], [], [], timeout)[0]:
                break
            byte = self.process.stdout.read(1)
            if not byte:
                raise RuntimeError("Gazebo service helper exited unexpectedly")
            if byte == b"\n":
                return data.decode()
            data.extend(byte)
        raise RuntimeError("Gazebo service helper protocol timed out")

    def exchange(self, line: str) -> str:
        if "\n" in line or "\r" in line:
            raise ValueError("Service protocol requires one line")
        self.process.stdin.write((line + "\n").encode())
        self.process.stdin.flush()
        return self._read()

    def services(self) -> list[str]:
        reply = self.exchange("list").split("\t")
        if not reply or reply[0] != "services":
            raise RuntimeError(f"Unexpected service-list reply: {reply}")
        return reply[1:]

    def call(self, endpoint: str, kind: str, request: str, attempts: int = 3) -> None:
        for attempt in range(1, attempts + 1):
            reply = self.exchange(f"{endpoint}\t{kind}\t{request}")
            if reply == "ok":
                return
            if reply != "timeout" or attempt == attempts:
                raise RuntimeError(f"{endpoint} failed on attempt {attempt}: {reply}")
            time.sleep(0.1)

    def close(self) -> None:
        if self.process.stdin and not self.process.stdin.closed:
            self.process.stdin.close()
        try:
            self.process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2.0)
        if self.process.stdout and not self.process.stdout.closed:
            self.process.stdout.close()

