"""Serial framing and board connection primitives for HIL tooling."""

from __future__ import annotations

import re
import time

import serial

BAUDRATE = 115200
_FRAME_RE = re.compile(r"^\$(.*)\*([0-9A-Fa-f]{1,2})$")


class HilError(RuntimeError):
    """Base error raised by the HIL host tools."""


class ProtocolError(HilError):
    """An invalid frame or firmware error response."""


def xor_checksum(body: str) -> int:
    checksum = 0
    for byte in body.encode("ascii"):
        checksum ^= byte
    return checksum


def build_frame(body: str) -> bytes:
    """Build the firmware's NMEA-like command frame."""
    return f"${body}*{xor_checksum(body):02X}\r\n".encode("ascii")


def parse_frame(line: str) -> str:
    """Validate a response frame and return its body."""
    match = _FRAME_RE.fullmatch(line.strip())
    if match is None:
        raise ProtocolError(f"Malformed response frame: {line!r}")
    body, received = match.groups()
    expected = xor_checksum(body)
    if int(received, 16) != expected:
        raise ProtocolError(
            f"Checksum mismatch for {body!r}: received {received}, expected {expected:02X}"
        )
    return body


class BoardConnection:
    """Non-blocking, line-oriented connection to one board's UART0."""

    def __init__(self, port: str, role: str) -> None:
        self.port = port
        self.role = role
        self._serial: serial.Serial | None = None
        self._rx = bytearray()

    def open(self) -> None:
        try:
            self._serial = serial.Serial(
                self.port,
                BAUDRATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
                write_timeout=1.0,
            )
        except serial.SerialException as exc:
            raise HilError(f"Cannot open {self.role} port {self.port}: {exc}") from exc
        time.sleep(0.1)
        self._serial.reset_input_buffer()

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None
        self._rx.clear()

    def __enter__(self) -> BoardConnection:
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def read_lines(self) -> list[str]:
        if self._serial is None:
            raise HilError(f"{self.role} port is not open")

        waiting = self._serial.in_waiting
        if waiting:
            self._rx.extend(self._serial.read(waiting))

        lines: list[str] = []
        while b"\n" in self._rx:
            raw, _, remainder = self._rx.partition(b"\n")
            self._rx = bytearray(remainder)
            line = raw.rstrip(b"\r").decode("ascii", errors="replace").strip()
            if line:
                lines.append(line)
        return lines

    def command(self, body: str, timeout_s: float = 2.0) -> str:
        if self._serial is None:
            raise HilError(f"{self.role} port is not open")

        expected = _expected_response_prefix(body)
        self._serial.write(build_frame(body))
        self._serial.flush()
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            for line in self.read_lines():
                if not line.startswith("$"):
                    continue
                response = parse_frame(line)
                if response.startswith("ERR,"):
                    raise ProtocolError(
                        f"{self.role} board rejected {body!r}: {response}"
                    )
                if response.startswith(expected):
                    return response
            time.sleep(0.005)

        raise HilError(
            f"No response from {self.role} board on {self.port} for {body!r}"
        )


def _expected_response_prefix(command: str) -> str:
    if command in ("MODELS", "CTRLS", "TRAJS"):
        return command
    if command.startswith(("LOG", "UTIL")):
        return f"OK,{command.split(',', 1)[0].rstrip('?')}"
    return f"OK,{command.split(',', 1)[0]}"


def detect_board(port: str, timeout_s: float = 2.0) -> str:
    """Identify a board from telemetry, preserving its original log period."""
    deadline = time.monotonic() + timeout_s
    with BoardConnection(port, "probe") as board:
        response = board.command("LOG?", timeout_s=max(0.0, deadline - time.monotonic()))
        fields = response.split(",")
        if (
            len(fields) != 3 or fields[:2] != ["OK", "LOG"]
            or not fields[2].isdigit()
            or (int(fields[2]) != 0 and not 10 <= int(fields[2]) <= 5000)
        ):
            raise ProtocolError(f"Invalid logging reply on {port}: {response}")
        period = int(fields[2])
        # Reserve time to restore logging within the per-port deadline.
        probe_deadline = deadline - 0.25
        try:
            board.command("LOG,50", timeout_s=max(0.0, probe_deadline - time.monotonic()))
            while time.monotonic() < probe_deadline:
                for line in board.read_lines():
                    if line.startswith("[MODEL] t="):
                        return "model"
                    if line.startswith("[CTRL] t="):
                        return "controller"
                time.sleep(0.005)
        finally:
            board.command(
                f"LOG,{period}", timeout_s=max(0.0, deadline - time.monotonic())
            )
    raise HilError(f"No model/controller telemetry on {port} within {timeout_s:g} s")