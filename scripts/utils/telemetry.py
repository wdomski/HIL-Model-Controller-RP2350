"""Experiment configuration, telemetry parsing, capture, and CSV output."""

from __future__ import annotations

import csv
import math
import re
import statistics
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

from .pairs import SUPPORTED_PAIRS, TRAJECTORIES, PairSpec
from .protocol import BoardConnection, HilError, ProtocolError

_TELEMETRY_RE = re.compile(r"^\[(CTRL|MODEL)\]\s+t=(-?\d+)\s+(.*)$")
_SIGNAL_RE = re.compile(r"([A-Za-z][A-Za-z0-9_]*)=([^\s]+)")


@dataclass(frozen=True)
class ExperimentConfig:
    model_port: str = "COM6"
    controller_port: str = "COM8"
    model: str = "Mass-Spring-Damper"
    controller: str = "PID"
    trajectories: tuple[str, ...] = ("sin",)
    duration_s: float | None = 20.0
    log_period_ms: int = 50


@dataclass(frozen=True)
class TelemetrySample:
    source: str
    host_time_s: float
    device_time_ms: int
    signals: dict[str, tuple[float, ...]]


@dataclass
class ExperimentResult:
    config: ExperimentConfig
    started_at: datetime
    samples: list[TelemetrySample]


StatusCallback = Callable[[str], None]


def parse_telemetry(line: str, host_time_s: float) -> TelemetrySample | None:
    """Parse one controller/model telemetry line; ignore unrelated text."""
    match = _TELEMETRY_RE.fullmatch(line.strip())
    if match is None:
        return None

    source, device_time, payload = match.groups()
    signals: dict[str, tuple[float, ...]] = {}
    for name, values_text in _SIGNAL_RE.findall(payload):
        try:
            signals[name] = tuple(float(value) for value in values_text.split(","))
        except ValueError as exc:
            raise ProtocolError(f"Invalid telemetry values in {line!r}") from exc

    return TelemetrySample(
        source=source,
        host_time_s=host_time_s,
        device_time_ms=int(device_time),
        signals=signals,
    )


def pair_for(config: ExperimentConfig) -> PairSpec:
    try:
        pair = SUPPORTED_PAIRS[(config.model, config.controller)]
    except KeyError as exc:
        supported = ", ".join(f"{model} + {ctrl}" for model, ctrl in SUPPORTED_PAIRS)
        raise HilError(
            f"Unsupported model/controller pair {config.model} + {config.controller}. "
            f"Supported: {supported}"
        ) from exc

    if config.trajectories and len(config.trajectories) != len(pair.setpoint_channels):
        raise HilError(
            f"{pair.model} + {pair.controller} requires "
            f"{len(pair.setpoint_channels)} trajectory selection(s)"
        )
    if not config.trajectories and len(pair.default_setpoints) != len(
        pair.setpoint_channels
    ):
        raise HilError(
            f"{pair.model} + {pair.controller} requires "
            f"{len(pair.setpoint_channels)} trajectory selection(s)"
        )
    unknown = [name for name in config.trajectories if name not in TRAJECTORIES]
    if unknown:
        raise HilError(f"Unknown trajectory: {', '.join(unknown)}")
    if config.duration_s is not None and (
        not math.isfinite(config.duration_s) or config.duration_s <= 0
    ):
        raise HilError("Duration must be finite and greater than zero")
    if config.log_period_ms < 10 or config.log_period_ms > 5000:
        raise HilError("Log period must be between 10 and 5000 ms")
    return pair


def run_experiment(
    config: ExperimentConfig,
    status: StatusCallback | None = None,
    stop_event: threading.Event | None = None,
    on_sample: Callable[[TelemetrySample], None] | None = None,
) -> ExperimentResult:
    """Capture telemetry; duration_s=None runs until stop_event is set."""
    pair = pair_for(config)
    if config.duration_s is None and stop_event is None:
        raise HilError("Indefinite capture requires a stop event")
    report = status or (lambda _message: None)
    samples: list[TelemetrySample] = []
    started_at = datetime.now(timezone.utc)

    with BoardConnection(config.model_port, "model") as model_board, BoardConnection(
        config.controller_port, "controller"
    ) as controller_board:
        report(f"Connected model {config.model_port} and controller {config.controller_port}")
        model_board.command("LOG,0")
        controller_board.command("LOG,0")

        model_reply = model_board.command(f"MODEL,{pair.model_id}")
        if not model_reply.endswith(f":{pair.model}"):
            raise ProtocolError(f"Unexpected model selection reply: {model_reply}")

        controller_reply = controller_board.command(f"CTRL,{pair.controller_id}")
        if not controller_reply.endswith(f":{pair.controller}"):
            raise ProtocolError(f"Unexpected controller selection reply: {controller_reply}")

        if config.trajectories:
            for channel, trajectory in zip(pair.setpoint_channels, config.trajectories):
                controller_board.command(f"SPTRAJ,{channel},{TRAJECTORIES[trajectory]}")
        else:
            for channel, setpoint in zip(
                pair.setpoint_channels, pair.default_setpoints
            ):
                controller_board.command(f"SP,{channel},{setpoint:g}")

        controller_board.command("RSTALL")
        time.sleep(0.2)
        model_board.command("UTIL,1")
        controller_board.command("UTIL,1")
        model_board.command(f"LOG,{config.log_period_ms}")
        controller_board.command(f"LOG,{config.log_period_ms}")
        duration = (
            "until stopped" if config.duration_s is None
            else f"for {config.duration_s:g} s"
        )
        report(f"Running {pair.model} + {pair.controller} {duration}")

        capture_start = time.monotonic()
        next_status = capture_start + 1.0
        try:
            while (
                config.duration_s is None
                or time.monotonic() - capture_start < config.duration_s
            ):
                if stop_event is not None and stop_event.is_set():
                    report("Capture stopped by user")
                    break

                now = time.monotonic()
                elapsed = now - capture_start
                for board in (model_board, controller_board):
                    for line in board.read_lines():
                        sample = parse_telemetry(line, elapsed)
                        if sample is not None:
                            samples.append(sample)
                            if on_sample is not None:
                                on_sample(sample)

                if now >= next_status:
                    report(f"Captured {len(samples)} samples ({elapsed:.1f} s)")
                    next_status = now + 1.0
                time.sleep(0.005)
        finally:
            for board in (model_board, controller_board):
                for command in ("LOG,0", "UTIL,0"):
                    try:
                        board.command(command)
                    except HilError:
                        pass

    if not any(sample.source == "CTRL" for sample in samples):
        raise HilError("No controller telemetry was received")
    if not any(sample.source == "MODEL" for sample in samples):
        raise HilError("No model telemetry was received")

    report(f"Capture complete: {len(samples)} samples")
    return ExperimentResult(config=config, started_at=started_at, samples=samples)


def write_csv(result: ExperimentResult, path: str | Path) -> Path:
    """Write experiment metadata followed by long-format telemetry."""
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["summary", "value"])
        writer.writerow(["started_at_utc", result.started_at.isoformat()])
        writer.writerow(["model", result.config.model])
        writer.writerow(["controller", result.config.controller])
        writer.writerow(["trajectories", ",".join(result.config.trajectories)])
        writer.writerow([
            "setpoints",
            ",".join(
                f"{value:g}"
                for value in (
                    () if result.config.trajectories
                    else pair_for(result.config).default_setpoints
                )
            ),
        ])
        writer.writerow(["model_port", result.config.model_port])
        writer.writerow(["controller_port", result.config.controller_port])
        writer.writerow(["sampling_period_ms", result.config.log_period_ms])
        writer.writerow([
            "duration_s",
            "unlimited" if result.config.duration_s is None
            else f"{result.config.duration_s:g}",
        ])
        for source, label in (("MODEL", "model"), ("CTRL", "controller")):
            values = [
                sample.signals["util"][0]
                for sample in result.samples
                if sample.source == source and sample.signals.get("util")
            ]
            if values:
                writer.writerow([
                    f"{label}_utilization_average_percent",
                    f"{statistics.fmean(values):.9g}",
                ])
                writer.writerow([
                    f"{label}_utilization_min_percent", f"{min(values):.9g}"
                ])
                writer.writerow([
                    f"{label}_utilization_max_percent", f"{max(values):.9g}"
                ])
                writer.writerow([
                    f"{label}_utilization_median_percent",
                    f"{statistics.median(values):.9g}",
                ])
        writer.writerow([])
        writer.writerow(
            [
                "source", "host_time_s", "device_time_ms", "signal", "channel",
                "value",
            ]
        )
        for sample in result.samples:
            for signal, values in sample.signals.items():
                for channel, value in enumerate(values):
                    writer.writerow(
                        [
                            sample.source, f"{sample.host_time_s:.6f}",
                            sample.device_time_ms, signal, channel, f"{value:.9g}",
                        ]
                    )
    return output


def default_output_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(__file__).resolve().parents[1] / "logs" / f"hil_{timestamp}.csv"