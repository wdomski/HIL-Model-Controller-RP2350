"""Model-independent HIL quality metrics and plotting."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable

from .protocol import HilError
from .telemetry import ExperimentResult, TelemetrySample, pair_for


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def control_quality(result: ExperimentResult) -> dict[str, float]:
    """Calculate vector tracking metrics over the latter half of a run."""
    pair = pair_for(result.config)
    if pair.quality_mode == "estimate":
        return _estimation_quality(result)

    tracking: list[tuple[TelemetrySample, tuple[float, ...], tuple[float, ...]]] = []
    for sample in result.samples:
        setpoint = sample.signals.get("sp")
        actual = pair.tracking_values(sample.signals)
        n_channels = len(pair.setpoint_channels)
        if (
            sample.source == "CTRL"
            and setpoint is not None
            and actual is not None
            and len(setpoint) >= n_channels
            and len(actual) >= n_channels
        ):
            tracking.append((sample, actual[:n_channels], setpoint[:n_channels]))
    if not tracking:
        raise HilError("Controller telemetry contains no measured/setpoint values")

    cutoff = tracking[0][0].host_time_s + (
        tracking[-1][0].host_time_s - tracking[0][0].host_time_s
    ) / 2.0
    steady = [entry for entry in tracking if entry[0].host_time_s >= cutoff]
    errors = [
        tuple(actual[i] - setpoint[i] for i in range(len(pair.setpoint_channels)))
        for _, actual, setpoint in steady
    ]
    error_norms = [math.sqrt(sum(value * value for value in error)) for error in errors]
    controls = [
        value for sample, _, _ in steady for value in sample.signals.get("u", ())
    ]
    setpoint_spans = [
        max(setpoint[channel] for _, _, setpoint in tracking)
        - min(setpoint[channel] for _, _, setpoint in tracking)
        for channel in pair.setpoint_channels
    ]
    measured_spans = [
        max(actual[channel] for _, actual, _ in tracking)
        - min(actual[channel] for _, actual, _ in tracking)
        for channel in pair.setpoint_channels
    ]

    metrics = {
        "controller_samples": float(len(tracking)),
        "capture_duration_s": tracking[-1][0].host_time_s - tracking[0][0].host_time_s,
        "setpoint_peak_to_peak": max(setpoint_spans),
        "measured_peak_to_peak": max(measured_spans),
        "control_peak_to_peak": max(controls) - min(controls) if controls else 0.0,
        "tracking_excited": float(any(span >= 1e-3 for span in setpoint_spans)),
        "steady_rmse": math.sqrt(
            sum(value * value for value in error_norms) / len(error_norms)
        ),
        "steady_mae": sum(error_norms) / len(error_norms),
        "steady_max_abs_error": max(error_norms),
    }
    for channel in pair.setpoint_channels:
        metrics[f"steady_rmse_ch{channel}"] = math.sqrt(
            sum(error[channel] * error[channel] for error in errors) / len(errors)
        )
    if pair.control_limit is not None and controls:
        threshold = pair.control_limit * 0.99
        metrics["control_saturation_fraction"] = sum(
            abs(value) >= threshold for value in controls
        ) / len(controls)
    pendulum_angles = [
        abs(_wrap_angle(sample.signals["y"][1] - pair.pendulum_target_angle))
        for sample, _, _ in steady
        if len(sample.signals.get("y", ())) > 1
    ]
    if pair.max_pendulum_angle is not None and pendulum_angles:
        metrics["steady_max_abs_pendulum_angle"] = max(pendulum_angles)
        metrics["pendulum_angle_limit"] = pair.max_pendulum_angle
    pendulum_ok = (
        pair.max_pendulum_angle is None
        or bool(pendulum_angles and max(pendulum_angles) <= pair.max_pendulum_angle)
    )
    metrics["quality_conclusive"] = float(
        bool(metrics["tracking_excited"] or not pair.requires_excitation)
    )
    metrics["quality_pass"] = float(
        bool(
            metrics["quality_conclusive"]
            and metrics["steady_rmse"] <= pair.max_rmse
            and pendulum_ok
        )
    )
    return metrics


def _estimation_quality(result: ExperimentResult) -> dict[str, float]:
    pair = pair_for(result.config)
    if pair.estimate_target is None or pair.estimate_tolerance is None:
        raise HilError("Estimator profile has no target or tolerance")

    samples = [
        sample for sample in result.samples
        if sample.source == "CTRL" and sample.signals.get("est")
    ]
    if not samples:
        raise HilError("Controller telemetry contains no parameter estimate")

    duration = samples[-1].host_time_s - samples[0].host_time_s
    cutoff = samples[0].host_time_s + duration / 2.0
    steady = [sample.signals["est"][0] for sample in samples if sample.host_time_s >= cutoff]
    controls = [value for sample in samples for value in sample.signals.get("u", ())]
    errors = [value - pair.estimate_target for value in steady]
    final_estimate = samples[-1].signals["est"][0]
    final_error = abs(final_estimate - pair.estimate_target)
    conclusive = duration >= pair.minimum_quality_duration_s
    metrics = {
        "controller_samples": float(len(samples)),
        "capture_duration_s": duration,
        "setpoint_peak_to_peak": 0.0,
        "measured_peak_to_peak": max(steady) - min(steady),
        "control_peak_to_peak": max(controls) - min(controls) if controls else 0.0,
        "tracking_excited": float(bool(controls and max(controls) - min(controls) >= 1e-3)),
        "steady_rmse": math.sqrt(sum(error * error for error in errors) / len(errors)),
        "steady_mae": sum(abs(error) for error in errors) / len(errors),
        "steady_max_abs_error": max(abs(error) for error in errors),
        "final_estimate": final_estimate,
        "estimate_target": pair.estimate_target,
        "final_estimation_error": final_error,
        "estimate_tolerance": pair.estimate_tolerance,
        "quality_conclusive": float(conclusive),
        "quality_pass": float(conclusive and final_error <= pair.estimate_tolerance),
    }
    if pair.control_limit is not None and controls:
        threshold = pair.control_limit * 0.99
        metrics["control_saturation_fraction"] = sum(
            abs(value) >= threshold for value in controls
        ) / len(controls)
    return metrics


def _series(
    samples: Iterable[TelemetrySample], source: str, signal: str, channel: int
) -> tuple[list[float], list[float]]:
    points = [
        (sample.host_time_s, sample.signals[signal][channel])
        for sample in samples
        if sample.source == source
        and signal in sample.signals
        and channel < len(sample.signals[signal])
    ]
    return [point[0] for point in points], [point[1] for point in points]


def plot_result(
    result: ExperimentResult,
    path: str | Path | None = None,
    show: bool = True,
) -> Path | None:
    """Plot tracking, control effort, model states, and pair-specific views."""
    import matplotlib.pyplot as plt

    pair = pair_for(result.config)
    show_pendulum = pair.model == "Pendulum-Cart"
    extra_rows = int(show_pendulum) + int(bool(pair.cartesian_planes))
    columns = max(1, len(pair.cartesian_planes))
    figure = plt.figure(figsize=(12, 8 + 2.8 * extra_rows))
    grid = figure.add_gridspec(3 + extra_rows, columns)
    figure.suptitle(f"{result.config.model} / {result.config.controller}")

    ctrl_samples = [sample for sample in result.samples if sample.source == "CTRL"]
    row = 0
    tracking_axis = figure.add_subplot(grid[row, :])
    row += 1
    for channel, label in enumerate(pair.tracking_labels):
        if pair.quality_mode == "estimate" and pair.estimate_target is not None:
            setpoint_points = [
                (sample.host_time_s, pair.estimate_target) for sample in ctrl_samples
            ]
        else:
            setpoint_points = [
                (sample.host_time_s, sample.signals["sp"][channel])
                for sample in ctrl_samples
                if len(sample.signals.get("sp", ())) > channel
            ]
        actual_points = [
            (sample.host_time_s, actual[channel])
            for sample in ctrl_samples
            if (actual := pair.tracking_values(sample.signals)) is not None
            and len(actual) > channel
        ]
        tracking_axis.plot(
            [point[0] for point in setpoint_points],
            [point[1] for point in setpoint_points],
            "--", label=f"sp {label}",
        )
        tracking_axis.plot(
            [point[0] for point in actual_points],
            [point[1] for point in actual_points], label=label,
        )
    tracking_axis.set_ylabel(
        f"Tracking [{pair.tracking_unit}]" if pair.tracking_unit else "Tracking"
    )
    tracking_axis.legend()
    tracking_axis.grid(True, alpha=0.3)

    if show_pendulum:
        pendulum_axis = figure.add_subplot(grid[row, :], sharex=tracking_axis)
        theta_times, theta = _series(result.samples, "CTRL", "y", 1)
        theta = [
            pair.pendulum_target_angle +
            _wrap_angle(value - pair.pendulum_target_angle)
            for value in theta
        ]
        pendulum_axis.plot(theta_times, theta, color="tab:red", label="theta")
        pendulum_axis.axhline(
            pair.pendulum_target_angle,
            color="black",
            linewidth=0.8,
            alpha=0.4,
            label="target",
        )
        pendulum_axis.set_ylabel("Angle [rad]")
        pendulum_axis.grid(True, alpha=0.3)

        rate_axis = pendulum_axis.twinx()
        rate_times, theta_rate = _series(result.samples, "MODEL", "x", 3)
        rate_axis.plot(
            rate_times, theta_rate, color="tab:orange", alpha=0.75,
            label="theta rate",
        )
        rate_axis.set_ylabel("Angular rate [rad/s]")
        lines = pendulum_axis.get_lines()[:1] + rate_axis.get_lines()
        pendulum_axis.legend(lines, [line.get_label() for line in lines])
        row += 1

    if pair.cartesian_planes:
        tracking_points = [
            (sample.signals["sp"], actual)
            for sample in ctrl_samples
            if "sp" in sample.signals
            and (actual := pair.tracking_values(sample.signals)) is not None
        ]
        for column, (title, horizontal, vertical) in enumerate(pair.cartesian_planes):
            spatial_axis = figure.add_subplot(grid[row, column])
            desired = [
                point[0] for point in tracking_points
                if len(point[0]) > max(horizontal, vertical)
            ]
            actual = [
                point[1] for point in tracking_points
                if len(point[1]) > max(horizontal, vertical)
            ]
            spatial_axis.plot(
                [point[horizontal] for point in desired],
                [point[vertical] for point in desired], "--", label="desired",
            )
            spatial_axis.plot(
                [point[horizontal] for point in actual],
                [point[vertical] for point in actual], label="measured",
            )
            spatial_axis.set_title(f"{title} trajectory")
            spatial_axis.set_xlabel(f"{pair.tracking_labels[horizontal]} [{pair.tracking_unit}]")
            spatial_axis.set_ylabel(f"{pair.tracking_labels[vertical]} [{pair.tracking_unit}]")
            spatial_axis.set_aspect("equal", adjustable="datalim")
            spatial_axis.legend()
            spatial_axis.grid(True, alpha=0.3)
        row += 1

    control_channels = max(
        (len(sample.signals.get("u", ())) for sample in ctrl_samples), default=0
    )
    control_axis = figure.add_subplot(grid[row, :], sharex=tracking_axis)
    row += 1
    for channel in range(control_channels):
        times, values = _series(result.samples, "CTRL", "u", channel)
        control_axis.plot(times, values, label=f"u[{channel}]")
    control_axis.set_ylabel(
        f"Control [{pair.control_unit}]" if pair.control_unit else "Control"
    )
    control_axis.legend()
    control_axis.grid(True, alpha=0.3)

    model_channels = sorted(
        {
            channel
            for sample in result.samples
            if sample.source == "MODEL"
            for channel in range(len(sample.signals.get("x", ())))
        }
    )
    state_axis = figure.add_subplot(grid[row, :], sharex=tracking_axis)
    for channel in model_channels:
        times, values = _series(result.samples, "MODEL", "x", channel)
        label = (
            pair.model_state_labels[channel]
            if channel < len(pair.model_state_labels)
            else f"x[{channel}]"
        )
        state_axis.plot(times, values, label=label)
    state_axis.set_xlabel("Capture time [s]")
    state_axis.set_ylabel("Model states")
    state_axis.legend()
    state_axis.grid(True, alpha=0.3)
    figure.tight_layout()

    output: Path | None = None
    if path is not None:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output, dpi=150)
    if show:
        plt.show(block=False)
    else:
        plt.close(figure)
    return output