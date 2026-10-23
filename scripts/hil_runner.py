"""Run and analyse experiments on the two-board HIL platform."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import serial
from serial.tools import list_ports

if __package__:
    from .utils.analysis import control_quality, plot_result
    from .utils.pairs import (
        CONTROLLERS,
        MODELS,
        SUPPORTED_PAIRS,
        TRAJECTORIES,
        TRAJECTORY_CATALOG,
        CatalogItem,
        PairSpec,
    )
    from .utils.protocol import (
        BAUDRATE,
        BoardConnection,
        HilError,
        ProtocolError,
        build_frame,
        parse_frame,
        xor_checksum,
    )
    from .utils.telemetry import (
        ExperimentConfig,
        ExperimentResult,
        StatusCallback,
        TelemetrySample,
        default_output_path,
        pair_for,
        parse_telemetry,
        run_experiment,
        write_csv,
    )
else:
    from utils.analysis import control_quality, plot_result
    from utils.pairs import (
        CONTROLLERS,
        MODELS,
        SUPPORTED_PAIRS,
        TRAJECTORIES,
        TRAJECTORY_CATALOG,
        CatalogItem,
        PairSpec,
    )
    from utils.protocol import (
        BAUDRATE,
        BoardConnection,
        HilError,
        ProtocolError,
        build_frame,
        parse_frame,
        xor_checksum,
    )
    from utils.telemetry import (
        ExperimentConfig,
        ExperimentResult,
        StatusCallback,
        TelemetrySample,
        default_output_path,
        pair_for,
        parse_telemetry,
        run_experiment,
        write_csv,
    )

def _print_board_catalog(
    port: str, role: str, command: str, expected: tuple[CatalogItem, ...]
) -> None:
    with BoardConnection(port, role) as board:
        response = board.command(command)
    entries = response.split(",")[1:]
    expected_entries = [f"{item.id}:{item.name}" for item in expected]
    if entries != expected_entries:
        raise ProtocolError(
            f"{role.title()} catalog is incomplete or differs from the host registry "
            f"({len(entries)}/{len(expected_entries)} entries)"
        )
    print(f"{role.title()} board ({port}):")
    for entry in entries:
        print(f"  {entry}")


def _print_status(message: str) -> None:
    print(message, flush=True)


def _print_metrics(metrics: dict[str, float]) -> None:
    print("Control quality (second half of capture):")
    for name, value in metrics.items():
        if name in ("tracking_excited", "quality_conclusive", "quality_pass"):
            continue
        if name == "controller_samples":
            print(f"  {name}: {int(value)}")
        else:
            print(f"  {name}: {value:.6g}")
    if not metrics["quality_conclusive"]:
        print("  quality_status: INCONCLUSIVE (capture is too short or lacks excitation)")
    else:
        print(f"  quality_status: {'PASS' if metrics['quality_pass'] else 'FAIL'}")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-port", default="COM6")
    parser.add_argument("--controller-port", default="COM8")
    parser.add_argument("--model", default="Mass-Spring-Damper")
    parser.add_argument("--controller", default="PID")
    parser.add_argument(
        "--trajectory",
        help="Comma-separated trajectory names; defaults to the selected pair profile",
    )
    parser.add_argument("--duration", type=float)
    parser.add_argument("--log-ms", type=int, default=50)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--plot-file", type=Path)
    parser.add_argument("--no-show", action="store_true")
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--show-models", action="store_true")
    parser.add_argument("--show-ctrls", action="store_true")
    parser.add_argument("--show-trajs", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.list_ports or args.show_models or args.show_ctrls or args.show_trajs:
        try:
            if args.list_ports:
                for port in list_ports.comports():
                    print(f"{port.device}: {port.description}")
            if args.show_models:
                _print_board_catalog(args.model_port, "model", "MODELS", MODELS)
            if args.show_ctrls:
                _print_board_catalog(
                    args.controller_port, "controller", "CTRLS", CONTROLLERS
                )
            if args.show_trajs:
                _print_board_catalog(
                    args.controller_port,
                    "controller",
                    "TRAJS",
                    TRAJECTORY_CATALOG,
                )
        except (HilError, serial.SerialException) as exc:
            print(f"HIL error: {exc}", file=sys.stderr)
            return 1
        return 0

    selected_pair = SUPPORTED_PAIRS.get((args.model, args.controller))
    if args.trajectory:
        trajectories = tuple(part.strip().lower() for part in args.trajectory.split(","))
    else:
        trajectories = selected_pair.default_trajectories if selected_pair else ()
    config = ExperimentConfig(
        model_port=args.model_port,
        controller_port=args.controller_port,
        model=args.model,
        controller=args.controller,
        trajectories=trajectories,
        duration_s=(
            args.duration
            if args.duration is not None
            else (selected_pair.default_duration_s if selected_pair else 20.0)
        ),
        log_period_ms=args.log_ms,
    )
    output = args.output or default_output_path()
    plot_file = args.plot_file or output.with_suffix(".png")

    try:
        result = run_experiment(config, status=_print_status)
        write_csv(result, output)
        metrics = control_quality(result)
        plot_result(result, plot_file, show=not args.no_show)
    except (HilError, serial.SerialException) as exc:
        print(f"HIL error: {exc}", file=sys.stderr)
        return 1

    print(f"CSV log: {output}")
    print(f"Plot: {plot_file}")
    _print_metrics(metrics)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())