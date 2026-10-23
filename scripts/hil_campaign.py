"""Run hil_runner.py for all or selected model/controller pairs."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from utils.pairs import SUPPORTED_PAIRS


def _pair_name(pair: tuple[str, str]) -> str:
    return f"{pair[0]}:{pair[1]}"


def _slug(text: str) -> str:
    return text.lower().replace(" ", "-").replace(":", "_")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--all", action="store_true", help="run every supported pair")
    selection.add_argument(
        "--pair",
        action="append",
        metavar="MODEL:CONTROLLER",
        help="pair to run; may be specified more than once",
    )
    parser.add_argument("--duration", type=float, help="capture duration in seconds")
    parser.add_argument("--sampling-rate", type=float, help="logging rate in Hz")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).parent / "logs")
    parser.add_argument("--model-port", default="COM6")
    parser.add_argument("--controller-port", default="COM8")
    parser.add_argument("--list-pairs", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    available = {_pair_name(pair): pair for pair in SUPPORTED_PAIRS}

    if args.list_pairs:
        print("\n".join(available))
        return 0
    if not args.all and not args.pair:
        parser.error("specify --all or at least one --pair")
    if args.duration is None or args.duration <= 0:
        parser.error("--duration must be positive")
    if args.sampling_rate is None or not 0 < args.sampling_rate <= 1000:
        parser.error("--sampling-rate must be in the range (0, 1000] Hz")

    requested = list(available) if args.all else args.pair
    unknown = [name for name in requested if name not in available]
    if unknown:
        parser.error(f"unsupported pair: {unknown[0]}; use --list-pairs")

    log_period_ms = round(1000.0 / args.sampling_rate)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    runner = Path(__file__).with_name("hil_runner.py")
    failures = 0

    for name in requested:
        model, controller = available[name]
        stem = _slug(name)
        command = [
            sys.executable,
            str(runner),
            "--model-port", args.model_port,
            "--controller-port", args.controller_port,
            "--model", model,
            "--controller", controller,
            "--duration", str(args.duration),
            "--log-ms", str(log_period_ms),
            "--output", str(args.output_dir / f"{stem}.csv"),
            "--plot-file", str(args.output_dir / f"{stem}.png"),
            "--no-show",
        ]
        print(f"\n=== {name} ===", flush=True)
        failures += subprocess.run(command, check=False).returncode != 0

    print(f"\nCampaign complete: {len(requested) - failures} passed, {failures} failed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())