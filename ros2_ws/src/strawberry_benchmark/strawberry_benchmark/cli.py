"""Command-line interface for generating and evaluating benchmark artifacts."""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

from .acceptance import GateStatus, evaluate_acceptance, load_acceptance_thresholds
from .io import read_trial_results, write_json_report, write_scenarios
from .metrics import compute_metrics
from .scenarios import generate_scenarios, load_benchmark_spec


def _write_or_print(payload: Mapping[str, Any], output: str | None) -> None:
    if output:
        write_json_report(payload, output)
    else:
        json.dump(
            payload,
            sys.stdout,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        sys.stdout.write("\n")


def _add_result_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--results", required=True, help="Trial .jsonl or .csv file")
    parser.add_argument(
        "--format",
        choices=("jsonl", "csv"),
        help="Input format; inferred from the suffix when omitted",
    )
    parser.add_argument(
        "--perception-macro-f1",
        type=float,
        help="Override F1 with the independently frozen real-test-set result",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="strawberry-benchmark",
        description="Generate scenarios and evaluate Strawberry URP trial logs.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    generate = subparsers.add_parser(
        "generate-scenarios", help="Generate the fixed 165-scenario manifest"
    )
    generate.add_argument("--config", required=True, help="Benchmark YAML config")
    generate.add_argument("--output", required=True, help="Output .jsonl or .csv")
    generate.add_argument(
        "--format", choices=("jsonl", "csv"), help="Override output format"
    )

    metrics = subparsers.add_parser("metrics", help="Aggregate a trial result log")
    _add_result_arguments(metrics)
    metrics.add_argument("--output", help="Write JSON instead of standard output")

    evaluate = subparsers.add_parser(
        "evaluate", help="Aggregate a trial result log and apply project gates"
    )
    _add_result_arguments(evaluate)
    evaluate.add_argument(
        "--project-config", required=True, help="Project YAML containing acceptance"
    )
    evaluate.add_argument("--output", help="Write JSON instead of standard output")
    return parser


def _metrics_from_args(args: argparse.Namespace):
    metrics = compute_metrics(read_trial_results(args.results, args.format))
    if args.perception_macro_f1 is not None:
        if not 0.0 <= args.perception_macro_f1 <= 1.0:
            raise ValueError("--perception-macro-f1 must be between 0 and 1")
        metrics = replace(metrics, perception_macro_f1=args.perception_macro_f1)
    return metrics


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "generate-scenarios":
            spec = load_benchmark_spec(args.config)
            scenarios = generate_scenarios(spec)
            written = write_scenarios(scenarios, args.output, args.format)
            summary = {
                "output": str(Path(args.output)),
                "positive_trials": spec.positive_trials,
                "negative_trials": spec.negative_trials,
                "total_trials": written,
            }
            _write_or_print(summary, None)
            return 0

        metrics = _metrics_from_args(args)
        if args.command == "metrics":
            _write_or_print({"metrics": metrics.to_dict()}, args.output)
            return 0

        thresholds = load_acceptance_thresholds(args.project_config)
        report = evaluate_acceptance(metrics, thresholds)
        _write_or_print(
            {"metrics": metrics.to_dict(), "acceptance": report.to_dict()},
            args.output,
        )
        if report.status is GateStatus.PASS:
            return 0
        if report.status is GateStatus.FAIL:
            return 1
        return 2
    except (KeyError, OSError, TypeError, ValueError) as exc:
        parser.error(str(exc))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
