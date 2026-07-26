"""Stable JSONL/CSV serialization for scenarios and trial results."""

from __future__ import annotations

from contextlib import contextmanager
import csv
import json
import os
from pathlib import Path
import tempfile
from typing import Any, Iterable, Iterator, Mapping, TextIO

from .models import Scenario, TrialResult


SCENARIO_CSV_FIELDS = (
    "schema_version",
    "trial_id",
    "kind",
    "occlusion",
    "lighting",
    "position",
    "seed",
    "expected_maturity",
)

TRIAL_CSV_FIELDS = (
    "schema_version",
    "trial_id",
    "mode",
    "scenario_kind",
    "occlusion",
    "lighting",
    "position",
    "seed",
    "expected_maturity",
    "predicted_maturity",
    "detection_confidence",
    "pick_attempted",
    "fruit_picked",
    "success",
    "first_failure_stage",
    "failure_code",
    "localization_error_mm",
    "planning_time_sec",
    "execution_time_sec",
    "timestamp_utc",
    "metadata_json",
)


@contextmanager
def _atomic_text_writer(path: str | Path) -> Iterator[TextIO]:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="",
            dir=target.parent,
            prefix=f".{target.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_path = Path(stream.name)
            yield stream
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, target)
        temporary_path = None
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def _json_line(record: Mapping[str, Any]) -> str:
    return json.dumps(
        record, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False
    )


def write_scenarios_jsonl(
    scenarios: Iterable[Scenario], path: str | Path
) -> int:
    count = 0
    with _atomic_text_writer(path) as stream:
        for scenario in scenarios:
            stream.write(_json_line(scenario.to_dict()))
            stream.write("\n")
            count += 1
    return count


def write_scenarios_csv(scenarios: Iterable[Scenario], path: str | Path) -> int:
    count = 0
    with _atomic_text_writer(path) as stream:
        writer = csv.DictWriter(stream, fieldnames=SCENARIO_CSV_FIELDS)
        writer.writeheader()
        for scenario in scenarios:
            writer.writerow(scenario.to_dict())
            count += 1
    return count


def write_scenarios(
    scenarios: Iterable[Scenario], path: str | Path, file_format: str | None = None
) -> int:
    selected = _select_format(path, file_format)
    if selected == "jsonl":
        return write_scenarios_jsonl(scenarios, path)
    return write_scenarios_csv(scenarios, path)


def write_trial_results_jsonl(
    results: Iterable[TrialResult], path: str | Path
) -> int:
    count = 0
    with _atomic_text_writer(path) as stream:
        for result in results:
            stream.write(_json_line(result.to_dict()))
            stream.write("\n")
            count += 1
    return count


def write_trial_results_csv(
    results: Iterable[TrialResult], path: str | Path
) -> int:
    count = 0
    with _atomic_text_writer(path) as stream:
        writer = csv.DictWriter(stream, fieldnames=TRIAL_CSV_FIELDS)
        writer.writeheader()
        for result in results:
            record = result.to_dict()
            metadata = record.pop("metadata")
            record["metadata_json"] = json.dumps(
                metadata,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
            writer.writerow(record)
            count += 1
    return count


def write_trial_results(
    results: Iterable[TrialResult], path: str | Path, file_format: str | None = None
) -> int:
    selected = _select_format(path, file_format)
    if selected == "jsonl":
        return write_trial_results_jsonl(results, path)
    return write_trial_results_csv(results, path)


def read_trial_results_jsonl(path: str | Path) -> list[TrialResult]:
    results: list[TrialResult] = []
    with Path(path).open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
                if not isinstance(record, Mapping):
                    raise ValueError("record must be a JSON object")
                results.append(TrialResult.from_dict(record))
            except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"invalid trial result at line {line_number}: {exc}") from exc
    return results


def read_trial_results_csv(path: str | Path) -> list[TrialResult]:
    results: list[TrialResult] = []
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = set(TRIAL_CSV_FIELDS) - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"trial CSV is missing columns: {', '.join(sorted(missing))}")
        for row_number, record in enumerate(reader, start=2):
            try:
                metadata_text = record.pop("metadata_json", "")
                record["metadata"] = json.loads(metadata_text) if metadata_text else {}
                results.append(TrialResult.from_dict(record))
            except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"invalid trial result at row {row_number}: {exc}") from exc
    return results


def read_trial_results(
    path: str | Path, file_format: str | None = None
) -> list[TrialResult]:
    selected = _select_format(path, file_format)
    if selected == "jsonl":
        return read_trial_results_jsonl(path)
    return read_trial_results_csv(path)


def write_json_report(report: Mapping[str, Any], path: str | Path) -> None:
    with _atomic_text_writer(path) as stream:
        json.dump(
            report,
            stream,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        stream.write("\n")


def _select_format(path: str | Path, file_format: str | None) -> str:
    if file_format is not None:
        normalized = file_format.lower()
        if normalized not in {"jsonl", "csv"}:
            raise ValueError("format must be 'jsonl' or 'csv'")
        return normalized
    suffix = Path(path).suffix.lower()
    if suffix in {".jsonl", ".ndjson"}:
        return "jsonl"
    if suffix == ".csv":
        return "csv"
    raise ValueError("cannot infer format; use a .jsonl/.ndjson/.csv suffix")
