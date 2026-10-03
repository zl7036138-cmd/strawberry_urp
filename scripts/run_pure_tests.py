"""Run every dependency-light unittest suite before ROS is available."""

from __future__ import annotations

import argparse
import json
import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
CLONE_SAFE_EXCLUSIONS = ROOT / "config" / "clone_safe_test_exclusions.json"
START_DIRS = sorted(
    list((ROOT / "ros2_ws" / "src").glob("*/test"))
    + list((ROOT / "tools").glob("**/test"))
    + list((ROOT / "tools").glob("**/tests"))
)


def _iter_tests(suite: unittest.TestSuite):
    for item in suite:
        if isinstance(item, unittest.TestSuite):
            yield from _iter_tests(item)
        else:
            yield item


def _clone_safe_suite(
    suite: unittest.TestSuite,
) -> tuple[unittest.TestSuite, int]:
    manifest = json.loads(CLONE_SAFE_EXCLUSIONS.read_text(encoding="utf-8"))
    suffixes = tuple(item["test_id_suffix"] for item in manifest["exclusions"])
    if len(suffixes) != len(set(suffixes)):
        raise RuntimeError("clone-safe exclusion suffixes must be unique")

    filtered = unittest.TestSuite()
    matched: set[str] = set()
    for test in _iter_tests(suite):
        test_id = test.id()
        matches = [suffix for suffix in suffixes if test_id.endswith(suffix)]
        if len(matches) > 1:
            raise RuntimeError(f"ambiguous clone-safe exclusion for {test_id}")
        if matches:
            matched.add(matches[0])
        else:
            filtered.addTest(test)
    missing = sorted(set(suffixes) - matched)
    if missing:
        raise RuntimeError(f"stale clone-safe exclusions did not match: {missing}")
    return filtered, len(matched)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--clone-safe",
        action="store_true",
        help=(
            "exclude only the frozen tests that require intentionally ignored "
            "large/local artifacts"
        ),
    )
    args = parser.parse_args()
    loader = unittest.TestLoader()
    combined = unittest.TestSuite()
    for start in START_DIRS:
        package_name = start.parent.name
        package_root = str(start.parent)
        if package_root not in sys.path:
            sys.path.insert(0, package_root)
        for test_file in sorted(start.glob("test_*.py")):
            module_name = f"_strawberry_tests_{package_name}_{test_file.stem}"
            spec = importlib.util.spec_from_file_location(module_name, test_file)
            if spec is None or spec.loader is None:
                raise RuntimeError(f"cannot load test module {test_file}")
            module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = module
            spec.loader.exec_module(module)
            combined.addTests(loader.loadTestsFromModule(module))
    excluded = 0
    if args.clone_safe:
        combined, excluded = _clone_safe_suite(combined)
    result = unittest.TextTestRunner(verbosity=2).run(combined)
    summary = {
        "profile": "clone_safe" if args.clone_safe else "full_local",
        "test_directories": len(START_DIRS),
        "tests_run": result.testsRun,
        "excluded_local_artifact_tests": excluded,
        "failures": len(result.failures),
        "errors": len(result.errors),
        "skipped": len(result.skipped),
        "successful": result.wasSuccessful(),
    }
    print(json.dumps(summary, sort_keys=True))
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
