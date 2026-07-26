"""Run every dependency-light unittest suite before ROS is available."""

from __future__ import annotations

import json
import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
START_DIRS = sorted(
    list((ROOT / "ros2_ws" / "src").glob("*/test"))
    + list((ROOT / "tools").glob("**/test"))
    + list((ROOT / "tools").glob("**/tests"))
)


def main() -> int:
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
    result = unittest.TextTestRunner(verbosity=2).run(combined)
    summary = {
        "test_directories": len(START_DIRS),
        "tests_run": result.testsRun,
        "failures": len(result.failures),
        "errors": len(result.errors),
        "skipped": len(result.skipped),
        "successful": result.wasSuccessful(),
    }
    print(json.dumps(summary, sort_keys=True))
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
