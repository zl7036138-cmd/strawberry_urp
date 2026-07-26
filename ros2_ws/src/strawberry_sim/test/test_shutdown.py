import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_sim.attachment_manager import (  # noqa: E402
    _shutdown_executor_and_wait,
)


class FakeWorkerPool:
    def __init__(self):
        self.wait_values = []

    def shutdown(self, *, wait):
        self.wait_values.append(wait)


class LegacyExecutor:
    def __init__(self):
        self.shutdown_timeouts = []
        self._executor = FakeWorkerPool()
        self._futures = []

    def shutdown(self, timeout_sec=None):
        self.shutdown_timeouts.append(timeout_sec)
        return True


class CurrentExecutor:
    def __init__(self):
        self.calls = []
        self._futures = []

    def shutdown(self, timeout_sec=None, *, wait_for_threads=False):
        self.calls.append((timeout_sec, wait_for_threads))
        return True


class ExecutorShutdownTests(unittest.TestCase):
    def test_legacy_executor_joins_private_worker_pool(self):
        executor = LegacyExecutor()
        self.assertTrue(_shutdown_executor_and_wait(executor, 1.25))
        self.assertEqual(executor.shutdown_timeouts, [1.25])
        self.assertEqual(executor._executor.wait_values, [True])

    def test_current_executor_uses_public_thread_wait(self):
        executor = CurrentExecutor()
        self.assertTrue(_shutdown_executor_and_wait(executor, 2.5))
        self.assertEqual(executor.calls, [(2.5, True)])


if __name__ == "__main__":
    unittest.main()
