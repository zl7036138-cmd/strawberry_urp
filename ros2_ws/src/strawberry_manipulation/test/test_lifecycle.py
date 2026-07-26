import pathlib
import sys
import threading
import time
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.lifecycle import (  # noqa: E402
    ExclusiveGoalGate,
    shutdown_executor_and_wait,
)


class FakeTask:
    def __init__(self, error=None):
        self.error = error
        self.result_calls = 0

    def done(self):
        return True

    def result(self):
        self.result_calls += 1
        if self.error is not None:
            raise self.error


class FakeWorkerPool:
    def __init__(self):
        self.wait_values = []

    def shutdown(self, *, wait):
        self.wait_values.append(wait)


class LegacyExecutor:
    def __init__(self, tasks):
        self._executor = FakeWorkerPool()
        self._futures = list(tasks)
        self.shutdown_timeouts = []

    def shutdown(self, timeout_sec=None):
        self.shutdown_timeouts.append(timeout_sec)
        return True


class ExclusiveGoalGateTests(unittest.TestCase):
    def test_rejects_second_owner_until_release(self):
        gate = ExclusiveGoalGate()
        self.assertTrue(gate.try_acquire())
        self.assertTrue(gate.active)
        self.assertFalse(gate.try_acquire())
        gate.release()
        self.assertFalse(gate.active)
        self.assertTrue(gate.try_acquire())

    def test_competing_threads_have_exactly_one_winner(self):
        gate = ExclusiveGoalGate()
        barrier = threading.Barrier(8)
        results = []
        results_lock = threading.Lock()

        def compete():
            barrier.wait()
            acquired = gate.try_acquire()
            with results_lock:
                results.append(acquired)

        threads = [threading.Thread(target=compete) for _ in range(8)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(results.count(True), 1)
        self.assertEqual(results.count(False), 7)

    def test_release_without_owner_fails(self):
        with self.assertRaises(RuntimeError):
            ExclusiveGoalGate().release()

    def test_wait_until_idle_tracks_release(self):
        gate = ExclusiveGoalGate()
        gate.try_acquire()

        def release_later():
            time.sleep(0.02)
            gate.release()

        worker = threading.Thread(target=release_later)
        worker.start()
        self.assertTrue(gate.wait_until_idle(0.5))
        worker.join()

    def test_wait_until_idle_times_out(self):
        gate = ExclusiveGoalGate()
        gate.try_acquire()
        self.assertFalse(gate.wait_until_idle(0.01))
        with self.assertRaises(ValueError):
            gate.wait_until_idle(0.0)

    def test_executor_shutdown_joins_workers_and_consumes_tasks(self):
        tasks = [FakeTask(), FakeTask(RuntimeError("expected"))]
        executor = LegacyExecutor(tasks)

        self.assertTrue(shutdown_executor_and_wait(executor, 3.0))
        self.assertEqual(executor.shutdown_timeouts, [3.0])
        self.assertEqual(executor._executor.wait_values, [True])
        self.assertEqual([task.result_calls for task in tasks], [1, 1])
        self.assertEqual(executor._futures, [])


if __name__ == "__main__":
    unittest.main()
