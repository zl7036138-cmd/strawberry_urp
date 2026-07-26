"""Dependency-free lifecycle helpers for the manipulation action server."""

from __future__ import annotations

import inspect
import threading


def shutdown_executor_and_wait(executor, timeout_sec: float | None = None) -> bool:
    """Stop an rclpy executor and join Jazzy worker threads deterministically."""

    try:
        supports_thread_wait = (
            "wait_for_threads" in inspect.signature(executor.shutdown).parameters
        )
    except (TypeError, ValueError):
        supports_thread_wait = False

    if supports_thread_wait:
        callbacks_stopped = bool(
            executor.shutdown(
                timeout_sec=timeout_sec,
                wait_for_threads=True,
            )
        )
    else:
        callbacks_stopped = bool(executor.shutdown(timeout_sec=timeout_sec))
        worker_pool = getattr(executor, "_executor", None)
        if worker_pool is not None and hasattr(worker_pool, "shutdown"):
            worker_pool.shutdown(wait=True)

    tasks = getattr(executor, "_futures", None)
    if tasks is not None:
        for task in tuple(tasks):
            if not task.done():
                continue
            try:
                task.result()
            except BaseException:
                pass
        tasks.clear()
    return callbacks_stopped


class ExclusiveGoalGate:
    """Thread-safe single-owner gate for a non-reentrant motion backend."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._active = False

    def try_acquire(self) -> bool:
        with self._condition:
            if self._active:
                return False
            self._active = True
            return True

    def release(self) -> None:
        with self._condition:
            if not self._active:
                raise RuntimeError("exclusive goal gate is not acquired")
            self._active = False
            self._condition.notify_all()

    def wait_until_idle(self, timeout_sec: float) -> bool:
        if timeout_sec <= 0.0:
            raise ValueError("timeout must be positive")
        with self._condition:
            return self._condition.wait_for(
                lambda: not self._active, timeout=timeout_sec
            )

    @property
    def active(self) -> bool:
        with self._condition:
            return self._active
