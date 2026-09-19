from pathlib import Path
import subprocess
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from strawberry_bringup.development_probe import (  # noqa: E402
    ProbeWatchdog,
    build_probe_payload,
    event_progress_signature,
    recorder_exit_code,
)


class ProbeWatchdogTests(unittest.TestCase):
    def test_startup_timeout_requires_first_status(self):
        watchdog = ProbeWatchdog(10.0, 5.0, 20.0, 60.0)

        self.assertIsNone(watchdog.timeout_outcome(14.99))
        self.assertEqual(watchdog.timeout_outcome(15.0), "STARTUP_TIMEOUT")

    def test_new_action_stage_extends_idle_window(self):
        watchdog = ProbeWatchdog(0.0, 10.0, 20.0, 100.0)
        self.assertTrue(
            watchdog.observe(
                {"state": "PICKING", "outcome": "PICK_STAGE_APPROACH_0.30"},
                5.0,
            )
        )
        self.assertIsNone(watchdog.timeout_outcome(24.9))
        self.assertTrue(
            watchdog.observe(
                {"state": "PICKING", "outcome": "PICK_STAGE_PLACE_0.90"},
                25.0,
            )
        )
        self.assertIsNone(watchdog.timeout_outcome(44.9))
        self.assertEqual(watchdog.timeout_outcome(45.0), "INACTIVITY_TIMEOUT")

    def test_duplicate_status_does_not_extend_idle_window(self):
        event = {"state": "PICKING", "outcome": "PICK_SENT"}
        watchdog = ProbeWatchdog(0.0, 10.0, 20.0, 100.0)

        self.assertTrue(watchdog.observe(event, 5.0))
        self.assertFalse(watchdog.observe(event, 15.0))
        self.assertEqual(watchdog.timeout_outcome(25.0), "INACTIVITY_TIMEOUT")

    def test_hard_timeout_cannot_be_extended_by_progress(self):
        watchdog = ProbeWatchdog(0.0, 10.0, 20.0, 30.0)
        watchdog.observe({"state": "PICKING", "outcome": "A"}, 29.9)

        self.assertEqual(watchdog.timeout_outcome(30.0), "HARD_TIMEOUT")

    def test_invalid_timeout_relationship_is_rejected(self):
        with self.assertRaises(ValueError):
            ProbeWatchdog(0.0, 30.0, 10.0, 20.0)


class ProbeReceiptTests(unittest.TestCase):
    def test_signature_tracks_batch_lists_and_action_stage(self):
        first = event_progress_signature(
            {"state": "PICKING", "outcome": "PICK_STAGE_APPROACH_0.3"}
        )
        second = event_progress_signature(
            {"state": "PICKING", "outcome": "PICK_STAGE_PLACE_0.9"}
        )
        self.assertNotEqual(first, second)

    def test_terminal_receipt_is_explicit(self):
        payload = build_probe_payload(
            outcome="PARTIAL_SUCCESS",
            events=[{"state": "DONE", "outcome": "PARTIAL_SUCCESS"}],
            selection_events=[
                {
                    "outcome": "TARGET_SELECTED",
                    "track_id": 2,
                    "moveit_rejections": [{"track_id": 1}],
                }
            ],
            ground_truth_score_events=[
                {"event": "PLACED", "target_id": 3, "maturity": "RIPE"}
            ],
            elapsed_sec=90.0,
            startup_timeout_sec=120.0,
            idle_timeout_sec=180.0,
            hard_timeout_sec=900.0,
        )

        self.assertEqual(payload["schema_version"], 3)
        self.assertTrue(payload["terminal_status_received"])
        self.assertFalse(payload["formal_acceptance"])
        self.assertEqual(payload["selection_events"][0]["track_id"], 2)
        self.assertEqual(payload["ground_truth_score_events"][0]["target_id"], 3)
        self.assertFalse(payload["ground_truth_score_events_used_for_control"])

    def test_timeout_receipt_is_nonterminal_and_nonzero(self):
        payload = build_probe_payload(
            outcome="HARD_TIMEOUT",
            events=[{"state": "PICKING", "outcome": "PICK_SENT"}],
            selection_events=[],
            ground_truth_score_events=[],
            elapsed_sec=900.0,
            startup_timeout_sec=120.0,
            idle_timeout_sec=180.0,
            hard_timeout_sec=900.0,
        )

        self.assertFalse(payload["terminal_status_received"])
        self.assertNotEqual(recorder_exit_code("HARD_TIMEOUT"), 0)
        self.assertEqual(recorder_exit_code("SUCCESS"), 0)

    def test_tracked_runner_cleans_the_full_process_group(self):
        repository = PACKAGE.parents[2]
        runner = (
            repository / "scripts" / "run_generalized_development_probe.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("setsid ros2 launch", runner)
        self.assertIn('source "${repo_root}/scripts/lib/process_group_cleanup.sh"', runner)
        self.assertIn('launch_pgid="${launch_pid}"', runner)
        self.assertNotIn('launch_pgid="$(ps -o pgid=', runner)
        self.assertIn('terminate_process_group "${launch_pgid}"', runner)
        self.assertIn('"outcome":"CLEAN"', runner)
        self.assertIn("STRAWBERRY_PROBE_CLEANUP_SMOKE_SEC", runner)
        self.assertIn("STRAWBERRY_DEVELOPMENT_OUTPUT_DIR", runner)
        self.assertIn("generalized_truth_isolation_audit", runner)
        self.assertIn("generalized_development_score", runner)
        self.assertNotIn("rm -f", runner)
        self.assertIn("trap cleanup EXIT", runner)
        self.assertIn("trap stop_on_signal INT TERM", runner)
        self.assertNotIn('kill -0 "${launch_pid}"', runner)

    def test_cleanup_library_terminates_live_process_group(self):
        repository = PACKAGE.parents[2]
        library = repository / "scripts" / "lib" / "process_group_cleanup.sh"
        command = f"""
set -eo pipefail
source '{library.as_posix()}'
setsid bash -c 'sleep 300 & wait' &
leader=$!
pgid="$(ps -o pgid= -p "${{leader}}" | tr -d '[:space:]')"
process_group_alive "${{pgid}}"
terminate_process_group "${{pgid}}" "${{leader}}" 8 8
! process_group_alive "${{pgid}}"
"""
        result = subprocess.run(
            ["bash", "-lc", command],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )

        self.assertEqual(result.returncode, 0, result.stderr)

    def test_cleanup_library_kills_term_resistant_group(self):
        repository = PACKAGE.parents[2]
        library = repository / "scripts" / "lib" / "process_group_cleanup.sh"
        command = f"""
set -eo pipefail
source '{library.as_posix()}'
setsid bash -c 'trap "" TERM; sleep 300 & wait' &
leader=$!
pgid="$(ps -o pgid= -p "${{leader}}" | tr -d '[:space:]')"
process_group_alive "${{pgid}}"
terminate_process_group "${{pgid}}" "${{leader}}" 2 8
! process_group_alive "${{pgid}}"
"""
        result = subprocess.run(
            ["bash", "-lc", command],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )

        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
