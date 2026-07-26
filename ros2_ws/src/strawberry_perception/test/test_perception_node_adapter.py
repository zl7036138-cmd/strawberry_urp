from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_perception.perception_node import (  # noqa: E402
    ULTRALYTICS_NUMPY_ENCODING,
    _prediction_arguments,
    _result_rows,
    _run_guarded_frame,
)


class _FakeTensor:
    def __init__(self, values):
        self._values = values

    def detach(self):
        return self

    def cpu(self):
        return self

    def tolist(self):
        return self._values


class _FakeBoxes:
    xyxy = _FakeTensor([[1.0, 2.0, 5.0, 7.0]])
    conf = _FakeTensor([0.8])
    cls = _FakeTensor([1.0])


class ResultAdapterTest(unittest.TestCase):
    def test_ultralytics_numpy_source_is_explicitly_bgr(self):
        self.assertEqual(ULTRALYTICS_NUMPY_ENCODING, "bgr8")

    def test_converts_tensor_like_box_fields(self):
        result = type("Result", (), {"boxes": _FakeBoxes()})()
        self.assertEqual(_result_rows(result), [[1.0, 2.0, 5.0, 7.0, 0.8, 1.0]])

    def test_none_boxes_returns_empty_rows(self):
        result = type("Result", (), {"boxes": None})()
        self.assertEqual(_result_rows(result), [])

    def test_prediction_arguments_pass_explicit_nms_iou(self):
        image = object()
        arguments = _prediction_arguments(image, 640, 0.60, 0.70, "0")
        self.assertEqual(
            arguments,
            {
                "source": image,
                "imgsz": 640,
                "conf": 0.60,
                "iou": 0.70,
                "device": "0",
                "verbose": False,
            },
        )


class GuardedFrameTest(unittest.TestCase):
    def test_conversion_or_inference_failure_does_not_publish(self):
        publications = []
        errors = []

        def fail_frame():
            raise RuntimeError("bad image")

        self.assertFalse(
            _run_guarded_frame(fail_frame, publications.append, errors.append)
        )
        self.assertEqual(publications, [])
        self.assertEqual(len(errors), 1)
        self.assertIn("RuntimeError", errors[0])
        self.assertIn("bad image", errors[0])

    def test_valid_frame_publishes_once(self):
        publications = []
        errors = []
        self.assertTrue(
            _run_guarded_frame(lambda: "output", publications.append, errors.append)
        )
        self.assertEqual(publications, ["output"])
        self.assertEqual(errors, [])

    def test_publish_failure_is_contained(self):
        errors = []

        def fail_publish(_output):
            raise OSError("publisher unavailable")

        self.assertFalse(
            _run_guarded_frame(lambda: "output", fail_publish, errors.append)
        )
        self.assertIn("OSError", errors[0])


class ConfigSourceTest(unittest.TestCase):
    def test_package_config_declares_numeric_nms_default(self):
        config = (PACKAGE_ROOT / "config" / "perception.yaml").read_text(
            encoding="utf-8"
        )
        self.assertIn("nms_iou_threshold: 0.70", config)

    def test_adapter_does_not_redeclare_ros_builtin_sim_time_parameter(self):
        source = (
            PACKAGE_ROOT / "strawberry_perception" / "perception_node.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn('declare_parameter("use_sim_time"', source)

    def test_adapter_never_requests_rgb_for_ultralytics_numpy_source(self):
        source = (
            PACKAGE_ROOT / "strawberry_perception" / "perception_node.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn('desired_encoding="rgb8"', source)

    def test_adapter_drains_entities_before_context_shutdown(self):
        source = (
            PACKAGE_ROOT / "strawberry_perception" / "perception_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn("signal_handler_options=SignalHandlerOptions.NO", source)
        self.assertIn("signal.signal(signal.SIGTERM, _request_stop)", source)
        self.assertLess(
            source.rindex("executor.shutdown("),
            source.rindex("rclpy.try_shutdown()"),
        )

    def test_diagnostic_does_not_shadow_rclpy_parameter_storage(self):
        source = (
            PACKAGE_ROOT / "strawberry_perception" / "shadow_diagnostic.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("self._parameters =", source)
        self.assertIn("self._runtime_parameters =", source)


if __name__ == "__main__":
    unittest.main()
