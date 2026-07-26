# strawberry_perception

The package separates YOLO post-processing from its ROS 2 adapter. Importing
`strawberry_perception.core` requires neither ROS 2 nor Ultralytics; the runtime
node imports those dependencies only when its console entry point starts.

The node subscribes to `/camera/color/image_raw`, runs the configured model,
maps model classes to stable `RIPE=1` and `UNRIPE=2` values, rejects unknown and
detections below the configured threshold, and publishes
`/strawberry/detections`. Equal inputs always produce the same confidence-first
ordering and frame-local uint32 target IDs.

The public camera message remains `rgb8`, but the adapter requests `bgr8` from
CvBridge before passing a NumPy array to Ultralytics. Ultralytics 8.4.92 treats
NumPy HWC sources as BGR and reverses them internally; passing an already-RGB
array would swap red and blue. ADR 0006 records the same-frame A/B evidence.

Startup is fail-fast: `model_path` must be an existing regular file,
`confidence_threshold` must be finite and in `[0, 1]`, `image_size` must be a
positive integer, and `nms_iou_threshold` must be finite and in `(0, 1]`. The
loaded weight must expose exactly `0=ripe` and `1=unripe`; missing, swapped,
aliased, or extra classes are rejected before subscribing to images. The
package default NMS IoU is `0.70` and is passed explicitly to every
`YOLO.predict` call.

Image conversion, inference, result adaptation, and publication are isolated
per frame. If one frame raises an exception, the node logs and drops that frame
without publishing a partial detection array, then remains available for the
next image.

The package-local `perception.yaml` keeps a numeric `0.60` fallback for direct
node startup. In full-system launches, `strawberry_bringup` supplies the shared
`confidence_threshold` launch argument to both perception and localization, so
the two filtering stages cannot silently use different cutoffs. The system
default remains `0.60`; any later replacement must first be frozen on the
validation set and then supplied once at the system launch boundary.

Run the isolated simulator channel/truth diagnostic after a workspace build:

```bash
bash scripts/run_shadow_forensics.sh \
  outputs/perception/yolo11s_640/weights/best.pt \
  results/t60/shadow_forensics_manual 224 5
```

The output contains exact source hashes, projected truth, both legacy-RGB and
correct-BGR predictions, overlays, and `summary.json`. It is diagnostic only
and never routes a target to the robot controller.

Run the no-motion simulator perception pre-gate with:

```bash
bash scripts/run_sim_perception_gate.sh \
  results/t60/sim_perception_pre_gate_manual \
  outputs/perception/yolo11s_640/weights/best.pt \
  225 10
```

The runner evaluates 10 frames at each of five ripe and five only-unripe fixed
positions. It requires ripe truth-associated frame recall and ripe target-pose
rate of at least 0.90, and a false-ripe negative-frame rate no greater than
0.05. It never starts manipulation. These operational thresholds do not replace
the real-image T30 gate or the P3/P4 end-to-end matrices.

Windows-compatible core tests:

```powershell
python -m unittest discover -s ros2_ws/src/strawberry_perception/test -p "test_*.py" -v
```

ROS 2 build and test (Ubuntu 24.04 / Jazzy):

```bash
cd ros2_ws
colcon build --packages-up-to strawberry_perception
colcon test --packages-select strawberry_perception --event-handlers console_direct+
```
