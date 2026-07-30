# Submission-v2 demonstration storyboard

Target length: 4-6 minutes, H.264, 1280 x 720. The video is a submission
demonstration, not a new formal matrix.

| Segment | Visual | Required overlay | Evidence boundary |
|---|---|---|---|
| Title | Project title and technology stack | ROS 2 Jazzy, Gazebo Harmonic, YOLO11s, MoveIt 2 | Simulation only |
| Architecture | Base overview camera, wrist RGB-D and data flow | Sequential camera cooperation, not fusion | System explanation |
| field-v3 run | Split-screen base/wrist video with detections, 3D target and action stages | Fixed scene, fixed target, perception-derived action | Non-formal development demonstration |
| Repeat result | v15c/v16/v17 result table | Three consecutive successes with bilateral contact and attach/detach | Hash-bound development evidence |
| Formal metrics | P3 and YOLO cards | F1 0.800675; P3 39/135; negatives 30/30 | Failed formal gates preserved |
| Conclusion | Deliverables and limitations | No hardware, damage or sim-to-real claim | Honest final status |

The live field-v3 recording must show both camera streams, at least one target
pose, the ordered action stages, and final `SUCCESS`. The assembler may speed
up the live segment uniformly, but it must not delete a failed stage or present
the run as a sample from the formal P3 matrix.
