# Demonstration video storyboard

Target length: 4-6 minutes. Every overlay must label the segment as
development, formal evidence, or diagnostic evidence.

| Time | Visual | Narration / overlay | Evidence boundary |
|---:|---|---|---|
| 0:00-0:25 | Title and architecture flow | Project title, ROS 2 Jazzy, Gazebo Harmonic, YOLO11s, MoveIt 2 | Simulation-only |
| 0:25-1:10 | Headed clear-scene run | Show RGB-D view, detection, target pose and Panda pick/place state transitions | Non-formal demonstration |
| 1:10-1:35 | Only-unripe scene | Show `NO_PICK` and no arm motion | Safety demonstration |
| 1:35-2:05 | T40/T50/T60 metric cards | 1.345 mm median localization; 9/10 truth-target pick; 10/10 Oracle integration | Accepted module gates |
| 2:05-2:45 | P3 generated charts | 39/135 overall; 33/45 no occlusion; 6/45 partial; 0/45 heavy | Formal P3 failed |
| 2:45-3:30 | P4 detection/localization comparison | Detection 300/300 but target pose 0/300 under heavy occlusion | No motion; intervention rejected |
| 3:30-4:10 | Failure analysis diagram/log excerpt | Centre depth lands on foreground occluder; association distance exceeds 0.080 m | Diagnostic explanation |
| 4:10-4:40 | Reproduction commands and manifest | Separate Ubuntu 24.04 WSL2: 246/246 tests and 10/10 smoke behaviours; show SHA-256 handoff | Clean reproduction accepted by ADR 0033 |
| 4:40-end | Conclusions and limitations | Working modules, negative safety, failed robustness gates, no real-robot claim | Honest final status |

Recording should use a fresh non-formal demo output directory and must not
overwrite any accepted or failed gate result. Never present a prerecorded
successful clear-scene run as a sample from the formal 135-trial positive
matrix.
