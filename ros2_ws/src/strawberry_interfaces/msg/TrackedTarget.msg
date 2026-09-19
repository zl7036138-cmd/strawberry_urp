# Copyright 2026 Strawberry URP Team
# SPDX-License-Identifier: Apache-2.0

# Acquisition time of the latest observation and the common robot-base frame.
std_msgs/Header header

# Stable identity assigned by the scene-level geometric tracker. Unlike a
# StrawberryDetection target_id, this value persists across frames/cameras.
uint32 track_id

# Acquisition-local identity of the latest source detection, for diagnostics.
uint32 source_detection_id

uint8 UNKNOWN=0
uint8 RIPE=1
uint8 UNRIPE=2
uint8 maturity

geometry_msgs/Pose pose
float32 detection_confidence
float32 position_sigma_m

# Number of geometrically associated observations accumulated by this track.
uint32 observation_count
