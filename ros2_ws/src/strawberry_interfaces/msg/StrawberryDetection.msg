# Copyright 2026 Strawberry URP Team
# SPDX-License-Identifier: Apache-2.0

# Stable maturity classification values. UNKNOWN is reserved for detections
# whose maturity cannot be determined reliably.
uint8 UNKNOWN=0
uint8 RIPE=1
uint8 UNRIPE=2

# Identity assigned by the perception node for this acquisition.
uint32 target_id

# One of UNKNOWN, RIPE, or UNRIPE.
uint8 maturity

# Detector confidence in the closed interval [0.0, 1.0].
float32 confidence

# Pixel-space bounding region in the image named by the array header.
sensor_msgs/RegionOfInterest bbox
