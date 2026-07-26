# Copyright 2026 Strawberry URP Team
# SPDX-License-Identifier: Apache-2.0

# Acquisition time and output coordinate frame. The v1 localization contract
# publishes this in panda_link0 and preserves the sensor acquisition timestamp.
std_msgs/Header header

# Identity copied from the source StrawberryDetection.
uint32 target_id

# Localized fruit-centre pose expressed in the header frame.
geometry_msgs/Pose pose

# Confidence copied from the source maturity detection, in [0.0, 1.0].
float32 detection_confidence

# Estimated one-sigma scalar position uncertainty, in metres.
float32 position_sigma_m
