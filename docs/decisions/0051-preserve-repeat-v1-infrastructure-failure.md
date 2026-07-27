# ADR 0051: Preserve repeat-v1 infrastructure failure

- Recorded: 2026-07-27
- Status: Accepted
- Scope: Repair only ROS-domain self-detection before the five-world matrix

## Context

The ADR-0050 repeat runner stopped before creating a trial directory, launching
Gazebo, or commanding the robot. Its free-domain check called
`ros2 node list --no-daemon --all --spin-time 1` and treated any output as an
occupied domain. On the current ROS 2 Jazzy CLI, that command reports its own
temporary node as `/_ros2cli_<pid>`. Every candidate domain therefore appeared
occupied.

The preserved infrastructure result records zero worlds started, zero robot
actions started, and zero trials consumed. This is not a mechanical trial
failure, but the v1 repeat invocation cannot be rewritten or presented as a
pass.

## Decision

1. Preserve the repeat-v1 contract, runner, empty result directory, and
   infrastructure-failure record.
2. Freeze a separately named
   `blender_v2_oracle_execution_repeat_5_v2` gate.
3. In the domain scan, remove only lines that exactly match
   `/_ros2cli_[0-9]+`. Any other discovered node continues to mark the domain
   occupied.
4. Keep the exact five-trial matrix, motion code, scene, target, dual-camera
   mode, preflight, grasp parameters, controller settings, thresholds, and
   5/5 requirement unchanged.
5. Do not count the pre-world v1 stop as one of the five v2 trials. V2 remains
   limited to five fresh worlds with one action per world and no retries.

## Consequences

The repair changes infrastructure discovery only. It cannot hide a real ROS
node or change manipulation behavior. Repeat-v1 remains failed before
execution; repeat-v2 becomes the sole authorized five-world matrix. All
formal, perception, held-out-test, and general pick authorizations remain
false.
