# Lazy fruit attachment (development backend)

Build with the normal Jazzy/Harmonic colcon workspace. Enable explicitly:

```bash
ros2 launch strawberry_bringup generalized_harvest.launch.py \
  gripper_attachment_backend:=lazy <other scene/model arguments>
```

The default remains `upstream`, preserving historical fixed-scene behavior.
The lazy backend emits the same Empty commands and attached/detached StringMsg
states. It creates no gripper joint during insertion. It refuses to create a
second incoming DetachableJoint for the fruit while its stem or another support
still exists. The adapter confirms stem release before requesting gripper
attachment; failed confirmation requests detach even for a queued attachment.

For generated scenes this backend also moves stem support out of the robot:
the launch writes a separate, geometry-free world-fixed support model into a
new temporary world. Fruit stays fixed before grasp without sharing the arm's
physical tree. Historical upstream worlds are unchanged. Transfer acknowledgments
must be received after the corresponding request; an old detached value is not
accepted as confirmation. This is reception ordering, not a transport request ID.

It does not reposition fruit, change gravity/collisions/controller tolerances,
read ROS truth or bypass bilateral contact. This addresses unsupported cyclic
topology, not every DART contact/reattachment limitation. An attached state means
the ECM component transition, as with the upstream plugin; it is not independent
proof of physical carrying, successful bin placement or stopped motion.

Protocol references (Apache-2.0 Gazebo source):
[DetachableJoint implementation](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/detachable_joint/DetachableJoint.cc),
[tree/contact limitations](https://gazebosim.org/api/sim/8/detachablejoints.html).
This is an independent compact implementation using the public ECM protocol;
it intentionally does not replicate upstream scoped-model fallback or initial attachment.
Child model names must resolve uniquely; ambiguous/missing children fail closed.

Cleared-world diagnostic fixtures may mount the plugin on a world with an
explicit `parent_model` plus `parent_link`. The unique parent is resolved after
robot insertion; missing/ambiguous endpoints and self-attachment never attach.
This is optional scenario assembly, not a harvesting command or contact bypass.
