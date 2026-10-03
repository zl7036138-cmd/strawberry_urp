# Read-only Gazebo command-chain audit (development only)

This optional plugin observes `panda_joint5` in the ECM. It has no command
publishers, ROS truth subscribers, or component writes. It is not required for
normal harvesting and does not qualify a run as safe or successful.

Build under WSL with the Jazzy vendor environment:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S tools/gazebo_command_audit -B /home/lzl/.cache/strawberry_urp/command_audit_v2
cmake --build /home/lzl/.cache/strawberry_urp/command_audit_v2 -j2
```

Insert the plugin into a **new development world copy**, never the historical
or formal world, with an absolute `output_file` path whose parent exists:

```xml
<plugin filename="/absolute/path/libstrawberry_command_audit.so"
        name="strawberry::CommandAudit">
  <output_file>/absolute/new/run/ecm_commands.jsonl</output_file>
</plugin>
```

The observer's Update priority is -1, before Physics (default 0), after all
PreUpdate command writers. Only UPDATE records represent the command to be
consumed by Physics; POST_UPDATE is a state observation. Physics can clear the
command after consumption. A post-physics zero is **not evidence of a zero
command being sent**. First check a nonzero command during a bounded replay.
Do not override Physics/observer priorities in the diagnostic world.

GRAPH records capture changes in incoming DetachableJoint counts, including
paused insertion. `multi_supported_child_count` reports children with more than
one such support. These are ECM topology observations, not proof that Physics
processed a constraint or that fruit was carried safely. A replacement parent
with an unchanged count may not emit a new GRAPH record.

`scripts/probe_empty_arm_command_chain.py` starts one empty-world simulation,
initializes the robot from a recorded command start and replays that same
bounded trajectory once. It observes joint/controller feedback, never uses
fruit truth, and applies unchanged controller tolerances. All output paths
are create-once; startup/acceptance/execution failures are retained. Run it
with a dedicated ROS_DOMAIN_ID and GZ_PARTITION and no other active simulator.
The captured start is a scenario initialization, not a claimed recovery move.

The replay validator limits travel to 2 rad, duration to 10 simulated seconds,
and preserves the 0.02 rad joint margin. This is a cleared-world control
diagnostic, **not** a replacement for MoveIt collision validation in a field.

For an over-bound original command, `--prefix-duration-sec 0.5` derives a short
recorded segment with zero terminal derivatives. The original trajectory hash
is retained and the receipt explicitly denies full-original-command replay.
Travel, timing and joint-margin bounds are not raised.

`--diagnostic-load-state welded|released --attachment-plugin /built/plugin.so`
adds a geometry-free 30 g fixture, assembled from URDF FK at the recorded initial
posture. It transfers a world-held support to the arm, optionally releases it,
then runs the same bounded command. This is deliberately **not a contact grasp**.
The optional world-mounted lazy plugin resolves an explicit unique `parent_model`
after insertion. Normal model-mounted configuration remains unchanged.

Build the updated observer into a fresh versioned directory. Fixture mode enables
FIXTURE_POSE records (off by default): arm/fixture component world poses and
relative pose at 100 Hz. A graph-only attached status cannot pass fixture checks;
welded mode also requires observed parent travel and stable relative pose, while
released mode requires observed falling. These are diagnostic thresholds only,
not harvesting safety thresholds. Results do not prove full-field collision
safety, placement, recovery or multi-fruit acceptance.
