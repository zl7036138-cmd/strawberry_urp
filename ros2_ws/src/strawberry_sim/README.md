# strawberry_sim

Gazebo Harmonic scene assets and fail-safe ROS adapters for the Strawberry URP
project.

The default launch is headless and contains a Blender-derived strawberry plant,
three independently graspable rigid fruits mounted at pedicel endpoints, a fixed
RGB-D camera, and a collection bin. The plant owns leaves and stems; each fruit
owns only its body, seeds, and calyx. The Panda model and working detachable
joint backend are injected by the manipulation integration stage; attachment
services remain disabled and fail closed until that backend explicitly reports
an initialized detached state.

```bash
ros2 launch strawberry_sim sim.launch.py headless:=true
```

Expected bridged topics include `/clock`, the three standard `/camera/*`
topics, and one pose stream for each fruit.

The original sphere-based tabletop scene remains available as an explicit
compatibility fixture:

```bash
ros2 launch strawberry_sim sim.launch.py \
  world_file:="$(ros2 pkg prefix strawberry_sim)/share/strawberry_sim/worlds/strawberry_tabletop_benchmark_v1.sdf" \
  scene_config_file:="$(ros2 pkg prefix strawberry_sim)/share/strawberry_sim/config/scene_tabletop_v1.yaml"
```

The larger Blender field is an opt-in, no-motion qualification scene. It keeps
the accepted v2 default unchanged and starts with the dual base/wrist cameras:

```bash
ros2 launch strawberry_sim field_v3.launch.py headless:=false
```

Field-v3 now qualifies scene loading, scale, rendering, sensor topics, target-1
wrist visibility, no-motion RGB-D localization, field-specific MoveIt
collisions, and controller-free pre-grasp planning. Its consolidated
60-sample run has 4.856 mm median/P95 centre error, retains all nine expected
collision objects, and discards a successful 47-waypoint plan without sending
a control command. The launcher deliberately disables fruit attachment and
pose-control services; field-v3 motion remains blocked until a separately
frozen Oracle execution gate is authorized and passes.
