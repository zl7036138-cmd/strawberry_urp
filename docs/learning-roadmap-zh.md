# 草莓 URP 项目学习教材（任务驱动版）

> 适用主线：固定机械臂、随机多植株、底座全局相机、夹爪近距离相机、
> 多目标三维跟踪、MoveIt 安全预选、连续采摘。
>
> 校验基线：2026-08-31 开发门审计工作树；最终冻结提交和 v15 进度记录在
> 本轮资格筛选完成后写入。历史单果证据见
> `config/generalized_runtime_progress_v14.json`。
>
> 本教材不要求死记代码行号。查找函数时优先使用 `rg`；代码变化后，应以
> 函数名、接口定义和测试为准。

---

## 1. 学完以后，你应该能做什么

完成这套教材，不以“看完多少页”判断，而以五项真实能力判断：

1. 能从底座相机的一帧图像开始，解释一颗草莓如何变成稳定 `track_id`，
   再变成一次抓放动作；
2. 能区分感知、定位、跟踪、选择、观察、规划、执行和验证失败；
3. 能解释系统为什么拒绝未成熟、高不确定度、陈旧、不可达或有碰撞风险的目标；
4. 能先写或修改依赖较少的测试，再安全地修改一处纯 Python 逻辑；
5. 能区分“代码具备”“开发运行证明”和“正式隐藏评测通过”三种结论。

当前项目已经证明：在多目标开发场景中，系统能在发布目标前用 MoveIt 淘汰
不可行目标，并对另一颗可行草莓完成视觉确认、物理接触、抓取、放置和验证。
尚未证明的是：同一批次连续成功采摘两颗以上可达成熟果，以及正式 30 个隐藏
种子的总体指标。学习过程中不得把“单果成功”表述成“最终泛化验收通过”。

---

## 2. 不再使用“全部读完再动手”的方法

### 2.1 每个单元固定采用六步循环

每次学习控制在约 60～90 分钟，按下面顺序进行：

1. **提出问题**：这一单元要解释哪个真实现象？
2. **先做预测**：运行前写下你认为会发生什么；
3. **只读必要代码**：一次精读一个函数或不超过约 80 行；
4. **执行安全实验**：先纯测试，再无运动仿真，最后才是有界动作；
5. **保存证据**：记录命令、输出路径、关键字段和退出码；
6. **脱离文档复述**：用自己的话解释数据如何流动、为什么可能失败。

如果只是看懂文字，却无法预测输出、解释日志或修改测试，这一单元还没有学会。

### 2.2 学习记录模板

每个单元新建一条个人笔记，至少写下：

```text
日期与当前提交：
本次问题：
运行前预测：
执行命令：
证据文件或关键输出：
实际结果与预测差异：
我现在能独立解释的内容：
仍不理解的问题：
```

完成单元后的第 1 天和第 7 天，不看教材回答该单元的三个“检查问题”。这比反复
重读更能暴露没有真正掌握的地方。

### 2.3 路线不是强制直线

- 完全零基础：按单元 0 → 8 学习；
- 已会 Python、Git 和 Linux：完成单元 0 的基线检查后，从单元 1 开始；
- 已熟悉 ROS 2：可以快速通过单元 2，但必须完成身份、时间戳和坐标系练习；
- 已熟悉 MoveIt：仍需学习本项目的真值隔离、碰撞物同步和一次重观察边界。

是否跳过由“阶段作品”决定，不由自我感觉决定。

---

## 3. 当前系统全景

### 3.1 运行时控制数据流

```text
底座 RGB ─▶ base perception ─▶ /strawberry/base/detections
                                      │
底座 Depth + CameraInfo + TF ─────────┤
                                      ▼
                           generalized localization
                                      │
                                      ▼
                         /strawberry/tracked_targets
                                      │
                         成熟度/新鲜度/不确定度/间隙
                                      │
                                      ▼
                              target_selector
                                      │
                  /strawberry/evaluate_target（MoveIt，不运动）
                                      │
                   ┌──────────────────┴──────────────────┐
                   ▼                                     ▼
      /strawberry/target_pose          /strawberry/wrist_observation_plan
                   └──────────────────┬──────────────────┘
                                      ▼
                           harvest_orchestrator
                                      │
                       移动到安全的夹爪观察位姿
                                      │
                                      ▼
腕部 RGB-D ─▶ wrist perception/localization ─▶ 腕部近距离确认
                                      │
                          基座与腕部估计安全融合
                                      │
                         再次 EvaluateTarget 安全检查
                                      │
                                      ▼
                    /strawberry/pick_and_place（Action）
                                      │
                    抓取 → 撤离 → 放置 → 验证 → 回全局位姿
                                      │
                           新鲜扫描后选择下一颗
```

### 3.2 三种身份不能混用

| 身份 | 谁产生 | 生命周期 | 能否用于正式控制 |
|---|---|---|---|
| `StrawberryDetection.target_id` | 单帧感知 | 一次采集内临时 | 只能作为来源诊断 |
| `TrackedTarget.track_id` | 三维跟踪器 | 跨帧稳定 | 是，当前控制身份 |
| Gazebo 果实实体 ID | 仿真世界 | 当前仿真实例 | 否，只用于物理映射和评测 |

当前广义 launch 明确设置：

```text
ground_truth_association_enabled: false
fruit_pose_source: tracked
```

Gazebo 真值可以生成场景、核对结果，并在真实双指接触后把匿名接触解析到仿真实体；
它不能替系统选择目标、纠正视觉位置或规划路径。

### 3.3 四层证据

| 层级 | 能证明什么 | 不能证明什么 |
|---|---|---|
| 代码阅读 | 设计意图 | 代码一定可运行 |
| 单元/契约测试 | 局部逻辑和接口 | 仿真物理成功 |
| 开发运行 | 某个已知开发场景的完整行为 | 未见种子上的总体性能 |
| 正式隐藏评测 | 冻结条件下的统计结论 | 评测范围之外的真实田间能力 |

---

## 单元 0：环境、Python 和安全基线

### 任务

建立可重复的 WSL 环境，确认当前工作区状态，并在不启动 ROS/Gazebo 的情况下跑完
依赖较少的测试。

### 必要基础

如果下面概念不熟，先补最小知识，不必系统学完整本书：

- Python：函数、类、`dataclass`、异常、列表/字典、回调；
- Shell：路径、引号、环境变量、退出码、前台/后台进程；
- Git：工作区、暂存区、提交、分支、远端；
- 测试：输入、预期输出、断言、回归。

### 环境命令

先在 Windows PowerShell 中进入项目专用发行版；不要依赖电脑当前的默认 WSL：

```powershell
wsl -d Ubuntu-24.04-URP
```

看到 Linux 提示符后，再执行：

```bash
cd "/mnt/c/Users/12753/Documents/New project/strawberry_urp"

export STRAWBERRY_COLCON_ROOT="$HOME/.cache/strawberry_urp/colcon"
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate

git status --short
git branch --show-current
git log -3 --oneline
```

不要把教材中的分支名当作固定答案。你应当解释终端实际显示的分支和未提交文件，
也不要覆盖与当前学习任务无关的修改。

若尚未构建：

```bash
bash scripts/build_and_test.sh
```

若已经构建，只跑依赖较少的完整回归：

```bash
/opt/strawberry_venv/bin/python scripts/run_pure_tests.py
```

当前审计记录是 735 项纯 Python 测试（0 failure、0 error、2 项按环境跳过），
以及 617 项 ROS/colcon 测试（0 failure、0 error、0 skip）。
后续项目增加测试后，总数可以变化；真正的通过条件是 failure/error 为 0，而不是死记
某个测试总数。

### 阶段作品

保存一份简短环境说明，写出 Windows/WSL 项目路径、Python 环境、构建产物根目录、
当前提交以及测试结果。

### 检查问题

1. 为什么构建产物放在 Linux 文件系统，而源码可以在 `/mnt/c`？
2. 为什么只 `source /opt/ros/jazzy/setup.bash` 还可能找不到本项目节点？
3. `git status` 出现未提交文件时，为什么不能直接重置工作区？

---

## 单元 1：先看见系统，再画出系统

### 任务

只启动双相机仿真，不执行机械臂采摘；确认两套 RGB-D、TF 和进程清理都正常。

### 安全实验

```bash
cd "/mnt/c/Users/12753/Documents/New project/strawberry_urp"
export STRAWBERRY_COLCON_ROOT="$HOME/.cache/strawberry_urp/colcon"

output="results/development/learning_dual_camera_$(date +%Y%m%d_%H%M%S)"
bash scripts/run_dual_camera_smoke.sh "$output" 228
echo "$output"
```

这个脚本不会发起采摘，并且输出目录不可覆盖。重点检查：

- `base_runtime_health.json` 与 `wrist_runtime_health.json`；
- `base_ready_rgb.png` 与 `wrist_ready_rgb.png`；
- `base_camera_tf.json` 与 `wrist_camera_tf.json`；
- `launch.log` 中是否存在未处理异常；
- 脚本退出后是否仍有残留 ROS/Gazebo 进程。

只读检查残留进程：

```bash
pgrep -af 'gz sim' || true
pgrep -af 'ros2 launch' || true
```

不要用宽泛的 `pkill` 代替理解清理范围。

### 阶段作品

根据实际输出画一张“节点—话题—坐标系”草图，至少包含底座相机、腕部相机、
`panda_link0`、`panda_hand` 和两个 optical frame。

### 检查问题

1. 为什么底座相机适合全局搜索，腕部相机适合近距离确认？
2. RGB、Depth 和 CameraInfo 为什么必须尺寸与时间相容？
3. “相机有图像”为什么不等于“目标已经能抓”？

---

## 单元 2：ROS 接口、身份、时间和异步通信

### 任务

不看实现，先从接口契约推导各模块能知道什么、不能知道什么。

### 精读入口

```text
ros2_ws/src/strawberry_interfaces/msg/StrawberryDetection.msg
ros2_ws/src/strawberry_interfaces/msg/TrackedTarget.msg
ros2_ws/src/strawberry_interfaces/msg/ObservationPlan.msg
ros2_ws/src/strawberry_interfaces/srv/EvaluateTarget.srv
ros2_ws/src/strawberry_interfaces/srv/MoveToObservation.srv
ros2_ws/src/strawberry_interfaces/action/PickAndPlace.action
```

在已加载 ROS 和工作区环境的终端运行：

```bash
ros2 interface show strawberry_interfaces/msg/TrackedTarget
ros2 interface show strawberry_interfaces/srv/EvaluateTarget
ros2 interface show strawberry_interfaces/action/PickAndPlace
```

### 必须理解的四种通信

| 方式 | 本项目例子 | 为什么使用它 |
|---|---|---|
| Topic | `/strawberry/tracked_targets` | 连续、多次、发布者不等待消费者 |
| Service | `/strawberry/evaluate_target` | 一次请求对应一次可行性回答 |
| Action | `/strawberry/pick_and_place` | 长耗时、有反馈、最后有结果 |
| TF | `panda_link0 → camera optical frame` | 表达不同坐标系之间的位姿关系 |

`Header` 的 `stamp` 和 `frame_id` 都是安全数据：前者判断是否陈旧、是否来自同一采集
时刻；后者判断一个三维点到底表达在哪个坐标系。数值相同但 frame 不同的两个点，
不能直接相减。

### 动手练习

画出以下三个消息之间的字段映射，并标出哪些字段不能直接复制：

```text
StrawberryDetection → TrackedTarget → TargetPose
```

然后运行接口契约测试：

```bash
/opt/strawberry_venv/bin/python -m unittest \
  discover -s ros2_ws/src/strawberry_interfaces/test -v
```

### 阶段作品

一张身份生命周期表：给定“同一颗草莓连续出现三帧，但单帧检测 ID 变化”的例子，
解释跟踪 ID 为什么仍应保持不变，以及完成 tombstone 如何防止它再次被采摘。

### 检查问题

1. Service 返回 `feasible=false` 和 Service 暂时不可用有什么区别？
2. 为什么 Action 需要 Feedback，而普通 Service 不适合完整抓放？
3. 检测 ID、跟踪 ID、仿真实体 ID 分别能流向哪些模块？

---

## 单元 3：TF、双相机、URDF/SDF 与物理场景

### 任务

解释一个相机像素如何经过内参、外参和机器人坐标链，成为
`panda_link0` 中的三维位置；同时理解视觉网格与碰撞体不是一回事。

### 最小数学基础

针孔反投影：

```text
X = (u - cx) × Z / fx
Y = (v - cy) × Z / fy
Z = Z
```

其中 `(u,v)` 是像素，`Z` 是深度，`fx,fy,cx,cy` 来自 CameraInfo。得到的点首先在
相机 optical frame 中，之后才通过 TF 转到 `panda_link0`。

你需要理解：

- 平移与旋转的组合顺序；
- 四元数用于表达旋转，不是四维位置；
- 底座相机相对基座固定，腕部相机会随 `panda_hand` 运动；
- URDF/Xacro 描述机器人关节与传感器安装；
- SDF 描述 Gazebo 世界、模型、传感器、visual 和 collision；
- Blender/网格资产负责外观，不应直接充当复杂精确碰撞体。

### 精读入口

```text
ros2_ws/src/strawberry_sim/urdf/
ros2_ws/src/strawberry_sim/launch/sim.launch.py
ros2_ws/src/strawberry_sim/worlds/
ros2_ws/src/strawberry_sim/models/
docs/dual-camera-development-v1.md
docs/assets/blender-asset-provenance-v2.md
```

不要按行通读整个 URDF。先使用 `rg` 找 frame、joint 和 camera：

```bash
rg -n "base_camera|wrist_camera|optical|panda_hand" \
  ros2_ws/src/strawberry_sim
```

### 动手练习

1. 从一次 CameraInfo 记录抄出内参，手算一个像素的相机坐标；
2. 从单元 1 的 TF JSON 说明底座和腕部相机分别由哪条坐标链连接到基座；
3. 在一个 SDF 模型中分别指出 visual、collision 和 sensor。

### 阶段作品

画出：

```text
panda_link0 → ... → panda_hand → wrist camera → wrist optical frame
panda_link0 → base camera mount → base optical frame
```

并用一句话说明错误外参会怎样同时破坏定位和运动安全。

### 检查问题

1. 为什么 optical frame 的轴方向不能凭肉眼猜？
2. 为什么移动腕部相机后必须重新使用新时间戳的 TF？
3. 为什么漂亮的植株网格不能直接等同于可靠碰撞模型？

---

## 单元 4：YOLO、RGB-D 定位与多目标跟踪

### 任务

跟踪一颗草莓从检测框到稳定三维轨迹，解释每一道拒绝门。

### 当前数据路径

```text
/camera/base/color/image_raw
  → /strawberry/base/detections
  → geometry-layer RGB-D 定位
  → MultiTargetTracker
  → /strawberry/tracked_targets
```

腕部使用独立话题，只确认当前目标：

```text
/camera/wrist/color/image_raw
  → /strawberry/wrist/detections
  → /strawberry/wrist/target_pose
```

### 精读顺序

1. `strawberry_perception/core.py`：检测结果转换和成熟度类别；
2. `strawberry_perception/perception_node.py`：ROS/Ultralytics 适配边界；
3. `strawberry_localization/generalized_depth.py`：深度层候选；
4. `strawberry_localization/generalized_node.py`：图像、深度、TF 和消息发布；
5. `strawberry_localization/tracking.py`：一对一三维匹配、平滑和 tombstone。

不要直接逐行读 `generalized_node.py`。先搜索调用点：

```bash
rg -n "geometry_layer|tracking_association|ground_truth_association|publish" \
  ros2_ws/src/strawberry_localization/strawberry_localization/generalized_node.py
```

### 关键概念

- CvBridge 给 Ultralytics 的 NumPy 图像使用项目规定的 BGR 适配，不能凭 ROS 图像名猜；
- 检测框中心可能落在叶片或背景上，因此广义模式使用深度分层和果实尺寸一致性；
- 检测、深度、CameraInfo 和 TF 必须满足 frame、尺寸、同步与新鲜度约束；
- `position_sigma_m` 是整条定位估计的不确定度，不应简单理解成单一深度 crop 的 MAD；
- 当前三维关联距离配置为 60 mm，稳定输出至少需要 3 次观察；
- 相关视频帧不能把系统偏差无限平均掉，因此跟踪不确定度有 5 mm 下限；
- 完成目标使用更窄的位置 tombstone，防止同一颗果实换 ID 后再次出现。

### 动手实验

```bash
/opt/strawberry_venv/bin/python -m unittest \
  discover -s ros2_ws/src/strawberry_perception/test -p 'test_core.py' -v

/opt/strawberry_venv/bin/python -m unittest \
  discover -s ros2_ws/src/strawberry_localization/test -p 'test_tracking.py' -v
```

阅读 `test_frame_local_ids_can_change_without_changing_track_ids` 和
`test_completed_track_is_a_persistent_position_tombstone`。先遮住断言，预测结果，再运行。

### 故障诊断练习

分别判断以下现象属于哪一层：

1. 图像能看到红果，但 detections 为空；
2. detections 有框，但深度有效像素不足；
3. 三维点存在，但 sigma 超过 15 mm；
4. 单帧 ID 每帧变化，但 track ID 稳定；
5. 已采摘果实重新生成了新 track ID。

### 阶段作品

写一份“一颗草莓的身份追踪报告”，包含检测时间戳、检测 ID、三维位置、sigma、
观察次数和稳定 track ID，并解释任何一次拒绝。

---

## 单元 5：安全目标选择与 MoveIt 零运动预评估

### 任务

解释为什么“最像成熟草莓”不等于“应该先抓”，并能根据候选数据预测选择结果。

### 精读入口

```text
ros2_ws/src/strawberry_bringup/strawberry_bringup/harvest_planning.py
ros2_ws/src/strawberry_bringup/strawberry_bringup/target_selector.py
ros2_ws/src/strawberry_manipulation/strawberry_manipulation/action_server.py
docs/decisions/0077-preflight-moveit-before-target-selection.md
```

先读纯逻辑 `rank_safe_targets()` 和 `target_rejection_reasons()`，再读 ROS 回调。

### 当前选择规则

候选首先必须同时满足：

- 成熟；
- 置信度达到冻结配置；
- 三维不确定度不高于 15 mm；
- 至少 3 次稳定观察；
- 数据年龄不超过 0.5 s；
- 与果实、收集箱等障碍保持最小间隙；
- 未被完成/跳过 tombstone 排除；
- MoveIt 证明当前状态到预抓取、接近、抓取和撤离路径连通可行。

只有通过全部门的目标才按以下顺序排序：更大间隙、更低不确定度、更高置信度、
更小真实关节移动量、稳定 track ID。

`/strawberry/evaluate_target` 只规划和碰撞检查，不执行轨迹。后台忙、Service 暂时
不可用和碰撞场景同步失败属于“延后评估”，不能缓存成“目标永久不可达”。

MoveIt 结果只在目标位置漂移不超过 5 mm 时复用；完成或跳过任一目标后，由于机械臂
当前状态已经变化，当前状态相关的可行性缓存全部失效。

### 动手实验

```bash
/opt/strawberry_venv/bin/python -m unittest \
  discover -s ros2_ws/src/strawberry_bringup/test \
  -p 'test_target_selector.py' -v

/opt/strawberry_venv/bin/python -m unittest \
  discover -s ros2_ws/src/strawberry_bringup/test \
  -p 'test_harvest_planning.py' -v
```

自己构造三颗纸面候选：

- A：置信度最高，但 MoveIt 不可行；
- B：可行、间隙较大、sigma 12 mm；
- C：可行、间隙较小、sigma 6 mm。

先预测最终顺序，再用测试中的 `HarvestCandidate` 调用 `rank_safe_targets()` 验证。

### 阶段作品

给出一份 `NO_PICK` 诊断，明确它是成熟度、数据质量、间隙、MoveIt 不可行，还是
后台暂时不可用；禁止只写“机械臂没抓到”。

### 检查问题

1. 为什么不能先按置信度选中，再让机械臂试一试？
2. 为什么后台 busy 不能等价成 unreachable？
3. 为什么目标完成后其他目标的 MoveIt 缓存也要失效？

---

## 单元 6：动态腕部观察、融合与抓放执行

### 任务

解释底座粗定位如何通过动态腕部观察得到近距离确认，以及确认后为什么还要再次检查
最终抓取路径。

### 当前过程

```text
选中目标
  → 生成有限组动态腕部观察位姿
  → 逐个检查观察位姿、IK、碰撞和连接到预抓取的路径
  → 移动到第一个安全视角
  → 只确认当前 track_id
  → 检查腕部修正距离、置信度和 sigma
  → 基座/腕部估计带系统误差下限融合
  → 对融合后的最终目标再次 EvaluateTarget
  → PickAndPlace Action
```

### 精读入口

```text
ros2_ws/src/strawberry_bringup/strawberry_bringup/harvest_planning.py
ros2_ws/src/strawberry_bringup/strawberry_bringup/harvest_orchestrator.py
ros2_ws/src/strawberry_manipulation/strawberry_manipulation/action_server.py
ros2_ws/src/strawberry_manipulation/strawberry_manipulation/moveit_backend.py
ros2_ws/src/strawberry_manipulation/strawberry_manipulation/core.py
```

重点搜索：

```bash
rg -n "generate_dynamic_views|fuse_position_estimates|WRIST_CONFIRMATION|FINAL_PICK" \
  ros2_ws/src/strawberry_bringup/strawberry_bringup

rg -n "evaluate_target|allow_target_contact|bilateral|attach|retreat|place" \
  ros2_ws/src/strawberry_manipulation/strawberry_manipulation
```

### 必须区分的三件事

1. **观察位姿可达**：相机能安全看到目标；
2. **目标抓取可行**：预抓取、接近、抓取、撤离连通；
3. **抓取执行成功**：控制器实测到位、双指接触、附着、放置和验证全部成功。

规划成功不代表执行成功；接触一个夹指也不代表稳定抓取；动作返回成功后仍需要检查
实际关节位置、箱内稳定和恢复状态。

### 证据阅读实验

阅读：

```text
config/generalized_runtime_progress_v14.json
docs/decisions/0077-preflight-moveit-before-target-selection.md
config/generalized_runtime_development_matrix_v1.json
```

回答：

- 为什么 track 1 没有进入批次重试？
- 为什么 track 2 能被选择？
- 腕部原始修正、融合修正和融合 sigma 分别是多少？
- 为什么一次 `SUCCESS` 仍不能证明单批次两果成功？

### 可选的有界开发运行

只有在已经完成单元 0～5、确认使用开发种子且理解机械臂会运动后，才运行完整开发
探针。下面使用已经公开调试过的开发种子 44008，不会打开正式隐藏矩阵：

```bash
cd "/mnt/c/Users/12753/Documents/New project/strawberry_urp"
export STRAWBERRY_COLCON_ROOT="$HOME/.cache/strawberry_urp/colcon"
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source "$STRAWBERRY_COLCON_ROOT/install/setup.bash"

scene_dir=".codex_tmp/learning_scene_44008_$(date +%Y%m%d_%H%M%S)"
ros2 run strawberry_sim generate_generalized_scene \
  --base-scene ros2_ws/src/strawberry_sim/config/scene.yaml \
  --base-world ros2_ws/src/strawberry_sim/worlds/strawberry_orchard.sdf \
  --output-dir "$scene_dir" --seed 44008 --profile mixed \
  --plant-count 2 --position-band middle --occlusion partial

tag="learning_$(date +%Y%m%d_%H%M%S)"
bash scripts/run_generalized_development_probe.sh 44008 "$scene_dir" "$tag"
```

运行后必须同时检查 `runtime_probe.json` 和 `cleanup_probe.json`。开发失败也是有效学习
证据，不得只保留成功截图。

### 阶段作品

从一次回执画出时间线：选择 → 观察 → 确认 → 最终评估 → PLAN → APPROACH →
GRASP → RETREAT → PLACE → VERIFY → DONE，并标出每一步的失败出口。

---

## 单元 7：连续采摘状态机与失败恢复

### 任务

说明为什么一颗草莓失败不会终止整个批次，同时证明系统不会无限重试或重复采摘。

### 当前批次状态机

```text
IDLE
  └─ start ─▶ SCANNING
                 ├─ 选中目标 ─▶ OBSERVING
                 │                 ├─ 成功 ─▶ CONFIRMING
                 │                 │            ├─ 成功 ─▶ PICKING
                 │                 │            │            ├─ 成功 ─▶ SCANNING
                 │                 │            │            └─ 失败 ─▶ 重观察/跳过
                 │                 │            └─ 失败 ─▶ 重观察/跳过
                 │                 └─ 失败 ─▶ 重观察/跳过
                 └─ 无安全目标/超时 ─▶ DONE
```

每个目标最多两次尝试，也就是只允许一次有依据的重观察。成功或最终跳过后发布完成
tombstone，跟踪器、选择器和碰撞场景共同抑制该目标。机械臂运动后必须通过新鲜扫描
屏障，旧缓存不能伪装成“已经重新观察”。

### 精读顺序

1. `harvest_sequence.py`：完全不依赖 ROS 的状态转移；
2. `test_harvest_sequence.py`：先从行为期望理解设计；
3. `harvest_orchestrator.py`：Topic/Service/Action 和定时器如何驱动纯状态机；
4. `development_probe.py`：怎样记录终态、进展和清理结果。

### 动手实验

```bash
/opt/strawberry_venv/bin/python -m unittest \
  discover -s ros2_ws/src/strawberry_bringup/test \
  -p 'test_harvest_sequence.py' -v
```

重点阅读并预测以下测试：

- `test_multiple_targets_continue_until_scan_finishes`；
- `test_one_failure_gets_exactly_one_retry_then_is_skipped`；
- `test_failed_target_does_not_stop_later_success`；
- `test_post_motion_barrier_rejects_cached_and_accepts_new_observation`；
- `test_no_candidate_is_safe_no_pick`。

### 阶段作品

给出三个纸面事件序列并手算最终 outcome：

1. 没有安全目标；
2. 第一颗失败两次、第二颗成功；
3. 两颗都成功。

然后用 `HarvestSequence` 写一个小测试验证你的答案。

### 检查问题

1. `NO_PICK`、`FAILED`、`PARTIAL_SUCCESS` 和 `SUCCESS` 分别表示什么？
2. 为什么失败目标只能重观察一次？
3. 为什么完成 tombstone 和新鲜扫描屏障缺一不可？

---

## 单元 8：测试驱动修改、故障归因和评测纪律

### 任务

完成第一次可审查的代码改动：改动范围小、有失败测试、有验证证据、不改变安全门。

### 第一次练习：先改测试，不动生产逻辑

在 `ros2_ws/src/strawberry_bringup/test/test_target_selector.py` 中增加边界测试：

- 评估位置漂移正好 5 mm 时允许复用；
- 漂移大于 5 mm 时拒绝复用；
- 负数或非有限阈值必须 fail-closed。

步骤：

1. 先阅读 `evaluation_matches_position()`；
2. 写下你预测的边界行为；
3. 增加测试并运行定向测试；
4. 如果测试暴露真实缺陷，再最小化修改纯函数；
5. 运行完整依赖较少回归；
6. 用 `git diff` 解释每一行修改的目的。

定向和完整测试：

```bash
/opt/strawberry_venv/bin/python -m unittest \
  discover -s ros2_ws/src/strawberry_bringup/test \
  -p 'test_target_selector.py' -v

/opt/strawberry_venv/bin/python scripts/run_pure_tests.py
```

第一次练习不要改成熟度、15 mm sigma、碰撞、关节限制、重试次数或正式评测配置。

### 第二次练习：只增加诊断，不改变决策

选择一个 `NO_PICK` 测试夹具，增加或改进机器可读诊断，让输出能区分：

- 数据门拒绝；
- MoveIt 路径不可行；
- MoveIt 暂时忙；
- 没有安全腕部视角。

先冻结决策结果，再验证新增诊断没有改变被选目标或安全阈值。

### 故障归因框架

看到失败时按顺序问：

1. **基础设施**：进程、节点、模型、话题、时钟是否就绪？
2. **感知**：检测框、类别、置信度是否正确？
3. **定位/跟踪**：深度、TF、sigma、track ID 是否有效稳定？
4. **选择**：成熟度、新鲜度、间隙和 MoveIt 预评估通过了吗？
5. **观察/确认**：腕部视角、身份、修正距离和融合通过了吗？
6. **规划/执行**：IK、碰撞、控制器终点、双指接触、附着和放置哪一步失败？
7. **恢复/清理**：机械臂是否回全局位姿，进程组是否 `CLEAN`？

### 评测纪律

- 开发种子可用于调试，但必须与正式隐藏种子集合隔离；
- 正式 30 种子在开发门通过前继续封存；
- 失败运行保留原始回执，不因结果不好而删除或重包装；
- 模型、配置、场景和输出使用哈希绑定；
- “工程豁免”不等于“指标通过”；
- 视觉封存图像与行为隐藏种子是两套不同的评测资产，不能混称。

### 阶段作品

提交一个最小补丁及说明，包含：问题、预测、失败测试、修改、定向测试、完整回归、
是否影响安全门、尚未验证的内容。

---

## 4. 结业任务：独立解释 v60，并设计无运动资格筛选

结业不要求你立刻解决“两果连续成功”，而要求你能独立设计一个不作弊、可复现的
下一步开发实验。

你需要交付：

1. 当前广义控制数据流图；
2. v60 中 track 1 被拒绝、track 2 成功的事件时间线；
3. 一份说明：为什么该结果证明了真值无关的单果完整抓放，却没有证明两果连续成功；
4. 一个开发场景计划：使用未参与调试的资格种子，先无运动筛选至少两颗成熟且
   MoveIt 可行的目标，不能读取正式隐藏种子；
5. 预先写好的成功标准、失败分类和清理检查；
6. 一个不放宽阈值的最小改动建议，并说明如何用测试验证。

达到下面标准才算“能改项目”：

| 能力层级 | 可观察行为 |
|---|---|
| L1 运行 | 能完成纯测试和无运动双相机 smoke，并找到证据 |
| L2 解释 | 能从消息、frame、track ID 解释完整数据链 |
| L3 诊断 | 能把失败定位到具体安全门或执行阶段 |
| L4 修改 | 能测试驱动地修改纯逻辑，并保持完整回归通过 |
| L5 实验 | 能设计开发验证，明确什么仍未被证明 |

---

## 5. 常见问题排查

| 现象 | 先检查什么 |
|---|---|
| `ros2` 找不到本项目包 | 是否 source ROS、虚拟环境和 colcon install overlay |
| Python 找不到 Ultralytics | 是否激活 `/opt/strawberry_venv`，入口脚本是否使用同一解释器 |
| 能看到节点但收不到话题 | 两个终端的 `ROS_DOMAIN_ID`、话题名和 QoS 是否一致 |
| 数据总是 stale | 是否使用仿真时间，header 时间戳和 TF 是否对应同一时间 |
| 输出目录已存在 | 选择新目录；受控脚本故意拒绝覆盖证据 |
| MoveIt 返回 busy | 当前是否有观察、抓取或恢复动作；不要缓存成不可达 |
| Gazebo 退出后仍占资源 | 先读取 PGID/进程，再使用脚本自带清理；不要盲目杀所有进程 |
| C 盘空间快速下降 | 检查 build/log、`.codex_tmp` 和开发结果；删除前确认范围和证据价值 |

---

## 6. 当前主线与历史基线的阅读边界

### 当前主线，优先阅读

```text
ros2_ws/src/strawberry_sim/strawberry_sim/generalized_scene.py
ros2_ws/src/strawberry_perception/strawberry_perception/perception_node.py
ros2_ws/src/strawberry_localization/strawberry_localization/generalized_node.py
ros2_ws/src/strawberry_localization/strawberry_localization/tracking.py
ros2_ws/src/strawberry_bringup/strawberry_bringup/harvest_planning.py
ros2_ws/src/strawberry_bringup/strawberry_bringup/target_selector.py
ros2_ws/src/strawberry_bringup/strawberry_bringup/harvest_sequence.py
ros2_ws/src/strawberry_bringup/strawberry_bringup/harvest_orchestrator.py
ros2_ws/src/strawberry_manipulation/strawberry_manipulation/action_server.py
ros2_ws/src/strawberry_manipulation/strawberry_manipulation/moveit_backend.py
```

### 历史基线，用来理解演进与回归

```text
ros2_ws/src/strawberry_bringup/strawberry_bringup/core.py
ros2_ws/src/strawberry_bringup/strawberry_bringup/orchestrator.py
ros2_ws/src/strawberry_bringup/strawberry_bringup/oracle_target_provider.py
ros2_ws/src/strawberry_bringup/strawberry_bringup/observation_sequence.py
ros2_ws/src/strawberry_manipulation/strawberry_manipulation/handoff_shadow.py
ros2_ws/src/strawberry_manipulation/strawberry_manipulation/pregrasp_shadow.py
早期 T30、P3、P4 与 field-v3 ADR
```

历史代码不是“错误代码”。它保留了固定场景回归、Oracle/Shadow 隔离和早期失败
证据；但学习当前多植株连续采摘时，不应让它取代广义主线。

---

## 7. 七个 ROS 包职责速查

| 包 | 当前主线职责 | 首选入口 |
|---|---|---|
| `strawberry_interfaces` | 消息、Service、Action 契约 | `msg/`、`srv/`、`action/` |
| `strawberry_sim` | 随机场景、Gazebo、双相机、物理接触 | `generalized_scene.py`、`sim.launch.py` |
| `strawberry_perception` | YOLO 成熟/未成熟检测 | `perception_node.py`、`core.py` |
| `strawberry_localization` | RGB-D 三维定位与多目标跟踪 | `generalized_node.py`、`tracking.py` |
| `strawberry_manipulation` | MoveIt 评估、动作规划和抓放执行 | `action_server.py`、`moveit_backend.py` |
| `strawberry_bringup` | 安全选择、腕部确认和连续批次编排 | `target_selector.py`、`harvest_orchestrator.py` |
| `strawberry_benchmark` | 指标、回执、正式验收 | `metrics.py`、`acceptance.py`、正式矩阵代码 |

---

## 8. 核心术语表

| 术语 | 本项目中的准确含义 |
|---|---|
| `panda_link0` | 广义控制目标最终统一表达的机械臂基座坐标系 |
| optical frame | 符合相机光学轴约定的传感器坐标系 |
| RGB-D | 彩色图、深度图和相机内参组成的观测 |
| TF | 带时间的坐标变换树 |
| ROI | 图像区域；当前广义控制不依赖固定 ROI |
| `track_id` | 三维跟踪器维护的跨帧稳定身份 |
| preflight | 不执行轨迹的 IK、碰撞和路径连通性检查 |
| tombstone | 已完成/跳过目标的持久抑制记录 |
| fresh-scan barrier | 机械臂运动后必须等待新观测，禁止复用运动前缓存 |
| fail-closed | 不确定或证据不足时保持静止/拒绝动作 |
| Oracle/Shadow | 历史真值控制与观察诊断隔离路径，不是当前广义控制输入 |
| ADR | 记录背景、决策、证据和后果的架构决策文档 |
| 开发种子 | 允许调试、但不能产生正式总体结论的场景种子 |
| 正式隐藏种子 | 冻结且未参与调试，只允许按合同执行正式评测的种子 |

这套教材的目标不是让你背下所有参数，而是让你在参数、代码或场景发生变化时，仍能
找到真实接口、提出可验证预测、保存证据，并清楚说明系统做到了什么、尚未做到什么。
