# 草莓成熟度识别与机械臂抓取项目：新项目总控交接

> 状态日期：2026-07-25  
> 原工作目录：`C:\Users\12753\Documents\New project\strawberry_urp`  
> 硬截止日期：2027-03-01  
> 用途：供新的 Codex 任务、VS Code 工作区或后续研究阶段直接接续。

## 1. 一句话结论

项目的 ROS 2、Gazebo、MoveIt、深度定位、状态机、抓取搬运、评测和
复现发布链路已经完成。球形草莓 v1 基线完成了正式 P3/P4/P5/P6，
但 P3 和 P4 的数值门失败；之后默认场景升级为 Blender 植株 v2，
新场景已完成一次成功的 Oracle 抓放，但当前 YOLO 只能稳定识别三颗
果实中的一颗成熟果。下一步不是重新搭架构，而是先做三颗果实的
隔离视觉诊断，并修正 v2 场景仍沿用 v1 定位半径的配置。

## 2. 必须先区分的两个仿真基线

### 2.1 v1：已冻结的球形草莓基线

- 场景：桌面、三个红/绿刚性球形果实、Panda、固定 RGB-D 相机、
  收集箱。
- 物理半径：35 mm。
- 作用：完成 T40/T60、正式 P3、P4、P5、P6。
- 正式结论只对 v1 有效。
- 归档场景：
  `ros2_ws/src/strawberry_sim/worlds/strawberry_tabletop_benchmark_v1.sdf`
- 归档清单：
  `ros2_ws/src/strawberry_sim/config/scene_tabletop_v1.yaml`

### 2.2 v2：当前默认的 Blender 植株基线

- 场景：花盆、草莓植株、叶片、叶柄、花序、果梗、两颗成熟果、
  一颗未成熟果、Panda、固定 RGB-D 相机、收集箱。
- 果实物理碰撞半径：26 mm。
- 当前默认世界：
  `ros2_ws/src/strawberry_sim/worlds/strawberry_orchard.sdf`
- 当前默认场景清单：
  `ros2_ws/src/strawberry_sim/config/scene.yaml`
- ADR：`docs/decisions/0036-adopt-blender-plant-scene-v2.md`
- v2 是 2026-07-25 新增的基线，尚未运行新的正式 P3/P4 矩阵。
- 禁止把 v1 的 39/135 正式结果写成 v2 的结果。

## 3. 固定范围与技术栈

- Windows 11 + WSL2。
- 发行版：`Ubuntu-24.04-URP`；另有用于干净复现的
  `Ubuntu-24.04-URP-Repro`。
- Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic、MoveIt 2。
- 机械臂：Franka Emika Panda。
- 固定式眼在手外 RGB-D 相机。
- 感知：YOLO11s，类别仅为 `RIPE` 和 `UNRIPE`。
- GPU：RTX 4060 Laptop，8 GB。
- 距离单位：米；角度：弧度；统一使用仿真时钟。
- 机器人基坐标系：`panda_link0`。

明确排除：

- 实体机器人控制；
- 仿真到现实迁移结论；
- 多相机融合；
- 自研运动规划器；
- 果柄剪切；
- 柔性叶片、柔性果实或果实损伤建模。

项目贡献应表述为：

> 可复现的草莓成熟度感知—三维定位—机械臂抓取搬运闭环与分层评测。

## 4. 对话与决策演进摘要

1. 冻结了 P0–P6 的分段计划、接口、指标、状态机、错误码和正式
   135+30 试验矩阵。
2. 新建 Ubuntu 24.04 WSL 环境，安装 ROS 2 Jazzy、Gazebo Harmonic、
   MoveIt 2，并建立七包 monorepo。
3. 用户下载并导入 Zenodo 草莓数据集、`yolo11s.pt` 和
   `yolo11n.pt`。
4. 完成数据校验、分组划分、训练、验证、阈值冻结和模型收据。
5. 历史 YOLO11s 未达到宏平均 F1 0.85；多轮受控优化仍未通过。
6. 完成两轮标签审核。冻结规则为“审阅分歧项一律保守排除”；
   不可判断成熟度、叶蒂完全挡住果实颜色的样本不得强行标成成熟或
   未成熟。
7. 用户允许使用未达标模型继续仿真工程。ADR 0026 将其定义为
   “工程豁免”，但没有把数值门改写为通过，也没有开放正式测试集。
8. 先使用 Oracle 真值完成定位、MoveIt、附着、抓放、状态机和异常
   归因，再接入感知控制。
9. v1 正式 P3 矩阵失败；P4 唯一一次仿真适配干预也失败；随后仍完成
   了带限制说明的 P5 发布与 P6 交付。
10. 发现旧场景的球形草莓过于简陋，用户提供两个 Blender 文件。
11. Blender 资产被转换为 Gazebo OBJ/MTL/SDF，默认场景升级为植株
    v2，并保留 v1 兼容场景。
12. v2 完成一次完整 Oracle 抓放；60 帧 Shadow 诊断显示只稳定识别
    `strawberry_1`，另一个成熟果和未成熟果未被识别。

## 5. 当前代码架构

ROS 包位于 `ros2_ws/src`：

| 包 | 职责 |
|---|---|
| `strawberry_interfaces` | 消息、Action、成熟度枚举、失败码 |
| `strawberry_sim` | Gazebo 世界、Panda、RGB-D、真值、接触与附着 |
| `strawberry_perception` | YOLO 推理与成熟度检测 |
| `strawberry_localization` | 深度中位数、相机投影、TF、目标关联 |
| `strawberry_manipulation` | MoveIt 场景、规划、抓放 Action |
| `strawberry_bringup` | 系统启动、Oracle、状态机和路由 |
| `strawberry_benchmark` | 场景矩阵、试验记录、指标和报告 |

总控架构文档：`docs/architecture.md`。

### 5.1 ROS 接口

相机：

- `/camera/color/image_raw`：RGB8；
- `/camera/depth/image_raw`：32FC1，单位米；
- `/camera/camera_info`。

项目接口：

- `/strawberry/detections`：
  `strawberry_interfaces/StrawberryDetectionArray`；
- `/strawberry/target_pose`：
  `strawberry_interfaces/TargetPose`；
- `/strawberry/pick_and_place`：
  `strawberry_interfaces/PickAndPlace` Action。

成熟度常量：

- `UNKNOWN=0`
- `RIPE=1`
- `UNRIPE=2`

稳定失败码：

1. `NO_TARGET`
2. `LOW_CONFIDENCE`
3. `DEPTH_INVALID`
4. `TF_TIMEOUT`
5. `UNREACHABLE`
6. `PLANNING_FAILED`
7. `COLLISION`
8. `GRASP_FAILED`
9. `PLACE_FAILED`
10. `STALE_DATA`

### 5.2 状态机

```text
INIT -> ACQUIRE -> DETECT -> LOCALIZE -> SELECT -> PLAN
     -> APPROACH -> GRASP -> RETREAT -> PLACE -> VERIFY
     -> DONE | FAILED
```

- 只选择成熟、深度有效、TF 有效、IK 可达目标。
- 无成熟果时返回 `NO_PICK`，机械臂不得运动。
- 深度或 TF 失败最多重取三帧。
- 规划失败只允许一次备用接近方向。
- 抓取失败后张开夹爪并安全结束，禁止无限重试。

### 5.3 Oracle 与 Shadow 隔离

- Oracle 控制：`/strawberry/oracle/target_pose`
- Shadow 检测：`/strawberry/shadow/detections`
- Shadow 定位：`/strawberry/shadow/target_pose`

Oracle 和 Shadow 不能使用同一个控制主题。仿真真值可以用于目标 ID
关联和评测，但不能替代感知算出的三维位置。

### 5.4 RGB/BGR 边界

ROS 相机主题保持 `rgb8`。Ultralytics 对 NumPy HWC 输入按 BGR 处理，
因此 perception adapter 在调用 YOLO 前必须通过 CvBridge 请求
`bgr8`。这个缺陷曾导致仿真中成熟果完全无法检测，已由 ADR 0006
修复。

## 6. 数据、标签和视觉模型

### 6.1 数据

- 来源：Zenodo record 6126677。
- 原始压缩包：
  `data/raw/zenodo_6126677/strawberries.zip`
- 732 张保留图像：
  - train：501
  - validation：116
  - sealed test：115
- 固定种子：`20260710`
- 类别 0/1 映射为成熟/未成熟；果柄类别保留在原始数据但不进入
  v1 主任务。

### 6.2 标签审核规则

- 框应紧贴完整可见果实主体，不包含大面积背景。
- 可以从颜色和可见果实判断时才标成熟或未成熟。
- 顶视图中叶蒂完全遮住果实颜色、无法可靠判断成熟度时：
  排除或记为不可判定，不强行二分类。
- 数据集没有绿框、模型红框却正确指向真实果实时：
  作为“疑似漏标”进入审核，不自动写回原标签。
- 两位审阅者分歧的项目一律保守排除。
- 原始标签从未被覆盖；所有修正均写入版本化派生数据集。

### 6.3 当前工程使用模型

```text
outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt
```

- SHA-256：
  `e3aca77e627469a1e9cf43c0776438623d881332163e03afe1fbcc55aba1af70`
- 图像尺寸：640
- 冻结阈值：0.58
- audited validation：
  - macro F1：0.800675
  - ripe F1：0.887064
  - unripe F1：0.714286
- 原始要求：macro F1 ≥ 0.85
- 科学状态：失败
- 工程状态：ADR 0026 允许在受限仿真中使用
- 正式 held-out test：未运行，继续封存

授权文件：`config/p3_perception_control_waiver_v1.json`。

普通 `system.launch.py` 的默认模型路径和阈值不是这份豁免模型。运行
感知控制或诊断时必须显式传入上面的模型和 `0.58`，或使用已经固定
这些值的脚本。

### 6.4 不得提升的仿真适配模型

```text
outputs/perception/yolo11s_640_sim_adapt_v1/weights/best.pt
```

- 合成集选择得分很好，但真实 audited validation 的
  macro/ripe/unripe F1 仅为约 0.626/0.754/0.497。
- ADR 0022 和 ADR 0032 已拒绝该模型。
- 不得把它设成默认模型，也不得把它称为通过。

## 7. v1 阶段结果

| 阶段 | 状态 | 关键结果 |
|---|---|---|
| P0 | 完成 | WSL/ROS/Gazebo/MoveIt、接口、稳定性 |
| P1 | 完成 | 场景、数据、732图像划分、内容哈希 |
| P2 | 工程完成，视觉豁免 | T40 和 Oracle 通过；T30 数值门失败 |
| P3 | 正式失败 | 39/135 成熟果试验成功，28.89% < 80% |
| P3 负例 | 通过 | 30/30 `NO_PICK`，无误摘未成熟果 |
| P4 | 干预完成但资格失败 | heavy 检测 300/300，目标位姿 0/300 |
| P5 | 完成 | 干净构建、冒烟、视频、报告和证据 |
| P6 | 完成 | 可校验 v1 发布压缩包 |

v1 关键指标：

- T40：100 个位置全部完成；
  - median：1.345 mm
  - p95：1.897 mm
- T60 Oracle：10/10，规划 p95 约 0.058841 s。
- 正式 P3：
  - 正例成功：39/135
  - 感知失败：84
  - 抓取失败：12
  - 负例安全：30/30
- P4 heavy 条件定位失败根因：
  检测框中心深度采到了前景遮挡物，而不是果实表面。

## 8. Blender v2 场景实现

### 8.1 用户提供的源文件

原始 Desktop 文件：

- `strawberry_ripe_visual_v1.blend`
- `Untitled1.blend`

仓库内归档为：

```text
assets/blender_sources/strawberry_ripe_visual_v1.blend
assets/blender_sources/strawberry_plant_v2.blend
```

SHA-256：

- 果实：
  `bd81d003f33f4ae35abc19fdc8192538cb0957f14507a53144126b41f84d3116`
- 植株：
  `399fbc5b0aab7aa459be1f89d5a004563ca5502c8735a2e43b303dbdaa7c8166`

### 8.2 转换与职责边界

转换脚本：

```text
tools/blender/export_gazebo_assets.py
```

检查脚本：

```text
tools/blender/inspect_blend_scene.py
```

导出格式：OBJ + MTL，再由 SDF 封装为 Gazebo 模型。

- 果实拥有：果体、种子、萼片中心、七片萼叶。
- 排除果实原文件中的长弯曲茎，防止与植株果梗重复。
- 植株拥有：冠部、叶片、叶脉、叶柄、花序、果梗。
- 排除植株 `.blend` 中导入的模板和已经复制进去的果实。
- 果实模型原点设在果体中心。
- 以萼片中心连接 Blender 果梗端点。

轻量化结果：

- 每颗果实：18,150 三角面；
- 植株：17,129 三角面；
- 三果实加植株：71,579 三角面；
- 轻量化前约 306,316 三角面。

资产溯源：
`docs/assets/blender-asset-provenance-v2.md`。

### 8.3 v2 场景对象

| ID | 成熟度 | 初始位置 `panda_link0` |
|---|---|---|
| `strawberry_1` | RIPE | `[0.419064753, -0.053479813, 0.546111838]` |
| `strawberry_2` | UNRIPE | `[0.550333202, 0.118588343, 0.532157422]` |
| `strawberry_3` | RIPE | `[0.578895651, 0.068192676, 0.540231110]` |

- Blender 有第四个果梗，但当前保持为空，以维持原三目标接口。
- 果实采用 26 mm 球形碰撞代理。
- 花盆和植株冠部进入 Gazebo/MoveIt 碰撞场景。
- 叶片和茎只作为视觉遮挡，不参与柔性接触或损伤模拟。
- 果实进入收集箱并稳定 1 秒即判成功。

### 8.4 v2 验证结果

静态与构建：

- 六个依赖包重新构建成功。
- dependency-light 测试：378 项运行，377 通过，1 项条件跳过。
- SDF 模型检查通过。
- 8 秒运行健康检查收到颜色、深度、内参、关节和三路真值。
- 轻量化后 8.054 秒墙钟推进 2.915 秒仿真时间。

Oracle 抓放：

```text
results/development/blender_scene_v2_oracle_smoke_v4/trial_01.json
```

- 1/1 成功；
- `target_id=1`；
- 双指真实接触；
- 附着、抬升、搬运、放置和验证完成；
- 未误碰其他果实；
- planning time：0.0332 s；
- execution time：103.9149 s。

Shadow 视觉：

```text
results/development/blender_scene_v2_shadow_60f_v1/shadow_window.json
```

- 10 帧预热后采 60 帧；
- 任意检测：60/60；
- 至少一颗成熟果：60/60；
- 未成熟果：0/60；
- 目标位姿：60/60；
- 每帧恰好一个成熟框；
- 始终关联 `target_id=1`；
- 置信度约 0.652302；
- `strawberry_3` 和 `strawberry_2` 未被识别。

结论：v2 将“仿真中没有成熟检测”改善为“稳定检测一颗成熟果”，
但还没有实现全场景识别。

## 9. 当前已知的不一致和风险

### 9.1 v2 定位偏移仍使用 v1 半径

当前文件：

```text
ros2_ws/src/strawberry_localization/config/localization.yaml
```

仍配置：

```yaml
surface_to_center_offset_m: 0.035
```

而 v2 果实碰撞半径已经是 `0.026`。`localization_gate.py` 和部分测试
也仍冻结为 `0.035`。这不会影响“是否发布 TargetPose”的 Shadow
统计，但会给 v2 三维中心估计引入约 9 mm 的系统差异。

新项目在做任何 v2 定位精度或正式矩阵前必须：

1. 保留 v1 专用的 35 mm 配置；
2. 新建 v2 专用 26 mm 配置或显式 launch 参数；
3. 对 v2 重新运行至少 100 个真值位置的定位门；
4. 不得用旧 v1 的 1.345/1.897 mm 指标代替 v2 指标。

### 9.2 Git 尚未形成可迁移基线

- 当前目录是 Git worktree，但 `HEAD` 尚不存在。
- 几乎所有项目文件仍显示为未跟踪。
- VS Code 的“活动更改过多”提示来自这个状态。
- 新项目不能依赖 commit SHA 恢复当前成果。

迁移前应先：

1. 排除 `build/`、`log/`、`results/`、缓存和临时 Blender；
2. 检查大文件策略；
3. 创建首次有意义的 Git 提交；
4. 为 v2 打标签或生成独立交付包。

### 9.3 旧 P6 压缩包不包含 Blender v2

现有文件：

```text
artifacts/p6/strawberry_urp_release_v1.zip
```

- 生成日期：2026-07-24；
- SHA-256：
  `0f306c8503bc54b03b16304a428387cdf5cf282661efd4323066fe53fd68d22a`
- 大小：78,598,403 bytes；
- CRC：通过。

它不包含：

- ADR 0036；
- `strawberry_plant_v2`；
- Blender 源文件；
- v2 Shadow runner；
- v2 Oracle/Shadow 结果。

若新项目要继续 v2，必须复制当前工作目录的源文件和必要工件，或先
生成新的 v2 交付包；不能只解压旧 P6。

### 9.4 视觉仍是当前主要瓶颈

- `strawberry_1` 稳定识别；
- `strawberry_3` 未识别；
- `strawberry_2` 未识别；
- 尚不能区分是自然叶片遮挡、果实尺度/朝向、绿色未成熟果与叶片
  混淆，还是模型域差异。

### 9.5 科学边界

- T30 0.85 数值门仍失败；
- held-out real test 仍封存；
- v1 P3 正式失败；
- v1 P4 干预失败；
- v2 尚无正式 P3/P4；
- 不允许实体机器人或 sim-to-real 声明；
- 不允许把工程豁免写成算法达标。

## 10. 新项目应保留的核心文件

必须保留：

```text
README.md
NEW_PROJECT_HANDOFF.md
docs/
config/
requirements/
scripts/
tools/
ros2_ws/src/
assets/blender_sources/
outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt
```

需要重训或重做数据时再保留：

```text
data/raw/zenodo_6126677/strawberries.zip
data/processed/
weights/yolo11s.pt
weights/yolo11n.pt
outputs/perception/
```

需要保留历史科学证据时：

```text
artifacts/
results/
docs/decisions/
```

可重新生成、不应作为主要迁移源：

```text
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
build/
log/
.pytest_cache/
.codex_tmp/
runs/
```

`.codex_tmp` 中有本次导出使用的 Blender 5.2 便携版，约 1.4 GB，
已被 `.gitignore` 排除。OBJ/MTL 已经生成，普通运行不依赖该目录。

## 11. 常用复现命令

进入 WSL：

```powershell
wsl -d Ubuntu-24.04-URP
```

进入项目：

```bash
cd "/mnt/c/Users/12753/Documents/New project/strawberry_urp"
```

### 11.1 完整构建与测试

默认构建目录位于 Linux 用户缓存：

```bash
bash scripts/build_and_test.sh
```

若希望使用仓库内现有构建：

```bash
export STRAWBERRY_COLCON_ROOT="$PWD/ros2_ws"
bash scripts/build_and_test.sh
```

### 11.2 查看当前 v2 场景

```bash
bash scripts/show_canonical_scene.sh
```

### 11.3 启动基础仿真

```bash
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
source ros2_ws/install/setup.bash
ros2 launch strawberry_sim sim.launch.py headless:=false
```

### 11.4 v2 60 帧无运动 Shadow

默认输出路径只能使用一次。再次运行时必须传入新目录：

```bash
bash scripts/run_blender_scene_v2_shadow_window.sh \
  "$PWD/results/development/blender_scene_v2_shadow_60f_v2"
```

该脚本固定：

- 当前豁免模型哈希；
- 阈值 0.58；
- 10 帧预热；
- 60 帧测量；
- 无 Oracle；
- 无 MoveIt；
- 无机械臂运动。

### 11.5 Oracle 抓放

```bash
export STRAWBERRY_COLCON_ROOT="$PWD/ros2_ws"
bash scripts/run_oracle_pick_gate.sh \
  1 220 \
  "$PWD/results/development/blender_scene_v2_oracle_smoke_next"
```

### 11.6 重导出 Blender 资产

说明：
`tools/blender/README.md`

主脚本：

```text
tools/blender/export_gazebo_assets.py
```

需要 Blender 5.2.0 LTS 或兼容版本。不要直接导出整个植株场景，
否则会重新引入模板果实和重复长茎。

## 12. 建议的新项目任务顺序

### 第一步：建立可迁移的 v2 基线

1. 从当前工作目录复制源代码、v2 Blender 源、生成网格、当前模型和
   必要证据。
2. 不使用旧 P6 压缩包作为 v2 唯一来源。
3. 建立干净 Git 首次提交。
4. 记录环境版本和当前文件哈希。

### 第二步：修复 v1/v2 几何参数分层

1. 保留 v1 `0.035 m` 定位配置。
2. 新建 v2 `0.026 m` 定位配置。
3. 让 launch 根据场景显式选择配置，禁止隐式混用。
4. 更新架构文档中只描述 35 mm 球形果实的段落。
5. 运行新的 v2 100 位置定位门。

### 第三步：三果实隔离视觉诊断

保持相机、模型、阈值和光照不变，机械臂不动，分别生成：

1. 只显示 `strawberry_1`；
2. 只显示 `strawberry_3`；
3. 只显示 `strawberry_2`。

每个场景预热 10 帧后采 60 帧，记录：

- 成熟/未成熟检测帧率；
- 每帧框数；
- 置信度；
- 关联目标 ID；
- TargetPose 可用率。

判定逻辑：

- 隔离后 3号或2号能识别：主因是叶片遮挡、相机角度或相邻目标；
  优先调整场景/相机，不重训。
- 隔离后3号成熟果仍不识别：检查尺度、朝向、萼片遮挡和材质域差异。
- 隔离后2号未成熟果仍不识别：确认是未成熟类别/绿色叶片混淆，
  再冻结有限合成数据方案。

### 第四步：只处理已定位的瓶颈

- 若为遮挡：调整叶片姿态、果实挂点或固定相机，使任务可见但仍保留
  合理自然遮挡。
- 若为材质：增加有限颜色、光照和纹理变化。
- 若确实需要微调：先冻结新 ADR、训练集、验证集、真实数据非退化
  规则和一次性训练预算；不要盲目反复重训。

### 第五步：重新进入 v2 集成门

依次执行：

1. v2 定位 100 点；
2. v2 Oracle 抓放至少 10 次，要求 ≥90%；
3. v2 感知控制的简单场景冒烟；
4. v2 多位置、自然遮挡无运动诊断；
5. 满足前置门后，另行冻结新的 v2 正式 P3/P4 定义。

## 13. 新项目中禁止直接做的事

- 不要把 v1 正式矩阵结果冒充 v2 结果。
- 不要写“YOLO 已达到 0.85”；真实结果是 0.800675。
- 不要开启 held-out real test。
- 不要重新运行已经消费的一次性 P4 干预并称为同一正式试验。
- 不要把被拒绝的 simulator-adaptation 模型设成默认权重。
- 不要删除 v1 兼容场景；它仍是历史证据的复现基础。
- 不要在未修正 26/35 mm 参数边界前宣称 v2 定位精度通过。
- 不要因为单次 v2 Oracle 抓放成功就宣称视觉端到端完成。

## 14. 新 Codex 任务可直接粘贴的首条消息

```text
请先阅读：
1. NEW_PROJECT_HANDOFF.md
2. docs/architecture.md
3. docs/decisions/0036-adopt-blender-plant-scene-v2.md
4. docs/blender-scene-v2-shadow-diagnostic-v1.md

当前默认基线是 Blender 植株 v2，不是旧球形 v1。旧 P3/P4 正式结果
只能作为 v1 历史证据。当前工程模型是
outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt，
SHA-256 为
e3aca77e627469a1e9cf43c0776438623d881332163e03afe1fbcc55aba1af70，
阈值 0.58。它在 v2 固定场景中 60/60 帧识别 strawberry_1，但没有
识别 strawberry_3 和 strawberry_2。

下一任务先不要训练模型，也不要运行正式测试。请先：
1. 将 v1 的35 mm和v2的26 mm定位偏移拆成显式场景配置；
2. 为三颗果实分别建立隔离、无运动、10帧预热+60帧测量诊断；
3. 根据隔离结果判断是遮挡/相机问题还是模型域问题；
4. 保留所有旧证据和正式测试封存边界。
```

## 15. 权威文件索引

- 总体状态：`README.md`
- 本交接：`NEW_PROJECT_HANDOFF.md`
- 架构：`docs/architecture.md`
- 实施任务约定：`docs/task-handoff.md`
- v2 决策：`docs/decisions/0036-adopt-blender-plant-scene-v2.md`
- v2 资产溯源：`docs/assets/blender-asset-provenance-v2.md`
- v2 Shadow 结果：
  `docs/blender-scene-v2-shadow-diagnostic-v1.md`
- 当前工程模型授权：
  `config/p3_perception_control_waiver_v1.json`
- v1 P3 结果：
  `artifacts/p3/p3_formal_matrix_outcome_handoff_v1.json`
- v1 P4 结果：
  `artifacts/p4/p4_sim_adapt_qualification_outcome_handoff_v1.json`
- P5：`artifacts/p5/p5_final_release_handoff_v1.json`
- P6：`artifacts/p6/p6_delivery_handoff_v1.json`

