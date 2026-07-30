# 草莓采摘 URP

[English](README.md) | [简体中文](README.zh-CN.md)

这是一个可复现的草莓成熟度识别、三维定位与机械臂抓取搬运仿真项目。系统运行在 ROS 2、Gazebo 和 MoveIt 2 之上，使用 YOLO11s 区分成熟与未成熟草莓，并控制 Franka Emika Panda 机械臂完成安全预抓取、接近、夹取、搬运、放置和恢复。

当前提交版已经在固定的 field-v3 草莓植株场景中跑通以下工程链路：

```text
底座全局相机
    ↓ 发现并选择目标区域
夹爪附近的腕部 RGB-D 相机
    ↓ YOLO 成熟度识别
深度图 + 相机内参 + TF
    ↓ 三维目标位置
MoveIt 2 碰撞检测与运动规划
    ↓
接近 → 抓取 → 搬运 → 放置 → 验证 → 恢复
```

> 当前结论是“固定场景下可工作的工程仿真系统”，不是通用田间采摘、真实机械臂或果实无损采摘证明。项目保留了所有未通过的正式指标与限制。

## 快速导航

- 新任务或新工作区接手项目：[`NEW_PROJECT_HANDOFF.md`](NEW_PROJECT_HANDOFF.md)
- 中文系统学习路线：[`docs/learning-roadmap-zh.md`](docs/learning-roadmap-zh.md)
- 系统架构与接口：[`docs/architecture.md`](docs/architecture.md)
- 环境复现说明：[`docs/reproduction.md`](docs/reproduction.md)
- field-v3 场景集成：[`docs/field-v3-integration.md`](docs/field-v3-integration.md)
- 最终项目总结：[`docs/submission-report.md`](docs/submission-report.md)
- 提交检查清单：[`docs/submission-checklist.md`](docs/submission-checklist.md)
- submission-v2 视频脚本：[`docs/submission-video-storyboard-v2.md`](docs/submission-video-storyboard-v2.md)

## 项目基线

- 操作系统：Ubuntu 24.04（WSL2）
- 中间件：ROS 2 Jazzy
- 仿真器：Gazebo Harmonic
- 运动规划：MoveIt 2
- 机械臂：Franka Emika Panda
- 视觉模型：YOLO11s
- 识别类别：`RIPE`、`UNRIPE`
- 相机方案：底座全局相机 + 腕部 RGB-D 相机
- 当前主场景：`field-v3`
- 项目截止日期：2027-03-01

ROS 2 工作空间位于 `ros2_ws/src`。数据集、训练权重、构建目录和大型运行日志通常不进入 Git，由固定清单、哈希和脚本进行复现或校验。

## 目录结构

| 路径 | 内容 |
|---|---|
| `ros2_ws/src/strawberry_sim` | Gazebo 世界、Panda 描述、草莓与植株模型、相机和控制器配置 |
| `ros2_ws/src/strawberry_perception` | YOLO 推理、类别与置信度输出 |
| `ros2_ws/src/strawberry_localization` | RGB-D 三维定位、相机内参与 TF 变换 |
| `ros2_ws/src/strawberry_manipulation` | MoveIt 规划、抓取状态机、接触与附着处理 |
| `ros2_ws/src/strawberry_bringup` | 系统总启动文件与场景配置 |
| `ros2_ws/src/strawberry_benchmark` | 评测、收据和结果检查 |
| `ros2_ws/src/strawberry_interfaces` | ROS 消息和动作接口 |
| `config` | 项目级冻结配置与实验约束 |
| `scripts` | 环境安装、构建测试、演示、报告和打包脚本 |
| `docs` | 架构、决策记录、实验报告、学习路线和交接文档 |
| `results` | 各阶段运行结果和诊断证据 |
| `artifacts` | 汇总指标、最终报告、视频、收据和提交包 |

## 场景与相机方案

### Blender 植株场景

2026-07-25 起，默认开发场景从彩色球体替换为项目提供的草莓植株和果实模型。场景包含叶片、花序、果梗以及三个可交互草莓。果实作为独立刚体挂接在植株上，可用于接触、夹取、附着、搬运和释放测试。

旧的桌面场景仍作为兼容性测试保留。Blender-v2 和 field-v3 属于新的仿真基线，不能用它们覆盖或重写早期 P3/P4 的正式结果。

### 双相机协作

项目支持三种相机配置：

- `camera_mount:=fixed`：历史固定相机。
- `camera_mount:=wrist`：机械臂腕部 RGB-D 相机。
- `camera_mount:=dual`：底座全局相机与腕部 RGB-D 相机同时存在。

双相机模式采用顺序协作：

1. 底座相机提供全局场景与目标区域。
2. 机械臂移动到观察姿态。
3. 腕部相机稳定后进行近距离识别和深度定位。
4. 目标坐标转换到机械臂规划坐标系。
5. 通过安全门后才允许生成或执行抓取规划。

这不是多相机点云融合。腕部相机是精确定位输入，底座相机主要负责全局观察和展示。两个相机外壳都加入 MoveIt 碰撞模型。

### field-v3 新场景

用户提供的 `st1.blend` 被整理为可选的 `field-v3` 场景，没有替代已冻结的 Blender-v2 基线。该场景保留了 101 个背景植株实例，并在外侧行设置一套经过验证的 Panda 工作单元。

field-v3 已完成：

- 底座相机全局观察；
- 腕部相机近距离成熟草莓识别；
- 深度与 TF 三维定位；
- 场景碰撞对象同步；
- 控制器无运动预抓取规划；
- 感知驱动的完整抓取和放置；
- 左右夹指原始与处理后接触确认；
- 果实 `attached → detached` 状态确认；
- 夹爪重新张开；
- 机械臂返回 `ready`。

三个连续的固定场景开发运行均完成：

```text
PLAN → APPROACH → GRASP → RETREAT → PLACE → VERIFY → DONE
```

field-v3 使用两个独立的单关节夹爪控制器。这是因为 DART 不会自动执行 Panda 右夹指的 mimic 约束。控制后端会同时发送左右夹指目标，并分别校验实际位置。

## 已修复的关键问题

### 初始姿态偶发错误

此前出现过“有一次腕部相机效果很好，但另一次只看到地面”的现象。根因不是视觉模型随机失效，而是两个不同的 `robot_state_publisher` 同时使用公共 `/robot_description`，Gazebo 生成器和 `gz_ros2_control` 控制器管理器会根据启动时序读到不同的机器人描述。

提交版采用以下修复：

- 仿真生成器使用专用 `/strawberry/sim/robot_description`；
- `gz_ros2_control` 明确读取仿真机器人的描述节点；
- 启动文件在 xacro 展开后再次写入 field-v3 初始关节值；
- headed runner 会检查 Gazebo 实际加载的关节值；
- 如果加载成默认姿态，运行会提前失败，不再生成具有误导性的演示。

因此，最终提交演示的观察姿态不再依赖随机启动顺序。

### 视觉颜色通道

早期 YOLO Shadow 测试把 RGB NumPy 图像直接传给 Ultralytics 的 BGR NumPy 输入路径，导致成熟目标无法正确输出。ADR 0006 修复了该接口边界。

### 三维定位偏差

Blender-v2 草莓网格曾存在面朝向错误。机械修正三角面顺序后，在相同的 100 个位置上，定位误差从约 44 mm 降至：

- 中位数：`4.503614 mm`
- P95：`5.045998 mm`
- 最大值：`5.196535 mm`

另一个约 29 mm 的自然植株定位误差来自 ROS 参数路由问题。显式传入 26 mm 表面到果心偏移后，同一目标误差下降到约 4.85 mm。

### 夹爪控制

原 `0.25 s` 停滞窗口可能在 Gazebo 产生第一帧夹指位移前超时。修复后：

- 停滞窗口延长至 `1.0 s`；
- 控制器结果为空时读取实时关节状态；
- 到达判断与控制器 `3 mm` 容差一致；
- 实际闭合位移小于 `2 mm` 时仍会安全拒绝。

隔离测试记录了 `14.138 mm` 的闭合位移、左右接触、重新张开与机械臂恢复。

## 当前阶段状态

| 阶段 | 状态 | 结论 |
|---|---|---|
| P0 工程基线 | 完成 | WSL2、ROS、Gazebo、MoveIt 冒烟测试和 30 分钟稳定性测试完成 |
| P1 仿真与数据 | 完成 | 仿真接口、固定下载、732 张图像划分和内容约束已准备 |
| P2 模块实现 | 工程完成，带指标豁免 | Oracle 抓取与 T40 定位通过；模型用于有限仿真控制，但宏 F1 未达到 0.85 |
| P3 端到端集成 | 正式门槛失败 | 135 个正样本场景成功 39 个；30 个仅未成熟草莓场景全部安全 `NO_PICK` |
| P4 鲁棒性干预 | 完成，但资格测试失败 | 重遮挡下成熟检测 300/300，但三维目标位姿 0/300 |
| P5 发布 | 完成并冻结 | 构建、测试、图表、报告、演示视频和证据清单完成 |
| P6 交付 | 完成 | 确定性压缩包、内嵌清单、CRC 与 SHA-256 校验完成 |

## 最新验证结果

截至 2026-07-29：

- 7 个 ROS 2 包全部构建成功；
- `396` 项 colcon 测试全部通过；
- 错误：`0`；
- 失败：`0`；
- 跳过：`0`；
- field-v3 最终演示结果：`SUCCESS`；
- 最终演示包含 `TARGET_READY`、`PLAN`、`APPROACH`、`GRASP`、`RETREAT`、`PLACE`、`VERIFY`、`DONE`；
- 左右夹指均记录到有效接触；
- 附着事件顺序为 `[true, false]`；
- 最终 H.264 视频时长为 270 秒。

其他已冻结工程结果：

- T40 定位门：100/100，通过；中位误差 `1.345 mm`，P95 `1.897 mm`。
- T60 Oracle 集成门：10/10，通过；规划 P95 `0.058841 s`。
- 单姿态感知开发门：10/10 成熟目标成功，10/10 仅未成熟目标安全 `NO_PICK`。
- 独立干净 WSL 复现：246/246 测试通过，10/10 发布冒烟行为通过。
- 自然植株单次感知抓取：完整完成七个动作阶段并返回 `ready`。
- field-v3 固定场景：三个连续感知抓取循环成功。

## 必须保留的科研边界

### YOLO 指标

当前接受的工程运行模型是：

```text
outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt
```

对应 SHA-256：

```text
e3aca77e627469a1e9cf43c0776438623d881332163e03afe1fbcc55aba1af70
```

其审计验证集指标为：

| 指标 | 结果 |
|---|---:|
| 宏 F1 | `0.800675` |
| 成熟 F1 | `0.887064` |
| 未成熟 F1 | `0.714286` |
| 原目标门槛 | `0.85` |
| 是否通过原数值门槛 | 否 |

ADR 0026 仅允许将这些固定字节和阈值 `0.58` 用于受限仿真工程控制，不会把失败指标改写为通过。

### 正式 P3 矩阵

正式 P3 包含：

- 3 种光照；
- 3 种遮挡；
- 5 个目标位置；
- 3 个 Gazebo 随机种子；
- 共 135 个成熟目标正场景；
- 30 个仅未成熟目标负场景。

结果：

| 项目 | 结果 |
|---|---:|
| 正场景成功 | `39/135` |
| 正场景成功率 | `28.89%` |
| 原通过门槛 | `80%` |
| 负场景安全 `NO_PICK` | `30/30` |
| 感知失败 | `84` |
| 抓取失败 | `12` |

因此 P3 正式门槛失败。不能用 field-v3 的固定场景成功替换这组结果。

### P4 重遮挡

P4 干预模型在重遮挡下恢复了 `300/300` 成熟检测帧，但产生 `0/300` 有效三维目标位姿。原因是检测框中心深度落在前景遮挡物上，重建点距离最近果实约 0.345–0.366 m。因此该候选被拒绝，未继续执行运动矩阵。

### 禁止扩大的结论

当前项目没有证明：

- 任意植株、任意姿态或任意果实均可稳定采摘；
- 真实机械臂已经运行；
- 对果实、果梗或植株无损；
- 已完成剪梗；
- 已完成仿真到现实迁移；
- 已通过封存的真实测试集；
- 已达到正式 P3 或 P4 门槛。

## 数据与模型

需要放到本地的三个固定资源如下：

| 文件 | 本地位置 | 完整性 |
|---|---|---|
| [`strawberries.zip`](https://zenodo.org/records/6126677/files/strawberries.zip?download=1) | `data/raw/zenodo_6126677/strawberries.zip` | MD5 `db8d5dcb4b8adebf1621788373fd3031` |
| [`yolo11s.pt`](https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11s.pt) | `weights/yolo11s.pt` | SHA-256 `85a76fe86dd8afe384648546b56a7a78580c7cb7b404fc595f97969322d502d5` |
| [`yolo11n.pt`](https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n.pt) | `weights/yolo11n.pt` | SHA-256 `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1` |

整理后的真实图像划分为：

- 训练集：501 张；
- 验证集：116 张；
- 封存测试集：115 张。

封存测试集没有被打开或消费。现有报告不包含正式测试集成绩。

项目还进行了验证集标签审计、未成熟样本重复曝光训练和一次干净标签训练。所有候选模型都按冻结规则进行选择；没有通过 `0.85` 门槛的候选不会被宣称为合格模型。

详细证据：

- `artifacts/perception/t30_validation_gate_summary.json`
- `artifacts/perception/t30_audited_validation_handoff_v1.json`
- `artifacts/perception/t30_opt2_outcome_handoff_v1.json`
- `artifacts/perception/t30_train_audit_outcome_handoff_v1.json`

## 环境安装与构建

完整步骤以 [`docs/reproduction.md`](docs/reproduction.md) 为准。

在干净的 Ubuntu 24.04 WSL2 中，先以 `root` 执行一次：

```bash
bash scripts/bootstrap_ubuntu_2404.sh
```

然后以普通用户在仓库根目录执行：

```bash
bash scripts/build_and_test.sh
bash scripts/run_system.sh headless:=false
```

无界面运行：

```bash
bash scripts/run_system.sh headless:=true
```

检查 CUDA、OpenCV、cv_bridge 和 YOLO 环境：

```bash
bash scripts/verify_environment.sh
```

如果 Python 网络安装不稳定，可先在 Windows PowerShell 中准备固定 Linux wheelhouse：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/cache_perception_wheels.ps1
```

## 运行 field-v3 演示

构建产物默认位于：

```text
$HOME/.cache/strawberry_urp/colcon
```

启动新的 headed 感知抓取演示：

```bash
export STRAWBERRY_COLCON_ROOT="$HOME/.cache/strawberry_urp/colcon"

bash scripts/run_field_v3_perception_pick_headed.sh \
  results/development/field_v3_perception_pick_headed_local \
  outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt \
  230 \
  60 \
  true \
  false
```

六个位置参数依次表示：

1. 新的输出目录；
2. 接受的模型路径；
3. ROS Domain ID；
4. 完成后的窗口保留秒数；
5. 是否显示窗口；
6. 是否录制原始演示视频。

输出目录必须不存在，脚本不会覆盖已有证据。使用录制模式时将最后一个参数改为 `true`。

## 生成提交材料

安装报告和视频生成依赖：

```powershell
python -m pip install -r requirements/submission-docs.txt
```

生成 DOCX 与 PDF：

```powershell
python scripts/build_submission_report.py
python scripts/build_submission_report_pdf.py
```

组装最终演示视频：

```powershell
python scripts/assemble_submission_video_v2.py
```

捕获测试收据需要在已加载 ROS 2 的 WSL 环境执行：

```bash
source /opt/ros/jazzy/setup.bash

python3 scripts/capture_submission_test_receipt.py \
  --test-result-base "$HOME/.cache/strawberry_urp/colcon/build"
```

生成并验证确定性提交包：

```powershell
python scripts/package_submission_v2.py
python scripts/verify_submission_v2.py
```

默认输出位于：

```text
artifacts/submission_v2/
├── report/
├── video/
├── test/
└── package/
```

提交包包含源代码、配置、文档、接受模型、必要证据、报告、视频、测试收据和内嵌 `SUBMISSION_INVENTORY.json`。大型原始 MJPG、完整失败运行目录、训练数据压缩包和 colcon 构建树不会放入精简提交包。

## 提交版状态

submission-v2 已完成以下检查：

- DOCX 结构校验通过；
- PDF 共 7 页并完成视觉检查；
- 视频媒体属性与成功结果收据通过；
- 396 项测试收据与提交 commit 绑定；
- ZIP CRC 校验通过；
- ZIP 内全部成员的大小和 SHA-256 与内嵌清单一致；
- P3/P4 失败结果和科研边界保留；
- 未使用真实硬件或封存测试集。

正式提交前仍需人工完成：

- 填写学号、指导教师和学院要求的封面字段；
- 在提交电脑上完整播放 MP4；
- 打开 DOCX 和 PDF 检查字体；
- 确认学校要求的文件名和大小限制；
- 保留 ZIP 外部 SHA-256 收据。

## 学习路线

建议按以下顺序学习项目：

1. ROS 2 节点、话题、服务、动作与参数；
2. URDF、xacro、Gazebo 世界与 ros2_control；
3. 相机模型、RGB-D 深度与坐标反投影；
4. TF 坐标树和时间同步；
5. YOLO 数据、训练、阈值与 F1；
6. MoveIt 规划场景、碰撞对象与轨迹；
7. Panda 夹爪接触、附着和释放；
8. 状态机、超时、安全门和失败恢复；
9. 自动测试、结果收据、哈希和可复现打包；
10. 从仿真迁移到实机前需要补做的标定与安全验证。

对应的中文课程式讲解见 [`docs/learning-roadmap-zh.md`](docs/learning-roadmap-zh.md)。

## 证据与决策记录

项目的重要技术选择通过 `docs/decisions` 下的 ADR 固定。主要入口包括：

- ADR 0026：低于原指标的模型仅用于受限仿真工程控制；
- ADR 0030：记录 P3 正式矩阵失败；
- ADR 0032：拒绝 P4 重遮挡候选；
- ADR 0034：冻结 P5 发布；
- ADR 0057：接受自然植株单场景感知抓取；
- ADR 0060：接受 field-v3 固定场景重复抓取工程证据。

所有结果应优先引用机器可读 JSON、收据和哈希，演示截图或视频只能作为辅助展示，不能替代正式指标。

## 许可与使用说明

本仓库用于本科生 URP 教学、研究和仿真验证。外部数据、预训练模型、Blender 资源和纹理仍遵循其各自来源的许可与署名要求，具体来源记录见 `docs/assets` 及相关数据清单。
