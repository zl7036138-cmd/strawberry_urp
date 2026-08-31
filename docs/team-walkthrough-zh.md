# 团队成员必读：项目自底向上全解析（从一无所知到看懂全局）

> 面向对象：刚加入、对项目一无所知的团队成员。
> 本文按“最底层 → 最上层”的顺序讲解：先讲运行平台，再讲数据、仿真、接口、感知、定位、操作、编排、评测，最后讲工程治理和当前状态。
> 配套权威文档：`docs/architecture.md`（接口契约）、`NEW_PROJECT_HANDOFF.md`（交接）、`docs/learning-roadmap-zh.md`（学习路线）、`docs/decisions/`（决策记录）。

---

## 0. 一句话总览

这是一个**可复现的草莓成熟度识别、三维定位与机械臂抓取搬运仿真项目**（本科生 URP / 毕业设计，截止 2027-03-01）：

- 在 **Gazebo 仿真**中搭建草莓植株场景；
- 用 **RGB-D 相机 + YOLO11s** 判断哪颗草莓是成熟的（红）还是未成熟的（绿）；
- 用**深度图 + 相机内参 + TF 坐标变换**算出成熟草莓的三维位置；
- 用 **MoveIt 2** 规划并控制 **Franka Emika Panda** 机械臂：接近 → 夹取 → 搬运 → 放入收集箱 → 验证 → 恢复；
- 整个过程由一个**确定性的状态机**编排，配有一套**门控评测体系**（每项正式结论都必须有机器可读证据 + 哈希收据）。

**重要基调**：项目当前结论是“固定场景下可工作的工程仿真系统”，不是“通用田间采摘机器人”。所有未通过的正式指标和失败实验都被原样保留，不会被美化。

---

## 1. 项目背景与目标

### 1.1 要解决的问题

草莓采摘的完整闭环分为三步：**看**（哪颗熟了）、**算**（它在哪里）、**动**（抓起来放进篮子）。本项目的贡献表述为：

> 可复现的草莓成熟度感知—三维定位—机械臂抓取搬运闭环与分层评测。

### 1.2 明确排除（不做的事）

| 排除项 | 原因 |
|---|---|
| 实体机器人控制 | 纯仿真项目 |
| 仿真到现实迁移（sim-to-real）结论 | 不做任何迁移声明 |
| 果柄剪切 | 只做抓取搬运，不剪梗 |
| 柔性叶片/果实/损伤建模 | 物理复杂度边界 |
| 多相机图像级融合 | 双相机是“顺序协作”，不是融合 |
| 自研运动规划器 | 用 MoveIt 2 |

### 1.3 工程方法论（这是本项目的灵魂）

项目坚持“**先验收再继续**”：

1. 每个阶段有**预先冻结的指标门槛**（如定位误差、成功率）；
2. 每项实验有**一次性消费的声明**（claim），防止反复试到过为止；
3. 所有结果绑定**哈希收据**（JSON 里记录每个文件、模型、配置的 SHA-256）；
4. 失败的结果**原样冻结保留**，不做选择性重跑；
5. 所有重大技术决策写入 **ADR**（Architecture Decision Record，`docs/decisions/`，现有 79 份，编号至 0077）。

**为什么这么重**：因为这是毕设/科研项目。答辩时“你为什么选这个方案”“这个 39/135 是怎么来的”，必须能立刻给出不可篡改的证据链。

---

## 2. L0 运行平台层：软件环境

| 项目 | 内容 |
|---|---|
| 宿主系统 | Windows 11 |
| Linux 环境 | WSL2 中的 Ubuntu 24.04 |
| 机器人中间件 | ROS 2 Jazzy |
| 仿真器 | Gazebo Harmonic |
| 运动规划 | MoveIt 2 |
| 机械臂 | Franka Emika Panda（7 自由度 + 两指夹爪） |
| 视觉模型 | YOLO11s（Ultralytics 8.4.92） |
| GPU | RTX 4060 Laptop 8GB（CUDA 可用） |

全局约定：

- **单位**：距离一律米（m），角度一律弧度（rad）；
- **时间**：所有节点使用仿真时钟 `use_sim_time=true`；
- **坐标基准**：所有目标位姿最终都表达在 `panda_link0`（机械臂基座）坐标系下；
- **新鲜度**：超过 0.5 仿真秒的传感器/检测数据一律拒绝（`STALE_DATA`）。

WSL 发行版有两个：日常开发用 `Ubuntu-24.04-URP`；干净复现验证用 `Ubuntu-24.04-URP-Repro`（证明换台机器也能跑出同样结果）。

---

## 3. L1 数据与模型层：草莓从哪来、怎么学

### 3.1 数据来源与划分

| 项目 | 内容 |
|---|---|
| 来源 | Zenodo record 6126677 公开草莓数据集（`strawberries.zip`，约 1.5 GB，MD5 已固定校验） |
| 保留图像 | 732 张 |
| 划分 | 训练 501 / 验证 116 / **封存测试 115** |
| 划分种子 | `20260710`（确定性，可复现） |
| 类别 | 类别 0/1 → `RIPE`（成熟）/`UNRIPE`（未成熟）；原始数据的“果柄”类被排除 |
| 预训练权重 | `yolo11s.pt`、`yolo11n.pt`（哈希固定） |

**“封存测试集”是全项目最重要的纪律**：115 张测试图 + 标签从未被打开、从未被推理、从未被用于调参。它的“测试收据”（receipt）是全局一次性资源——一旦声明消费就永久消失。任何模型想上正式测试，必须先通过验证集上的 0.85 门槛。

### 3.2 标签审计（数据质量工程）

因为怀疑验证集有漏标，项目做了两轮人工双审：

- 冻结 21 个候选框，两位审核者独立判定；
- 规则：**分歧一律保守排除**；无法可靠判断成熟度的（如叶蒂完全遮住颜色）不强行二分类；
- 最终只有 13 个共识修正被应用到**版本化派生数据集**，原始标签从未被覆盖。

### 3.3 模型训练历程（为什么折腾这么多轮）

| 尝试 | 验证集 macro-F1 | 结果 |
|---|---:|---|
| 历史 YOLO11s 基线 @0.31 | 0.7871 | 未达 0.85 门槛 |
| 受控候选 Opt1 @0.44 | 0.7809 | 退化，拒绝 |
| 审计标签重评历史预测 | 0.8169 | 仍不达标 |
| 26 检查点统一扫描选 `baseline__best` @0.31 | 0.8158 | 仍不达标 |
| 未成熟样本×2 曝光训练 @0.54 | 0.7957 | 退化，拒绝 |
| 干净标签重训 `best.pt` @0.58 | **0.8007**（ripe 0.8871 / unripe 0.7143） | 拒绝晋升，但被用作工程豁免模型 |
| 仿真合成数据微调 | 真实集 macro 0.6255 | 严重退化，拒绝 |

**关键结论与纪律**：

- 原始数值门槛（macro-F1 ≥ 0.85）**至今未通过**；
- ADR 0026 给出“**工程豁免**”：允许把 0.8007 这个模型（SHA-256 `e3aca77e…1af70`，阈值 0.58）用于**受限的仿真工程集成**，但不改写失败指标、不打开封存测试集；
- 被拒绝的模型永远不得设为默认模型，也不得被称为“通过”。

---

## 4. L2 仿真场景层：三个世界

项目有**三个并存的仿真基线**，结论互不通用，这是理解项目的第一关键点。

### 4.1 v1：桌面球形草莓（已冻结的历史基线）

- 桌面 + 三个红/绿刚性球体果实（半径 **35 mm**）+ Panda + 固定 RGB-D 相机 + 收集箱；
- 作用：完成了 T40/T60、正式 P3、P4、P5、P6 全部历史阶段；
- 现在仅作兼容性回归场景保留。

### 4.2 v2：Blender 植株场景（当前默认基线）

- 来源：项目负责人提供的 Blender 文件（`strawberry_ripe_visual_v1.blend`、`strawberry_plant_v2.blend`）；
- 内容：花盆 + 草莓植株（叶片、叶脉、叶柄、花序、果梗）+ 三颗独立刚体果实（挂在花萼中心连接点）；
- 果实碰撞半径 **26 mm**；
- 三颗果实初始位置（`panda_link0` 系）：
  - `strawberry_1` = RIPE，`[0.419, -0.053, 0.546]`
  - `strawberry_2` = UNRIPE，`[0.550, 0.119, 0.532]`
  - `strawberry_3` = RIPE，`[0.579, 0.068, 0.540]`
- 资产管线：Blender → OBJ/MTL（`tools/blender/export_gazebo_assets.py`）→ SDF → Gazebo；轻量化后每果 18,150 三角面、植株 17,129 三角面；
- 踩过的坑：草莓网格三角面法线方向反了（内卷 winding），导致 RGB-D 相机采到果实背面，定位误差 44 mm → 机械修正后降到约 4.5 mm。

### 4.3 field-v3：大田场景（可选，固定场景演示用）

- 来源：`st1.blend`；
- 内容：5×12 m 地面 + 三条种植垄 + **101 株静态背景植株**（24 种网格变体实例化）+ 一个作业位（放入已验证的 v2 植株与三果实）+ Panda + 双相机；
- 背景约 17.3 万实例化三角面（磁盘上只存 4.2 万）；
- 普通启动**默认不动机器人**；完整执行必须走哈希校验、fail-closed 的专用 runner。

### 4.4 场景内的重要仿真机制

| 机制 | 作用 |
|---|---|
| RGB-D 相机 | 输出 `/camera/color/image_raw`（RGB8）、`/camera/depth/image_raw`（32FC1，米）、`/camera/camera_info` |
| 真值发布器 | 发布每颗果实的仿真真值位姿；历史 Oracle 路径可按冻结合同同步碰撞几何，当前广义路径只允许仿真适配和事后评分读取，禁止进入控制决策 |
| 附着管理器 | Gazebo `DetachableJoint`：夹稳后把果实“粘”在夹爪上，松爪后解除；启动时暂停物理、确认全部果实处于分离初始态后才恢复 |
| 接触传感器 | 左右夹指原始 Gazebo 接触信号（附着成功的主要证据） |
| 场景条件注入 | 可编程地注入光照（dim/nominal/bright）与遮挡（none/partial/heavy）——正式矩阵的基础设施 |

### 4.5 相机方案（三种，按需选）

| 模式 | 说明 |
|---|---|
| `camera_mount:=fixed` | 历史默认：固定“眼在手外”相机 |
| `camera_mount:=wrist` | 腕部相机：机械臂夹爪附近，**唯一精确定位输入** |
| `camera_mount:=dual` | 底座概览相机（320×240@10Hz）+ 腕部精测相机（640×480@30Hz），**顺序协作**：底座发现目标 → 机械臂移动到观察姿态 → 腕部稳定后精测 |

注意：dual 不是多相机融合。底座流只做全局观察/展示，腕部流才是精确定位输入；两个相机外壳都参与 MoveIt 碰撞检测。

---

## 5. L3 接口契约层：各模块之间怎么说话

接口定义包：`strawberry_interfaces`。**架构契约（`docs/architecture.md`）是本层权威**，改动需 ADR 记录。

### 5.1 话题、服务、动作

| 接口 | 类型 | 内容 |
|---|---|---|
| `/strawberry/detections` | 话题 | `StrawberryDetectionArray`：检测框列表（target_id、成熟度、置信度、像素 ROI） |
| `/strawberry/target_pose` | 话题 | `TargetPose`：目标 ID + 位姿 + 置信度 + 位置估计 σ |
| `/strawberry/pick_and_place` | **动作（Action）** | 给定目标位姿与放置位姿，返回结果、失败码、规划/执行耗时 |
| `/strawberry/tracked_targets` | 话题 | 广义模式：全部稳定三维目标（带 track_id） |
| `/strawberry/run_harvest` | 服务 | 广义模式：启动一次连续采摘批次 |
| `/strawberry/harvest_status` | 话题 | 广义模式：批次状态、成功/跳过/失败/剩余目标 |
| `/strawberry/evaluate_target` | 服务 | 广义模式：不执行轨迹的 IK/碰撞预检查 |
| `/strawberry/oracle/*` `/strawberry/shadow/*` | 命名空间 | Oracle（真值控制）与 Shadow（观察诊断）物理隔离，见 §8 |

### 5.2 稳定失败码（1–10，全局统一）

```
1 NO_TARGET        没有目标
2 LOW_CONFIDENCE   置信度不足
3 DEPTH_INVALID    深度无效
4 TF_TIMEOUT       坐标变换超时
5 UNREACHABLE      IK 不可达
6 PLANNING_FAILED  规划失败
7 COLLISION        碰撞
8 GRASP_FAILED     抓取失败
9 PLACE_FAILED     放置失败
10 STALE_DATA      数据过期
```

每个失败都能归因到**具体阶段**——这是项目能做“失败归因统计”的基础。

### 5.3 一个著名的坑：RGB/BGR 边界

ROS 相机话题保持 `rgb8`，但 Ultralytics 对 NumPy HWC 输入按 **BGR** 解释并自行做 BGR→RGB 反转。感知适配器必须在调用 YOLO 前用 CvBridge 请求 `bgr8`。早期因此出现“仿真里成熟果完全检测不到”的幽灵 bug（ADR 0006 修复）。

---

## 6. L4 感知层：机器人的眼睛

包：`strawberry_perception`。

| 模块 | 职责 |
|---|---|
| `perception_node.py` | 订阅相机图像 → YOLO11s 推理 → 发布 `/strawberry/detections`（类别 + 置信度 + 框） |
| `shadow_diagnostic.py` | Shadow 模式：只观察不动手，用于诊断窗口与帧计数 |
| `sim_perception_gate.py` | 仿真感知预门控评测 |

感知本身只做两件事：**这个框里是不是草莓**、**是成熟还是未成熟**。它不做三维定位（那是下一层的事）。

### 广义检测器（2026-08 新成果）

针对多植株场景重新采集训练数据（120 个开发场景：72 训练 / 24 验证 / 24 资格，与正式矩阵**零重叠**），微调出的广义检测器 v2：

- 冻结运行参数：图像尺寸 800、置信阈值 0.2052…、NMS IoU 0.50；
- 验证集：成熟果精确率 96.15%、召回率 90.91%；
- 独立资格集（只跑一次）：精确率 95.65%、召回率 97.78%；
- 资格门通过。

---

## 7. L5 定位层：把像素变成三维坐标

包：`strawberry_localization`。这是全项目**踩坑最多、工程最细**的一层。

### 7.1 基础流程

1. 收到检测框 → 取框中心 30% 区域内的**有效深度中位数**；
2. 相机内参反投影 → 得到相机光轴系下的三维点；
3. TF 变换：`strawberry_camera_optical_frame` → … → `panda_link0`；
4. 输出 `TargetPose`（带估计 σ）。

### 7.2 关键技术点（每个都有血泪史）

| 技术点 | 说明 |
|---|---|
| **表面→果心偏移** | 深度测到的是果实表面，不是果心。沿相机光线方向补一个偏移：v1 用 35 mm，v2/field-v3 用 26 mm，**由场景配置显式传入**，禁止依赖节点名隐式匹配（曾因此产生 29 mm 误差） |
| **时间戳缓存** | 检测有推理延迟。定位器缓存 60 组深度/CameraInfo（2 秒窗口），给迟到的检测配对**最近的同帧**数据（50 ms 同步容差），而不是拿最新无关深度乱配 |
| **网格法线修复** | Blender 导出后三角面 winding 内卷 → 相机采到果实背面 → 44 mm 误差。机械翻转面索引后降到中位 4.5 mm / P95 5.0 mm |
| **深度分层（遮挡感知）** | 框中心深度可能落在前景叶片/遮挡物上。新增几何深度分层定位器：按深度层支持度选果实层，拒绝弱层（曾发现 17 像素弱层压过 280 像素果实层）。重遮挡下 P95 误差从旧算法 361.7 mm 降到 26.9 mm |
| **多目标跟踪（广义模式）** | 用 60 mm 几何门限维护跨帧稳定的 `track_id`，运行时**不读** Gazebo 真值身份 |
| **方差加权融合** | 底座相机与腕部相机的定位估计按逆方差融合，并给腕部一个**非零系统误差下限**——防止单次近距离观测用虚假高置信度覆盖稳定的底座定位 |

### 7.3 定位质量的历史成绩

| 门控 | 结果 |
|---|---|
| T40（v1，100 位置） | 100/100 通过；中位 1.345 mm，P95 1.897 mm |
| Blender-v2（修复后，100 位置） | 100/100 通过；中位 4.504 mm，P95 5.046 mm |
| field-v3（无运动 Shadow，60 帧） | 60/60 目标位姿；中位误差 4.856 mm |

---

## 8. L6 操作层：机器人的手

包：`strawberry_manipulation`。核心是 MoveIt 2 规划 + 夹爪闭环。

### 8.1 抓取几何（场景绑定的“抓取剖面”）

| 参数 | v1（35 mm 球） | v2（26 mm 果） |
|---|---:|---:|
| 手部后置距离 | 0.1054 m | 0.0964 m |
| 夹指闭合指令 | 0.025 m | 0.022 m |
| 预抓取抬高 | 0.15 m | 0.15 m |
| 夹指张开指令 | 0.04 m | 0.04 m |

剖面由精确的 `(world_name, fruit_collision_radius_m)` 场景对解析，缺失/重复/不匹配一律 fail-closed。

### 8.2 安全接近与抓放动作

1. **防护接近**：先垂直抬到终点上方 0.02 m → 原地转向 → 移到 `y=-0.10 m` 安全走廊 → 沿走廊平移 → 对准目标 → 降到预抓取点。笛卡尔分段 ≤0.01 m / 10°，插值碰撞检查 ≤0.01 rad（不漏检）；
2. **碰撞球同步**：MoveIt 里所有果实都是碰撞球（半径来自场景合同）。历史 Oracle/固定场景执行前从真值快照同步球体；当前广义主线必须只从 `/strawberry/tracked_targets` 同步（防“幽灵障碍物”，同时避免真值进入控制）；
3. **只移除目标球**：只有最后直线下降段前才移除**选中目标**的碰撞球（打开接触走廊），其他果实始终是障碍；
4. **双侧接触 + 附着**：夹爪闭合后确认左右两指都有真实接触 → Gazebo attach → 抬升 0.08 m → 越过箱壁 → 对准收集箱 → 下降 → 张开 → detach → 果实中心在箱内停留 1 仿真秒 = 成功；
5. **一次性纠偏**：控制器报告完成但实测终点超 10 mm 容差时，允许**从实测位姿重规划一次**；再失败立即判败，绝不允许无限重试；
6. **夹爪双指控制器**：DART 不执行 Panda 右指 mimic 约束 → 改为左右两个显式单关节控制器，同时下发、分别验证实测位置；闭合位移 <2 mm 安全拒绝。

### 8.3 广义模式的新增能力

- **MoveIt 预选**（ADR 0077，最新）：目标发布前先做连通预抓取/接近/抓取/撤离可行性检查，不可行目标根本不发布（避免批次里反复失败）；
- 12 个有限腕部观察视点按确定性优先级排序，逐个试，找到**零运动风险**的方案才执行；轨迹一旦开动，任何失败立即安全终止（不许换视角试探）。

---

## 9. L7 编排层：系统的大脑

包：`strawberry_bringup`。

### 9.1 确定性状态机

```
INIT → ACQUIRE → DETECT → LOCALIZE → SELECT → PLAN
     → APPROACH → GRASP → RETREAT → PLACE → VERIFY → DONE | FAILED
```

安全规则（全部硬编码）：

- 只选**成熟、深度有效、TF 有效、IK 可达**的目标；没有成熟果 → `NO_PICK`，**机械臂不得运动**（这是负样本安全的根基）；
- 深度/TF 失败最多重取 3 帧；
- 规划失败只允许 1 次备用接近方向；
- 抓取失败 → 张开夹爪安全结束，禁止无限重试；
- 附着无法解除 → **硬停止**，不做恢复运动（防止夹着的果实碰坏场景、掩盖真实故障）。

### 9.2 Oracle / Shadow 隔离（评测的根基）

| 模式 | 控制源 | 用途 |
|---|---|---|
| Oracle | `/strawberry/oracle/target_pose`（仿真真值） | 先证明“感知之外的环节全对” |
| Shadow | `/strawberry/shadow/*`（感知输出，只观察） | 让感知在不动手的情况下被测量 |

这是历史 Oracle/Shadow 验证合同：两者**不得共用同一控制话题**；当时真值只可用于目标 ID 关联和冻结路径的碰撞几何，**绝不能替代感知算出的三维位置**。当前广义主线限制更严格：定位默认不创建真值订阅，碰撞球来自稳定跟踪目标，并由无运动 ROS 图审计验证隔离。

### 9.3 广义连续采摘编排器

- `/strawberry/run_harvest` 启动批次：观察 → 腕部确认 → 抓取 → 放置 → 重新扫描，循环；
- 失败目标只允许**一次重新观察**；重试时不可见则标记**跳过**，继续扫其他目标（单目标失败不终止批次）；
- 完成/耗尽的目标 ID 全系统广播一次，跟踪器、选择器、碰撞场景、批次队列全部抑制它；
- 真值在广义模式下**只剩一个合法用途**：仿真适配层把“感知选中的物理接触”映射到可 detach 的 Gazebo 实体——不能选目标、排序、修正位姿或做路径选择。

---

## 10. L8 评测层：怎么证明“真的做到了”

包：`strawberry_benchmark`。项目没有“跑一跑看着不错”这种证据——所有结论来自门控评测 + 哈希收据。

### 10.1 门控体系（正式阶段）

| 门 | 内容 | 结果 |
|---|---|---|
| **P0** 工程基线 | WSL/ROS/Gazebo/MoveIt 冒烟 + 30 分钟稳定性 | ✅ |
| **P1** 仿真与数据 | 场景、数据、确定性划分、内容哈希 | ✅ |
| **P2** 模块 | T40 定位门 + Oracle 抓取门通过；T30 视觉门**数值失败**（宏 F1 0.8007 < 0.85，凭 ADR 0026 豁免继续工程） | ⚠️ 豁免 |
| **P3** 端到端正式矩阵 | 3 光照 × 3 遮挡 × 5 位置 × 3 种子 = **135 正场景 + 30 负场景**，全量跑完、一次成型、禁止重跑 | ❌ 39/135（28.89% < 80%） |
| **P4** 鲁棒性干预 | 唯一一次干预（仿真适配模型）：重遮挡检测 300/300 恢复，但目标位姿 0/300（框中心深度落在前景遮挡物上）→ 拒绝晋升 | ❌ |
| **P5** 发布 | 干净环境复现（246/246 测试）、10/10 行为冒烟、报告、图表、演示视频（288.7 s） | ✅ |
| **P6** 交付 | 确定性压缩包、内嵌清单、CRC + SHA-256 校验 | ✅ |

### 10.2 P3 失败的归因（教科书级失败分析）

- 正场景成功 39/135（28.89%），门槛 80%；
- 负场景安全 **30/30 `NO_PICK`**——零误摘未成熟果，这是最值得保留的成绩；
- 96 个失败 = **84 个感知失败 + 12 个抓取失败**：说明瓶颈在“看”，不在“抓”。

### 10.3 门控之外的证据纪律

- 每个试验**全新世界 + 独立 ROS domain**，一个场景一次机会；
- 收据（receipt）记录所有文件的 SHA-256、配置指纹、结果摘要；
- 验收器会检查“缺失、重复、多余场景”并直接判失败；
- 干净的独立 WSL 发行版复现（`Ubuntu-24.04-URP-Repro`）验证可复现性。

### 10.4 广义采摘的正式矩阵（当前工作前沿，仍封存）

`config/generalized_harvest_matrix_v1.json`：**30 个正式隐藏种子** = 18 正样本（近/中/远 × 无/部分/重度遮挡均衡）+ 6 全未成熟负样本 + 6 不可达/不安全样本。

评测器同时检查：检测召回/精确率、ID 切换、定位 P95、σ、误采未成熟果、碰撞、负样本安全停止、单目标成功率、完整场景完成率。**只有全部 30 个一次性运行完成且 `overall_pass: true`，才能声称泛化验收达标。**

---

## 11. L9 工程治理层：让项目经得起审计

### 11.1 仓库布局

```
strawberry_urp/
├── ros2_ws/src/        # 7 个 ROS 2 包（核心代码）
├── config/             # 冻结的正式矩阵、进度、豁免、契约 JSON
├── scripts/            # 环境安装、构建、门控运行、验证、打包脚本（100+）
├── tools/              # 数据/感知/Blender 处理工具
├── data/               # 原始数据、清单（不进入 Git）
├── weights/ outputs/   # 预训练权重、训练产物（哈希管理，不进入 Git）
├── results/ artifacts/ # 运行结果、汇总证据（不进入 Git）
├── docs/               # 架构、ADR（decisions/）、报告、学习路线
└── assets/blender_sources/  # Blender 源文件（进入 Git）
```

### 11.2 ADR 决策记录

`docs/decisions/` 现有 79 份 ADR（编号至 0077），格式 `NNNN-简短描述.md`，每份记录**背景 → 决策 → 后果**。写论文、答辩、换人接手时直接引用。代表性：

- ADR 0006：修复 RGB/BGR 通道边界；
- ADR 0026：低于门槛模型仅限工程豁免使用；
- ADR 0030：P3 正式矩阵失败冻结；
- ADR 0032：拒绝 P4 重遮挡候选；
- ADR 0060：接受 field-v3 固定场景三次连续抓取（工程证据）；
- ADR 0077：MoveIt 预选先于目标发布。

### 11.3 Git 与复现

- 仓库：`https://github.com/zl7036138-cmd/strawberry_urp`（当前分支 `codex/wrist-readiness-diagnostics`）；
- 大型产物（数据、权重、构建、日志、结果）不入 Git，由**清单 + 哈希 + 脚本**复现；
- 每个正式阶段产出“handoff”JSON：把证据、哈希、结论一次性冻结。

---

## 12. 一次完整抓取的数据流（把上面九层串起来）

以 field-v3 演示为例，跟着数据走一遍：

```
① [L2 仿真] Gazebo 渲染场景，发布底座/腕部 RGB + 深度 + 内参
② [L4 感知] YOLO11s 推理腕部图像 → /strawberry/detections（哪颗是 RIPE）
③ [L7 编排] 选择器按成熟度/深度有效/TF 有效/IK 可达筛选，选中目标
④ [L5 定位] 框中心深度中位数 → 反投影 → TF → panda_link0 系 TargetPose
⑤ [L6 操作] MoveIt 同步碰撞球 → 规划防护接近路径 → 预检查
⑥ [L6 操作] 直线下降 → 双指闭合 → 双侧接触确认 → Gazebo attach
⑦ [L6 操作] 撤离 → 越箱壁 → 对准收集箱 → 张开 → detach → 箱内停 1 s
⑧ [L7 编排] VERIFY 验证 → 机械臂回 ready 位姿 → DONE
⑨ [L8 评测] 全流程反馈序列、接触证据、耗时、收据哈希写入结果 JSON
```

**Field-v3 的成绩**：三个连续全新世界全部完成 `PLAN → APPROACH → GRASP → RETREAT → PLACE → VERIFY → DONE`，双指原始+处理后接触确认，附着序列 `attached→detached`，机械臂回到 ready 位姿误差 6.24e-13 rad，单次动作耗时约 208–268 s。

---

## 13. 环境搭建与运行方法（新成员动手指南）

### 13.1 三个固定资源（先放到本地）

| 文件 | 目标位置 | 校验 |
|---|---|---|
| Zenodo `strawberries.zip` | `data/raw/zenodo_6126677/` | MD5 `db8d5dcb4b8adebf1621788373fd3031` |
| `yolo11s.pt` | `weights/` | SHA-256 `85a76fe8…502d5` |
| `yolo11n.pt` | `weights/` | SHA-256 `0ebbc80d…644ee1` |

### 13.2 干净环境三步走（全新 Ubuntu 24.04 WSL2）

```bash
# 1. root 执行一次：安装 ROS 2 Jazzy、Gazebo、MoveIt、Python 环境
bash scripts/bootstrap_ubuntu_2404.sh

# 2. 普通用户：构建 + 全量测试
bash scripts/build_and_test.sh

# 3. 启动系统（有界面）
bash scripts/run_system.sh headless:=false
```

检查环境（CUDA/OpenCV/cv_bridge/YOLO）：

```bash
bash scripts/verify_environment.sh
```

网络不稳时先在 Windows PowerShell 准备固定 Linux wheelhouse：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/cache_perception_wheels.ps1
```

### 13.3 运行 field-v3 演示

```bash
export STRAWBERRY_COLCON_ROOT="$HOME/.cache/strawberry_urp/colcon"
bash scripts/run_field_v3_perception_pick_headed.sh \
  results/development/field_v3_new \
  outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt \
  230 60 true false
```

参数依次：输出目录（**必须不存在**）、模型路径、ROS domain ID、结束后窗口保留秒数、是否显示窗口、是否录制视频。

### 13.4 广义多植株采摘（当前前沿）

```bash
# 1. 生成随机场景（1~3 株 × 每株 2~3 果）
ros2 run strawberry_sim generate_generalized_scene \
  --base-scene ros2_ws/src/strawberry_sim/config/scene.yaml \
  --base-world ros2_ws/src/strawberry_sim/worlds/strawberry_orchard.sdf \
  --output-dir results/generalized/dev_seed_17036 \
  --seed 17036 --profile mixed --plant-count 3 \
  --position-band middle --occlusion partial

# 2. 启动广义采摘系统
ros2 launch strawberry_bringup generalized_harvest.launch.py \
  world_file:=$PWD/results/generalized/dev_seed_17036/generalized_seed_017036.sdf \
  scene_config_file:=$PWD/results/generalized/dev_seed_17036/generalized_seed_017036.yaml \
  model_path:=$PWD/outputs/perception/yolo11s_640_generalized_dev_v2/weights/best.pt \
  headless:=false

# 3. 启动一次连续采摘
ros2 service call /strawberry/run_harvest std_srvs/srv/Trigger "{}"
ros2 topic echo /strawberry/harvest_status
```

详见 `docs/generalized-harvest-v1.md`。

---

## 14. 代码结构速查（七个包）

| 包 | 职责 | 关键模块 |
|---|---|---|
| `strawberry_interfaces` | 消息、动作、枚举、失败码 | 全局接口契约 |
| `strawberry_sim` | Gazebo 世界、Panda、相机、真值、附着、接触 | `core.py`、`attachment_manager.py`、`ground_truth_publisher.py`、`scene_conditions.py`、`generalized_scene.py` |
| `strawberry_perception` | YOLO 推理与成熟度检测 | `perception_node.py`、`shadow_diagnostic.py`、`sim_perception_gate.py` |
| `strawberry_localization` | 深度反投影、TF、跟踪、深度分层 | `node.py`、`core.py`、`tracking.py`、`generalized_node.py`、`generalized_depth.py` |
| `strawberry_manipulation` | MoveIt 场景/规划/抓放 Action/接触 | `action_server.py`、`moveit_backend.py`、`moveit_scene.py`、`grasp_geometry.py`、`gripper_*` |
| `strawberry_bringup` | 启动组合与确定性编排 | `orchestrator.py`、`target_selector.py`、`harvest_orchestrator.py`、`harvest_sequence.py` |
| `strawberry_benchmark` | 场景矩阵、试验、指标、收据、验收 | `formal_matrix.py`、`acceptance.py`、`metrics.py`、`generalized_acceptance.py` |

`scripts/` 下每个 `run_*` 对应一个门控/演示，`validate_*`/`summarize_*` 对应结果校验与汇总。**`results/` 与 `artifacts/` 是只读证据区**，不要修改其中的内容。

---

## 15. 当前状态与下一步（2026-08-31，开发门审计）

### 已证明

- ✅ 广义检测器通过独立资格门（P 95.65% / R 97.78%）；
- ✅ 无真值依赖的单果抓放闭环（seed_44008：1 果 HARVESTED，双侧物理接触，MoveIt 预选拒绝不可行目标，批次安全收尾）；
- ✅ 广义定位默认不创建真值订阅；真实 ROS 图无运动隔离审计通过；
- ✅ 全量回归：735 项纯 Python 测试（0 错误、0 失败、2 跳过）和 617 项 ROS/colcon 测试（0 错误、0 失败）。

### 当前阻塞点

> 尚无任何开发场景证明**同一批次连续采下 ≥2 颗**可行成熟果。

发现种子 `45001～45018` 的无运动筛选全部清理干净且通过真值隔离，但 18 场中只有
`45007` 同时形成至少两条稳定、成熟且 MoveIt 可行的轨迹。29 条候选中只有 7 条
可行；保持滚转分支一致的抓取正确性修复没有提高这一比例。当前证据指向保守可行域
与随机场景分布的组合瓶颈，不能靠降低安全阈值解决。

### 下一步（按序）

1. 在干净冻结提交上对从未调试的 `46001～46018` 先做无运动资格筛选；
2. 若不足五个合格场景，停止并报告，不执行行为、不放松阈值；
3. 若正好选满五场，各执行一次连续采摘，并要求至少 4/5 同批采摘 ≥2 颗；
4. 开发门全部通过后才允许创建一次性 claim 并打开封存的 **30 种子正式矩阵**。

---

## 16. 科研边界：哪些话能说，哪些话不能说

**可以说**：

- 固定场景下，感知 → 定位 → 规划 → 抓取 → 放置 → 恢复的工程链路已跑通（v2 单次、field-v3 连续三次）；
- 定位精度在验证过的场景下达到毫米级（4.5–5 mm 量级）；
- 负样本安全 30/30（v1）——不会误摘未成熟果；
- 广义检测器已通过独立资格门。

**不能说**（会被同行/答辩老师当场打回）：

- ❌ “能摘任意植株、任意姿态的草莓”——只验证过固定场景；
- ❌ “真实机械臂已经运行”——纯仿真；
- ❌ “无损采摘/已剪梗”——未建模、未实现；
- ❌ “视觉模型达标”——T30 数值门至今失败，宏 F1 0.8007 < 0.85，靠工程豁免才继续；
- ❌ “P3/P4 通过”——P3 正式矩阵 39/135 失败，P4 干预被拒；
- ❌ “通过正式测试集”——封存测试集从未打开；
- ❌ 用 field-v3 的固定场景成功去“覆盖”v1 的 P3 失败——两个基线结论不通用。

---

## 17. 给新成员的阅读建议（最快上手路径）

1. 先看本文 §0、§2、§4，理解“三个场景基线”和“门控评测”两个核心概念；
2. 读 `docs/architecture.md` 的 ROS 接口与状态机部分（§9 的官方版）；
3. 读 `docs/learning-roadmap-zh.md`，按 1→10 的顺序补技术背景；
4. 读 `docs/generalized-harvest-v1.md`（当前工作前沿）和最新的 `config/generalized_runtime_progress_v*.json`；
5. 动手：按 §13 搭环境、跑通 `run_system.sh`，再跑一次 field-v3 演示；
6. 遇到“为什么当初这么设计”，去 `docs/decisions/` 找对应 ADR。

**唯一铁律**：项目里所有正式结论都绑定机器可读 JSON、收据和哈希。截图、视频只是辅助展示，不能替代正式指标。
