# 基于 ROS 2、YOLO 与 Gazebo 的草莓成熟度识别及机械臂抓取搬运仿真

结项收口版：2026-10-03

技术基线：`codex/wp01-evidence-validation@e21b704`
完整结构索引：[`FINAL_ARCHITECTURE.md`](FINAL_ARCHITECTURE.md)

## 摘要

本项目构建了一套运行在 ROS 2 Jazzy、Gazebo Harmonic 和 MoveIt 2 上的
草莓采摘仿真系统。系统使用底座 RGB-D 相机发现和跟踪候选目标，以腕部 RGB-D
相机进行近距离确认，通过 YOLO 区分成熟与未成熟草莓，并结合深度、相机内参和
TF 计算草莓在 Panda 基座坐标系中的三维位置。机械臂侧实现了碰撞感知目标选择、
确定性有限抓取候选、抓取前完整路径资格验证、证书与执行身份绑定、显式载荷生命
周期、入箱释放验证以及有界恢复。

项目保留了所有未通过的正式结果：真实图像成熟度模型宏平均 F1 为 0.800675，
未达到 0.85 原门槛；正式 P3 成熟果正场景为 39/135；P4 重遮挡条件有效三维
位姿为 0/300。后续工程开发证明了固定 field-v3 单果完整往返、一个开发种子
同批两果、抓前整链正/负例，以及受控障碍中从失败的名义抓法自动选择 G03 并
完成抓放。后续结果是受限开发证据，不替代历史正式失败，也不构成真实机械臂、
无损采摘或普遍田间泛化证明。

## 1. 项目目标与边界

目标是在可复现仿真环境中闭合：

```text
成熟度识别 → RGB-D 三维定位 → 多目标跟踪 → 安全目标选择
→ 近距离确认 → 抓取方案资格验证 → 机械臂抓放 → 释放验证 → 重新扫描
```

当前范围不包括移动底盘、田间导航、真实 Panda、剪梗、果实损伤建模、柔性植被
动力学和 sim-to-real 安全认证。

## 2. 系统组成

| 层 | 实现 | 输出/作用 |
| --- | --- | --- |
| 仿真 | Gazebo Harmonic、DART、Panda、植株/果实/料箱、双 RGB-D | 图像、深度、关节、接触、附着与评分真值 |
| 感知 | YOLO11s | 成熟度、置信度、二维框 |
| 定位 | 深度中位数、CameraInfo、TF2 | `panda_link0` 中三维目标与不确定度 |
| 追踪 | 多帧位置匹配 | 稳定 `track_id`、新鲜度、观测数、成熟度 |
| 决策 | 安全硬门、MoveIt 预检、动态观察位姿 | 当前目标与腕部观察计划 |
| 操作 | G00–G14、whole-chain gate、payload lifecycle | 授权抓取方案和稳定失败阶段 |
| 规划控制 | MoveIt 2、ros2_control、双夹指 Action | 碰撞检查轨迹与实际执行 |
| 任务 | 连续采摘编排、tombstone、失败恢复 | 已采、跳过、失败、剩余目标 |
| 评测 | JSON 收据、truth isolation、cleanup、hash manifest | 可审计结论及边界 |

广义运行时的 localization、selection、orchestration 和 manipulation 不允许订阅
Gazebo 果实真值。真值只用于场景生成、仿真适配和离线评分。

## 3. 最终操作架构

### 3.1 ADR 0085：载荷生命周期

```text
EMPTY → CONTACT → HOLDING → ESCAPED → AT_BIN → RELEASED
```

载荷所有权、碰撞语义和恢复授权被分离。进入 `HOLDING` 后，普通规划/执行失败
不能自动解除挂接、开爪或回零；系统保留 carried collision object 并返回
`MOTION_WITHHELD`。只有到达 `AT_BIN` 后才允许释放。

### 3.2 ADR 0086：抓前整链资格门

在第一次夹爪命令之前，系统于 copied PlanningScene 中依次检查：

```text
PREGRASP → APPROACH → GRASP_STATE → VIRTUAL_ATTACH
→ ESCAPE → TRANSPORT → BIN_APPROACH
```

评估期间真实 controller、gripper 和 attach 命令均为 0，live scene 不被改变。
故意封堵运输的负例在 `TRANSPORT_FAILED` 时保持 payload=`EMPTY`、恢复目标碰撞
对象且不进入真实抓取。

### 3.3 ADR 0087：有限候选与证书执行

系统生成 15 个固定顺序的局部 X/Y 倾角候选 `G00–G14`，使用 5° 第一环和
10° 外环。每个候选包含 pregrasp、grasp、escape 及几何 fingerprint。搜索
策略为 first-feasible，不评分、不随机排序。`AuthorizedGraspPlan` 绑定 target、
candidate、geometry、scene 和 certificate identity；任一不匹配即在实际运动前
拒绝。

最终 profile 明确区分：

- `standard_conservative`：默认，adaptive execution 关闭，但保留 ADR 0086
  nominal 整链门和 ADR 0085 payload lifecycle；
- `bounded_adaptive_demo`：显式开启，只用于复现 ADR 0087-E 受控挑战。

## 4. 实验结果

### 4.1 历史正式基线（不得改写）

| 指标 | 结果 | 判定 |
| --- | ---: | --- |
| 真实图像 YOLO audited validation macro-F1 | `0.800675`，门槛 `0.85` | FAIL；仅工程豁免 |
| P3 成熟果正场景 | `39/135 = 28.89%`，门槛 `80%` | FAIL |
| P3 仅未成熟负场景 | `30/30` 安全 `NO_PICK` | 该子项 PASS |
| P4 重遮挡 | 检测 `300/300`，有效位姿 `0/300` | FAIL |
| T40 历史定位 | `100/100`，median 1.345 mm，P95 1.897 mm | 历史 v1 PASS |
| T60 Oracle | `10/10`，规划 P95 0.058841 s | PASS |

### 4.2 后正式工程扩展

| 扩展 | 结果 | 结论边界 |
| --- | --- | --- |
| field-v3 | 历史三次固定场景成功；2026-09-23 当前代码单果往返成功 | 固定场景开发基线 |
| 广义同批两果 | seed 45504 完成两个不同成熟目标 | 单个反复调试开发种子；严格安全证据仍不完整 |
| ADR 0086 正例 | 两目标七阶段 FEASIBLE，资格区间零命令 | whole-chain 正向运行时链成立 |
| ADR 0086 负例 | 前五阶段通过，TRANSPORT_FAILED，payload EMPTY | 能在抓前拒绝“抓得到但运不出” |
| ADR 0087-D | 感知输入 G00 plan-only 证书，零执行 | 候选和 copied scene 运行时连通 |
| ADR 0087-E | G00–G02 APPROACH_FAILED，G03 FEASIBLE 并 ACTION_SUCCEEDED | 受控机制证明，不是一般成功率提升 |

ADR 0087-E 的 `G03` 完成：

```text
PLAN → AUTHORIZED_G03 → APPROACH → GRASP → CONTACT → HOLDING
→ RETREAT → ESCAPED → PLACE → AT_BIN → RELEASED
→ VERIFY → RETURN_ROUTE → DONE
```

完整纯 Python 回归在规范 WSL 环境中为 1040 项、0 失败、0 错误、3 个环境
跳过；完整 ROS 2/colcon 构建测试为 897 项、0 失败、0 错误、0 跳过。
两份机器可读回执均归档于 `artifacts/final_evidence/`。Windows Python 的
非规范运行保留了 5 个平台契约失败，用于说明测试环境边界，不计作最终通过回执。

## 5. 两条视觉证据线

### 5.1 真实图像成熟度模型

主数据来自 Zenodo record 6126677。audited validation macro-F1 为
0.800675，未达到 0.85 门槛；只允许在受限仿真工程中使用，held-out real test
继续封存。

### 5.2 广义仿真检测器

广义 Gazebo 开发检测器在独立 simulator qualification split 上记录 ripe precision
95.65%、recall 97.78%。该结果只描述生成仿真域，不能写成真实草莓数据精度，
也不能支持 sim-to-real 结论。

## 6. 仿真与现实差异

当前植被和果实主要是刚体或保守碰撞代理。真实叶片可弯曲、果梗有柔顺性，果实
受夹持力后可能被动居中或损伤；这些动力学没有在当前模型中验证。系统的主要贡献
是感知—定位—规划—操作决策链以及可审计安全门，不是果实材料或植物柔性仿真。

## 7. 可复现与证据交付

- 最终架构：`docs/FINAL_ARCHITECTURE.md`
- 最终交接：`NEW_PROJECT_HANDOFF.md`
- profile：`config/final_run_profiles.yaml`
- 永久证据：`artifacts/final_evidence/`
- manifest：`artifacts/final_evidence/manifest.json`
- 纯测试 CI：`.github/workflows/pure-tests.yml`
- 许可与资产：`LICENSE`、`THIRD_PARTY_NOTICES.md`、`ASSET_PROVENANCE.md`
- 最终归档工具：`scripts/package_final_delivery.py`（仅允许干净 Git 树）

历史 submission-v2、P5/P6 和失败矩阵继续保留，但不再代表当前代码交付版本。

## 8. 最终结论

项目已经完成一套可复现、分层和可审计的仿真草莓采摘系统，并证明了固定场景
单果、一个开发种子同批两果、抓前整链正/负授权路径以及一个受控非名义候选抓取。
同时，项目没有通过原始真实图像 YOLO、P3、P4、五场多果或正式 30 种子泛化门。

因此最终准确表述为：

> 本项目在仿真中实现并验证了双相机成熟草莓定位、碰撞感知抓放、抓前整链资格
> 验证、载荷生命周期和有限候选自适应抓取机制；其泛化、柔性植物动力学和实机
> 安全仍属于未来工作。
