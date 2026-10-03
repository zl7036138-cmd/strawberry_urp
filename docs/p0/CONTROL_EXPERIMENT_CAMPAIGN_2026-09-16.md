# v20–v26 控制实验收官报告（seed 45504）

**分支**：`codex/wp01-evidence-validation`　**提交链**：`…5333256 → d543019 → 1fb69cd → 19103a0`（全部已推送确认）
**结论**：A3 已定——两种残余失败均为仿真物理/管线层结构缺陷，记录为已知限制；真机用 setLoad 方案。

---

## 实验矩阵（全部同场景 seed 45504）

| 探针 | 提交 | 单变量 | 结果 |
|---|---|---|---|
| v20 | `059b158` | 混合控制器（j1-6 effort + j7 position） | 观察通过，j4 被踢中止 |
| v21 | — | 窗口首验 | 窗口数据通路 bug（9 关节 vs 7），全拦 |
| v22 | `6abd440` | 窗口按名映射修复 | **最深**：抓取→搬运→对准→释放全通，回零 3 次 −4 |
| v23 | `5333256` | 静止头 + 踢重试 | 头无效（仍 +0.54），重试 4/4 再败 |
| v24 | `d543019` | 200 Hz | 漂移 0.058→0.035（−40%），踢缩短 0.4→0.17s |
| v25/v25b | `1fb69cd` | gain 2.0 | 控制稳定但观察规划系统性贴边拒绝（14 次） |
| v26 | `19103a0` | gain 1.5 | 漂移对增益不敏感（0.057–0.063），定格结论 |

## 两种残余失败机制（最终定性）

1. **焊接冲量踢**（需果实）：DART 焊接约束投影冲量，目标开始时 +0.5 rad/s 反向 0.4s。静止头/重试/窗口均无效——物理层，命令整形够不到。
2. **回零陈旧速度漂移**（无需果实）：−0.058 rad/s 恒速漂移，对增益不敏感，200 Hz 仅减半（0.035）——命令管线偏斜，插件层现象。

## 流程真实完成度

观察 → 接近 → 抓取（接触解析附着）→ 撤回 → 提升 → 走廊 → 对准 → **释放** 全部机械成功；唯一未完成的是带果/空气回零段。`harvested=0` 是料箱验证未运行，非抓取失败。

## 遗留可选项（未做，需再决策）

- A4：update_rate 50 Hz（与 200 Hz 反向实验）
- A5：焊点 link7 → panda_hand（动冻结基线）
- 插件级修复（升级/改源）

## 关键文件

- 收据：`.codex_tmp/generalized_harvest_seed_45504_v2[0-6]_*/v*_findings_receipt.json`
- `issue629_comparison.json`（上游对照：closed not_planned，无修复可等）
- 契约测试：`test_effort_gravity_comp_contract.py`（Plan-A 契约）
- 零速窗口：`moveit_backend.py::_wait_for_zero_velocity_between_goals`

## 追加轮次（v25–v33，插件加载链与 GC 路线闭环）

| 探针 | 提交 | 变量 | 结果 |
|---|---|---|---|
| v25/v25b | `1fb69cd` | gain 2.0 | 控制稳定但观察规划系统性贴边拒绝（OMPL 种子偏移，14 次拒绝）|
| v26 | `19103a0` | gain 1.5 | 漂移对增益不敏感（0.057–0.063）→ 增益假设否定 |
| v27–v29 | `b9df7bf`/`05d0333` | GC 插件加载链 | 三层阴影（GZ plugin path / LD_LIBRARY_PATH / AMENT_PREFIX_PATH）逐层验证，`compensate_gravity` 参数始终未解析 |
| v30 | `40c404a` | 插件文件绝对路径参数化 | 加载确定性达成（rename 实验证实）|
| v31 | `ad19ed1` | +force_torque 传感器 | 首块放错位置（ros2_control 内）→ gz SIGABRT |
| v32 | `4a7641d` | 传感器块移出 ros2_control | 插件加载成功，但 `compensate_gravity` 参数未解析 |
| v33 | `f186fac` | 最终 Plan-A 配置锁定 | 完整复现 v22：流程到释放，回零受两机制限制 |

### 最终定性（v32 关键发现）

`compensate_gravity` 的实现读取 `JointTransmittedWrench` 组件，而 **Jazzy 的 `hardware_interface::InterfaceInfo` 结构没有 parameters map**（name/min/max/initial_value/data_type），该分支依赖的参数解析特性仅存在于 Rolling/Kilted。**GC 路线在 Jazzy 上被 API 硬阻断**，与配置无关。

### 最终配置（Plan-A final，契约测试锁定）

- 臂：position 接口 ×7，JTC 容差 0.05
- 防御纵深保留：零速窗口（按名映射）+ 静止头 + 一次踢签名重试 + 200 Hz
- 流程能力：观察→接近→抓取→搬运→对准→释放 **可复现**；回零受限于两个插件物理/管线层缺陷（A3 记录为已知限制）
- 重启路径：插件源码补丁 / Rolling-Kilted 升级 / Bullet 物理引擎
