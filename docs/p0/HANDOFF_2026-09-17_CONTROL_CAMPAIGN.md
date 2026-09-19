# STRAWBERRY_URP 交接文档（控制实验战役收官 · 2026-09-17）

**项目地址（GitHub）**：https://github.com/zl7036138-cmd/strawberry_urp
**分支**：`codex/wp01-evidence-validation`　**HEAD**：`6fcd27c`（本地=远程，工作区干净）
**Windows 路径**：`C:/Users/12753/Documents/New project/strawberry_urp`（WSL: `/mnt/c/...`，发行版 `Ubuntu-24.04-URP`，Jazzy + gz Harmonic，venv `/opt/strawberry_venv`）
**colcon 缓存**：`~/.cache/strawberry_urp/colcon`

---

## 一、当前能力（可复现事实）

- **单果动作链已打通并可复现**（v22/v23/v26/v33 四次独立运行）：
  观察 → 接近（APPROACH）→ 抓取（接触解析附着，双指 0.024/0.027）→ 撤回 → 提升 → 清空走廊 → 重定向 → 降至转运高度 → 走廊平移 → 对准料箱上方 → 张爪释放
- **未完成**：入箱验证（`placed_sim_target_ids=[]`，`harvested_distinct_count=0`）、放置验证门（VERIFY 未到达）、回零——全部止于下述两个插件层缺陷
- 基线测试：857 纯 Python + 233 manipulation + 116 sim + 145 bringup 全绿
- 正式 30 种子矩阵与 115 张封存测试集**全程未触碰**

## 二、两个残余缺陷（gz_ros2_control + DART + Jazzy 的结构性问题）

1. **焊接冲量踢**（需果实附着）：新目标被接受瞬间 +0.5 rad/s 反向冲击 ~0.4s，0.1s 内突破 0.05 容差。机理：DetachableJoint（果焊在 link7）的投影修正力，物理层。
2. **回零陈旧速度漂移**（空手也发生）：−0.058 rad/s 恒速漂离（200 Hz 下降到 0.035），对增益不敏感（1.0/1.5/2.0 三组数据）。机理：插件执行旧段速度指令、不按误差闭环，管线层。

## 三、已尝试并被否定的方案（勿重复）

| 方案 | 轮次 | 结论 |
|---|---|---|
| effort 接口 + PID/重力补偿 | v12–v20 | 速度踢随惯量反比放大（j7 ±2.871 钳位 bang-bang，4 组增益同签名）|
| 混合控制器（j1-6 effort + j7 position） | v20 | j7 治好，但 j4 被踢暴露 effort 整体不可用 |
| 零速窗口 / 静止头 / 踢签名重试 | v21–v23 | 窗口与头无法触及目标内踢；重试 4/4 再败（冲量持续）|
| 200 Hz | v24 | 漂移减半（0.058→0.035）但方向不变——保留 |
| gain 2.0 / 1.5 | v25/v25b/v26 | 漂移对增益不敏感；2.0 还破坏观察规划（OMPL 贴边）|
| 上游 `add/gravity_compensation` 分支 | v27–v32 | 插件加载链修复完成（绝对路径加载已验证），但 `compensate_gravity` 读 `JointTransmittedWrench` 且解析需要 hardware_interface 的 parameters map——**Jazzy 的 InterfaceInfo 没有该字段，被 API 硬阻断** |

## 四、最终锁定配置（契约测试 `test_effort_gravity_comp_contract.py`）

- 臂：position 接口 ×7，JTC 容差 0.05，`update_rate: 200`
- 插件：系统 `gz_ros2_control-system`，`position_proportional_gain: 1.0`（1.5/2.0 已试）
- 防御纵深（保留在后端 `moveit_backend.py`）：零速确认窗口（按名映射速度）+ 静止目标头 + 一次踢签名重试 + 路径边细分
- 决策依据：`docs/p0/CONTROL_EXPERIMENT_CAMPAIGN_2026-09-16.md`（33 轮矩阵 + 两机制定性 + 被否定方案清单）

## 五、继续方向（按成本排序）

1. **A3（已选，完成）**：记录为仿真限制。真机语义 = Franka `setLoad` + 力矩接口重力补偿（与仿真缺陷无关）
2. **物理引擎实验**：`physics="bullet_featherstone"`（Gazebo Harmonic 原生支持），改一行配置，直接用 v26 探针 A/B——最便宜的未试变量
3. **插件源码补丁**：fork gz_ros2_control，修 position 路径的陈旧速度保持（stale-command hold）与焊接冲量透传——工程量 1–2 周
4. **升级 Rolling/Kilted**：解锁 `compensate_gravity`（需 hardware_interface parameters 特性）——整机迁移
5. **作业语义变体**：提升后提前释放（绕开 align 踢，需改验证逻辑——动作业语义，需用户认可）

## 六、基础设施（全部可复用）

- **遥测**：`arm_telemetry.py` + `scripts/record_arm_telemetry.py`（双时钟、命令因果链、单轮 7 万+样本）
- **证据**：`motion_evidence.py`（run/scenario 强制绑定）+ v9–v33 收据链（`docs/p0/CONTROL_EXPERIMENT_CAMPAIGN_2026-09-16.md` + 各探针目录 `v*_findings_receipt.json` + `issue629_comparison.json`）
- **探针**：`.codex_tmp/run_v*_probe_with_telemetry.sh`（改目录名即可复用）
- **契约**：`test_effort_gravity_comp_contract.py`（Plan-A final）+ 停稳门/资产测试

## 七、运行要点

- 探针脚本示例：`.codex_tmp/run_v33_probe_with_telemetry.sh`（改 seed/目录名）
- 构建命令：`python "$(command -v colcon)" build --symlink-install --base-paths ros2_ws/src --packages-select <pkg>`（在 WSL 内）
- 推送 GitHub 靠用户本机代理，`443` 间歇阻断（api.github.com 可达），失败就重试并 `git ls-remote` 验证
- Gazebo 有头模式默认 PAUSED（历史教训：先解除暂停再等控制器）
- `/tmp` 在 WSL 会话间隔离，持久文件放 `.codex_tmp` 或 `$HOME`

## 八、验收口径提醒

`passed ≠ 需求通过`；`harvested_distinct_count` 的记分口径 = 焊接附着→料箱验证→分离确认的完整链，释放动作本身不算采摘成功。G1（单批次≥1 果完整闭环含验证）与单批次多果仍未达成。
