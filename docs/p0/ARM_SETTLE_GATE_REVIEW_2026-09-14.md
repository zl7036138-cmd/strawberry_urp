# 手臂停稳判定修复与 v7 失败诊断

本轮修复了手臂停稳判定中的三类假阳性，工程测试通过；seed 45504 的 v7 开发探针仍为 `RECOVERY_HOME_FAILED`，未证明采摘闭环或 G1 运动安全。

## 证据与修改范围

工作基线为 `c94b6a9c333ea9491d7b35f4205b49de59f37c2f`。进入本轮时已有四个未提交文件，涉及 place 走廊、wrist-roll 候选、home 容差及对应测试。它们作为继承改动保留，不计为本轮成果；本轮未创建提交、推送或改写历史。

本轮实际代码修改限定在 `MoveItBackend._wait_until_arm_settled()`：

1. 未收到实测关节状态时，不再用规划场景缓存证明停稳。
2. 连续有效采样之间的间隔达到原有停稳窗口时，重新累计窗口；重复或倒退的本地序号不计为新采样。
3. 空样本、维度错误、NaN/Inf 直接拒绝停稳确认。

原有超时、停稳窗口、位移阈值、控制器容差、关节速度、料箱净空与采摘路径配置均保留。曾暂时加入但未经验证的 joint6 降速覆盖已撤回，不属于交付改动。

新增 `ros2_ws/src/strawberry_manipulation/test/test_arm_settle_gate.py`，直接调用生产回调及停稳判定，以受控时钟和关节消息替代 ROS 传输。原有 `test_moveit_backend.py` 仅为两处单关节测试夹具补充关节名。

## RED → GREEN 验证

以下三种场景均先在生产旧实现上得到预期的 `AssertionError: True is not false`，再逐项修复：

| 缺陷 | 失败证据 | 修复后行为 |
|---|---|---|
| 无 `/joint_states`，但规划缓存不变 | `red_cached_scene.xml` | 到超时仍无法确认停稳 |
| 三次相同样本跨越长反馈中断 | `red_feedback_gap.xml` | 中断不能计入连续稳定窗口 |
| joint6 为 NaN，其余关节不变 | `red_nonfinite.xml` | 拒绝无效反馈 |

新增文件共 10 项测试，还覆盖新鲜稳定数据的正常放行、单个缓存样本、部分关节消息、慢漂移、恢复反馈后重新计时、维度/Inf 错误及重复/倒退的本地序号。

| 验证入口 | 结果 |
|---|---|
| 目标测试：新文件 + `test_moveit_backend.py` | 77 passed |
| 仓库 `scripts/run_pure_tests.py` | 807 tests，0 failures / errors / skipped |
| manipulation 分包 pytest | 189 passed |
| bringup 分包 pytest | 145 passed |
| 两包 colcon test，并分别读取 test-result | 189 与 145，均 0 failures / errors / skipped |
| venv Python 调用 colcon build | 2 packages finished |
| 安装后导入源码位置与停稳方法检查 | 指向当前 Windows 磁盘挂载源码，修复存在 |
| `git diff --check` | 通过 |
| `git apply --reverse --check` 继承补丁 | 通过，仅检查，未执行撤回 |

这些测试集互相重叠，不能相加成独立测试或需求数量。曾将两个包放入同一 pytest 进程，出现同名 `test_core.py` 的 `ImportPathMismatchError`；保留 `regression.log/xml`，最终采用分包进程与仓库全量入口。构建仍有 setuptools 的 `Unknown distribution option: tests_require` 警告，未将其隐藏或描述为零警告。

## v7 开发探针保留的阴性结果

运行目录：`.codex_tmp/generalized_runtime_dev_v7_seed45504`。运行使用开发场景 45504、640×480 基座 RGB-D；进程退出码 4。评分与原始事件均保留：

- recorder / terminal outcome：`RECOVERY_HOME_FAILED`；评分 `evidence_status=FAIL`。
- `harvested_distinct_count=0`；无已确认物理采摘；正式终态确认计数未知。
- approach 阶段 `panda_joint7` 跟踪误差 0.066519 rad，超过控制器 0.050000 rad 容差。
- home 第 1/7 段再次因 `panda_joint7` 误差 0.050089 rad 超差终止，代码停止自动重试。
- 关节编号勘误：原始日志 `State tolerances failed for joint 6` 的 6 是从零计数的数组下标，不是关节名。已核对本机 `/opt/ros/jazzy/include/joint_trajectory_controller/joint_trajectory_controller/tolerances.hpp` 第 300、319 行及已安装控制器的关节顺序；该下标对应 `panda_joint7`。报告旧快照中“关节 6 / joint6”的运行故障定位由本条更正。上述 NaN 单元测试仍确实注入 `panda_joint6`，它与运行故障定位无关。
- 更早出现过规划起点与新关节样本偏差 0.071497 rad，执行前保护拒绝该路径。
- truth isolation audit 通过；清理收据为 CLEAN，并检查运行进程组 11454 不再存在。
- 缺失独立碰撞、关节限位、附着和场景终态证据。日志中的零计数不能证明“零碰撞”或完整安全。

日志没有记录完整的期望/实际轨迹、速度及双时钟关节样本，因此尚不能在速度过快、状态滞后、物理/控制耦合等假说之间判定原因。停稳判定缺陷是离线可复现的独立问题，不能直接归因为此次 `panda_joint7` 失败的原因，也没有运行修复后的集成探针来验证因果或性能收益。

## 来源与门控限制

`PHASE_P0_REVIEW_PACKET.md` 写的是请求 `PROCEED_WITH_LIMITS`，并要求显式 G0 决定；不能把建议写成已放行。当前文档未提供可核验的 G0/G1 放行记录。v7 在此核验完成前已执行，未绑定符合 P0 的运行身份，故只保留为失败诊断，不纳入正式安全/性能验收；此后未继续集成运动。

本轮源码仍包含新增未提交开发改动，不能与旧的九文件 inherited ledger 冒名匹配。工程测试确实执行并通过，但不宣称它们是符合 P0 准入的正式结果。保存源码快照、差分、文件 SHA-256 和测试原始收据用于复核；未伪造 canonical run identity，未改写旧 ledger。正式证据应在合法冻结源码后重新捕获。

Formal-30 与真实 115 图测试集保持封存；未执行这两类受保护资源的行为或评分。

## 未解决事项与下一工作边界

- 当前序号仍是回调到达计数，缺采集时间戳、独立速度与 motion epoch；重放同一带时间戳消息、暂停仿真时钟等情况仍需要后续契约实现。
- 长反馈中断修复使用原有 `settle_window_sec` 判断，未宣称更短间隔具备连续监测保证。
- 操作所有权、超时/取消后的物理停止、独立终态监视仍需按 `NEXT_PHASE_HANDOFF.md` 的 P1 接口完成。
- `panda_joint7` 失败应先加入期望与实际轨迹的同步记录，再作有依据的单变量变更。
- place/wrist-roll 运行时验证及单批次多果采摘依旧未证明。

本轮结果与机器可读清单：`results/development/arm_settle_gate_20260914/`。原始 RED/错误/回归/构建日志另存 `.codex_tmp/settle_gate_review_20260914/`，不会覆盖 v7 或历史结果。

独立复核的编号勘误与核验过的头文件/控制器配置另存 `results/development/arm_settle_gate_20260914/independent_review/`；其中 `summary_corrected.json` 更正原 `summary.json` 对运行故障关节的标注，旧汇总与其哈希清单原样保留。复核后没有改变生产代码、速度或安全阈值，测试数采用本轮完整验证的 807 / 189 / 145，各入口重叠不相加。
