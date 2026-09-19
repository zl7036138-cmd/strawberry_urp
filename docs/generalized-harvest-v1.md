# 固定机械臂多植株连续采摘 v1

## 当前实现

这一扩展保留原有固定场景作为回归基线，新增一条独立的广义采摘路径：

1. 由固定种子生成 1～3 株植株、每株 2～3 颗草莓的场景，同时输出匹配的 YAML、SDF 和 SHA-256 回执。
2. 底座相机输出全部成熟和未成熟候选；RGB-D 定位节点不再只处理最高置信度目标。
3. 三维跟踪器使用 60 mm 几何门限维护跨帧稳定 `track_id`，运行时不读取 Gazebo 真值身份。
4. 选择器按成熟度、数据新鲜度、观测次数、定位标准差、果实/收集箱间隙和运动代价确定性排序。
5. 每个目标生成 12 个有限腕部视点，转换为 Panda 手部位姿；MoveIt 在运动前检查预抓取、抓取、撤离 IK 与插值碰撞状态。
6. `/strawberry/run_harvest` 依次执行观察、腕部视觉确认、抓取、放置和重新扫描。失败目标只允许一次重新观察；若重试时已不可见，也会明确标记跳过并继续扫描其他目标。

广义模式中，规划碰撞球来自 `/strawberry/tracked_targets`。定位节点默认关闭真值关联，并且在关闭时根本不创建真值话题订阅。Gazebo 实体身份只由仿真适配层在真实双指接触后解析，用于 attach/detach；评分记录器可以在行为结束后读取真值事件，但这些数据不能回流到成熟度判断、候选排序、目标位姿修正或路径选择。

## 生成并运行一个新场景

在已构建并加载 ROS 2 工作区的终端中执行：

```bash
ros2 run strawberry_sim generate_generalized_scene \
  --base-scene ros2_ws/src/strawberry_sim/config/scene.yaml \
  --base-world ros2_ws/src/strawberry_sim/worlds/strawberry_orchard.sdf \
  --output-dir results/generalized/dev_seed_17036 \
  --seed 17036 \
  --profile mixed \
  --plant-count 3 \
  --position-band middle \
  --occlusion partial
```

然后启动双相机系统：

```bash
ros2 launch strawberry_bringup generalized_harvest.launch.py \
  world_file:=$PWD/results/generalized/dev_seed_17036/generalized_seed_017036.sdf \
  scene_config_file:=$PWD/results/generalized/dev_seed_17036/generalized_seed_017036.yaml \
  model_path:=$PWD/outputs/perception/yolo11s_640_generalized_dev_v2/weights/best.pt \
  headless:=false
```

系统稳定后启动一次连续采摘：

```bash
ros2 service call /strawberry/run_harvest std_srvs/srv/Trigger "{}"
ros2 topic echo /strawberry/harvest_status
```

关键运行接口：

- `/strawberry/tracked_targets`：全部稳定/正在稳定的三维目标；
- `/strawberry/target_pose`：当前安全排序选中的目标；
- `/strawberry/wrist_observation_pose`：动态计算的 Panda 手部观察位姿；
- `/strawberry/wrist_observation_plan`：同一目标的有限观察位姿组，按确定性优先级排列；
- `/strawberry/evaluate_target`：不执行轨迹的 IK/碰撞预检查；
- `/strawberry/move_to_observation`：碰撞检查后的腕部观察运动；
- `/strawberry/run_harvest`：启动连续采摘；
- `/strawberry/harvest_status`：批次状态、成功、跳过、失败和剩余目标。

选择器先做保守工作区、视野、果实间距和收集箱外轮廓间隙过滤，然后发布最多12个候选观察位姿。连续采摘控制器按顺序调用 MoveIt：规划、IK 或碰撞检查失败且机械臂尚未运动时才尝试下一个；一旦轨迹执行已经开始，任何失败都会立即按安全失败处理，不会继续试探其他视角。腕部确认结果使用带系统误差下限的方差加权融合，避免单次近距离观测以虚假的高置信度覆盖稳定的底座相机定位。

## 开发验证状态

截至 2026-09-01，当前里程碑的完整纯 Python 回归为 `752 tests, 0 failures, 0 errors, 2 skipped`，完整 ROS 2/colcon 回归为 `634 tests, 0 failures, 0 errors, 0 skipped`。无运动真实 ROS 图审计已通过：定位、跟踪、选择、编排和操作控制节点均未订阅真值话题；只有仿真适配/评分边界可以读取真值。

开发发现种子 `45001～45018` 已按冻结顺序完成两轮无运动筛选。18/18 场景都清理为 `CLEAN`，18/18 真值隔离审计通过；29 条候选轨迹中只有 7 条通过 MoveIt，只有种子 `45007` 同时拥有至少两条稳定、成熟且 MoveIt 可行的轨迹。抓取姿态现已保证预抓取和抓取使用同一滚转分支，但这一正确性修复没有改变 1/18 的场景资格率。它说明当前瓶颈是可行工作区/场景分布，不是通过放宽成熟度、15 mm 不确定度、50 mm 腕部修正、碰撞或重试阈值可以合理解决的问题。

在冻结提交 `9300ce9` 上，从未调试的资格种子 `46001～46018` 也已完成一次无运动筛选：18/18 探针完成、18/18 真值隔离通过、18/18 清理为 `CLEAN`，且所有场景都明确禁止轨迹执行。68 条候选中有 26 条成熟轨迹进入 MoveIt，7 条可行，但只有 `46016` 一场具有两条可行成熟轨迹。由于少于预先要求的五场，筛选器拒绝启动行为批次；没有单独挑选 `46016` 做抓取，也没有打开正式矩阵。机器可读结论见 `config/generalized_runtime_progress_v15.json` 与 ADR 0079。

随后对筛选方向进行了纠正。旧门槛只证明“检测器建立了两条稳定成熟轨迹且 MoveIt 可行”，却没有先证明场景中至少两颗成熟果在底座 RGB-D 深度图上真实可见且三维定位正确。开发种子 `45101` 与 `45104` 共生成6颗成熟果，只有4颗通过深度可见性判定，最终只有2颗在30 mm内正确定位；可见成熟果定位召回率为50%，成熟定位精确率为66.67%。种子 `45104` 的 MoveIt 探针单独判定可进入多果运行，但同帧3颗深度可见成熟果只正确定位1颗，并产生1个错误的已接受成熟位置，因此新的联合门明确拒绝该场景。默认相机安装位姿保持不变，参数化接口只用于跨场景比较固定硬件布局，不允许按目标或种子动态调相机。

开发矩阵 v2 现在要求顺序同时成立：至少2颗深度可见成熟真值、至少2颗在30 mm内正确定位的成熟真值、错误接受的成熟位置为0，并且至少2条稳定成熟轨迹通过 MoveIt 连通路径评估。深度真值只存在于无运动开发评分器中，运行时定位、选择和控制仍不读取真值。机器可读结论见 `config/generalized_runtime_progress_v16.json` 与 ADR 0080。

新鲜发现块 `45201～45218` 已在冻结提交 `69dce3c` 上完成：18/18 场景物化、真值隔离和进程清理均通过，且没有执行轨迹。全部18场都有至少2颗深度可见成熟果，但只有6场通过完整视觉门、5场通过旧 MoveIt 门，两组场景完全不重叠。64颗深度可见成熟果中35颗定位正确；47个已接受成熟位置中只有35个正确，说明旧运行时会让系统性深度偏差形成稳定轨迹。近区视觉门通过5/6但 MoveIt 通过0/6；远区视觉门通过0/6而 MoveIt通过5/6，证实当前问题是相机测量质量与机械臂可行工作区的错位，而不是单独一侧的阈值问题。

对五个旧 MoveIt 合格远区场景的回放进一步发现：同一固定视角的相关帧会把50～128 mm的系统性深度偏差错误平均到5 mm不确定度，底座启动时额外乘0.5又进一步低估风险。现在底座几何残差权重恢复为1.0，跟踪器在视角重置前保留历史不确定度、当前不确定度和位置创新三者的最大值；腕部相机仍保留已验证的0.5校准。五个场景复验全部在运动前安全拒绝，MoveIt评估从13条降到5条，8条轨迹明确标记 `HIGH_UNCERTAINTY`，另一个场景在本次取样窗口内没有形成可评估成熟候选；5/5真值隔离和清理通过。机器可读结论见 `config/generalized_runtime_progress_v17.json` 与 ADR 0081。

下面的 v5～v14 文字是保留的开发时间线，用于解释问题如何被发现和修复；其中的旧测试数量和当时的“尚未成功”结论不是当前项目状态：

最近一次 v5 完整 ROS 2 回归当时为 `505 tests, 0 errors, 0 failures, 0 skipped`，对应定向测试为 `31 passed`；机器可读结果见 `config/generalized_runtime_progress_v5.json`。该阶段的随机开发场景给出以下结论：

1. 开发种子 `17036` 已实际经过“底座发现 → 安全选择 → 动态腕部观察 → 腕部确认 → 发起抓取”。精确真值隔离诊断证明抓取失败的直接原因是夹爪右指接触收集箱背板，而不是目标实体映射错误。选择器现已把收集箱外轮廓作为静态障碍；同一风险目标会返回安全 `NO_PICK`。
2. 现有固定场景视觉模型在额外随机开发种子上的多目标召回不足：`17037`、`17038` 各只形成一个稳定成熟轨迹且均位于收集箱风险区，`17039`、`17042` 没有形成检测。它因此尚未达到进入正式隐藏评测的开发门槛。

此后已完成全部120个开发场景，并以 v2 配置微调广义检测器。冻结运行参数为图像尺寸800、置信阈值0.20524564385414124、NMS IoU 0.50：验证集成熟果精确率96.15%、召回率90.91%；同一参数在只执行一次的独立资格集达到精确率95.65%、召回率97.78%。检测器资格门已经通过。

在线瓶颈仍在 RGB-D 定位与安全运动闭环。开发种子44001形成4条稳定轨迹并安全拒绝风险目标，同时暴露并修复了“失败果实换新ID回流”的问题。针对开发种子44003新增的只读逐框 RGB-D 诊断发现：旧逻辑会让17像素的弱深度层压过280像素的真实果实层。修复后，该成熟果的评分定位误差为4.3 mm，在线稳定成熟轨迹从1条增加到2条；风险目标仍被安全拒绝为 `NO_PICK`，没有降低安全阈值。

对开发种子44012的完整重放纠正了此前的初步判断：日志中的0.854 m是被拒绝深度层的果实尺寸一致性残差，并不是底座与腕部定位差；新鲜运行也没有复现起始状态越界。真正的问题是腕部视野同时出现多颗成熟果时，局部定位器会按最小不确定度选中另一颗果实。现在底座相机选中的三维目标会作为50 mm范围的空间提示传给腕部定位器，腕部轨迹在观察运动后清空重建，确认消息继续使用底座拥有的跟踪ID；ID不匹配、非有限值、修正过大、低置信度和高不确定度都会给出稳定拒绝原因并安全失败。

加入腕部专用的保守不确定度校准后，同一种子已经跨过腕部确认，实际记录到 `APPROACH → GRASP_POSE → RETREAT → PLACE`。但记录器在动作仍为 `PICK_SENT` 时达到墙钟超时，没有收到终态结果，所以这只能证明流水线已推进到受保护的放置阶段，仍不能宣称一次完整抓放成功。冻结的历史定位核心已恢复到清单记录的27,119字节和原SHA-256；泛化场景的支持度排序与腕部校准放在独立模块中。定向测试78项、完整回归493项全部通过。正式30种子继续封存。

随后已把临时探针升级为带启动、无进展和绝对硬上限的正式开发记录器，并将机械臂 `PLAN/APPROACH/GRASP/RETREAT/PLACE/VERIFY/DONE` 反馈纳入回执。腕部静止确认窗口从1.5秒增加到3秒，但15 mm不确定度、50 mm修正距离、置信度、碰撞和一次重试门槛均未放宽。开发种子44012的新终态回执为 `PARTIAL_SUCCESS`：目标2在首次抓取中因夹爪接触或仿真附着失败，第二次确认超时后被跳过；系统没有终止批次，而是继续自主选择目标7，完整执行抓取、撤离、放置、验证和 `DONE`，最终把目标7记录为 `HARVESTED`。这已经证明一颗感知控制草莓的完整抓放，也证明单目标失败后能继续处理其他目标；尚未证明同一批次成功采摘两颗或采完全部可达成熟果。

进一步的v6开发诊断修复了三类底座RGB-D危险失效：弱前景层误删果面、像素阈值切出伪小深度层，以及可见果面质心带来的单侧遮挡方向偏差。最终代码在6个开发种子的只读真值评分中定位19个目标，19/19均低于30 mm，最大误差9.22 mm；12个通过15 mm控制门限的目标最大误差7.40 mm，其余7个全部因高不确定度拒绝。完整回归为510项全通过，冻结定位核心未修改。运行时复验仍未取得同批两果成功，因此总体泛化门仍未通过，30个正式隐藏种子继续封存；详见 `config/generalized_runtime_progress_v6.json` 与ADR 0069。

开发运行器同时修复了 `setsid` 后过早查询PGID的竞态：现在直接使用子进程PID作为进程组ID，只有连续确认ROS/Gazebo组为空才写入 `cleanup_probe: CLEAN`。独立就绪后清理冒烟以退出码0结束且无残留进程。正式30种子仍继续封存。

### 当前多果开发门

当前开发矩阵为 `config/generalized_runtime_development_matrix_v2.json`。发现块 `45101～45118` 已用于观测门开发，`45201～45218` 已用于不确定度修复前的完整发现及修复后代表场景复验；由于运行时行为已经改变，下一轮新鲜发现必须使用 `--batch-index 2`，即 `45301～45318`。v2 资格块 `46101～46118` 尚未使用，只有远区测量质量在新鲜发现块上改善并冻结代码后才能运行。先运行不执行机械臂轨迹的筛选：

```bash
python scripts/run_generalized_feasibility_sweep.py \
  --matrix config/generalized_runtime_development_matrix_v2.json \
  --split discovery \
  --batch-index 2 \
  --output-dir .codex_tmp/generalized_feasibility_discovery_b02
```

只有干净且冻结的 Git 提交才能运行 `--split qualification`。筛选器先用只读真值评分确认至少两颗成熟果在底座深度图中可见、至少两颗定位误差不超过30 mm且不存在错误接受的成熟位置，再要求至少两条稳定成熟轨迹通过 MoveIt；联合门全部成立才算一个合格场景。筛选器按矩阵顺序选择前五场，不足五场时必须停止并报告，不能降低安全门，也不能启动行为批次。正好选满五场后，才可在同一提交上各执行一次：

```bash
python scripts/run_generalized_qualification_batch.py \
  --sweep-summary .codex_tmp/generalized_feasibility_qualification_b00/sweep_summary.json \
  --output-dir .codex_tmp/generalized_runtime_qualification_b00
```

开发记录 schema v3 保存完整 `/strawberry/selection_status` 事件；运行后评分器只用这些控制事件和物理接触/放置事件做证据关联。通过要求保持为 5/5 收到终态且清理干净、全部安全计数为 0、至少 4/5 同批采摘两颗不同目标、被接受成熟目标完整抓放成功率至少 80%，且每个失败目标最多一次重新观察。

开发数据工具链已冻结在 `config/generalized_development_capture_v1.json`：72个训练种子、24个验证种子和24个独立资格种子，与正式矩阵零重叠。采集器把同一画面中的所有可见果实写成多行 YOLO 标签，并用深度支持度剔除完全遮挡的真值投影框；资格集不会出现在训练配置中。种子 `41001` 的 Gazebo 冒烟采集已成功生成3个可见标签（2个成熟、1个未成熟）。

完整采集命令：

```bash
bash scripts/run_generalized_development_capture.sh
```

采集脚本完成全部120场景后才会生成数据清单、哈希回执、训练用 `dataset.yaml` 和训练不可见的 `qualification.yaml`。审核清单后，可使用单独冻结的低学习率微调配置：

```bash
yolo detect train \
  cfg=ros2_ws/src/strawberry_perception/config/train_yolo11s_640_generalized_dev_v2.yaml
```

训练完成后，先在验证集扫描阈值；只有验证门通过，才能把同一阈值、图像尺寸和 NMS IoU 固定后在资格集执行一次：

```bash
ros2 run strawberry_benchmark generalized-detector-eval \
  --model outputs/perception/yolo11s_640_generalized_dev_v2/weights/best.pt \
  --dataset-root data/processed/generalized_development_capture_v1 \
  --split validation \
  --imgsz 800 --nms-iou 0.50 --iou-threshold 0.50 \
  --output outputs/perception/yolo11s_640_generalized_dev_v2/validation_gate.json

ros2 run strawberry_benchmark generalized-detector-eval \
  --model outputs/perception/yolo11s_640_generalized_dev_v2/weights/best.pt \
  --dataset-root data/processed/generalized_development_capture_v1 \
  --split qualification \
  --threshold 0.20524564385414124 \
  --imgsz 800 --nms-iou 0.50 --iou-threshold 0.50 \
  --output outputs/perception/yolo11s_640_generalized_dev_v2/qualification_gate.json
```

资格集不得重新扫描阈值；`config/generalized_harvest_matrix_v1.json` 中的正式种子不得用于训练、调参或挑选模型。

## 30 种子正式矩阵

冻结配置位于 `config/generalized_harvest_matrix_v1.json`，由18个正样本、6个全未成熟负样本和6个不可达/不安全样本组成。正样本均衡覆盖近、中、远位置与无、部分、重度遮挡。

本节命令只记录封存流程，不代表已经授权执行。开发门全部通过后，仍需先在干净冻结提交上创建单次 claim；claim 会绑定矩阵、物化清单、模型、运行配置和 Git 提交，并拒绝覆盖已有输出。本轮没有创建或消费正式 claim。

只物化场景、不运行行为：

```bash
ros2 run strawberry_benchmark materialize-generalized-harvest \
  --matrix config/generalized_harvest_matrix_v1.json \
  --base-scene ros2_ws/src/strawberry_sim/config/scene.yaml \
  --base-world ros2_ws/src/strawberry_sim/worlds/strawberry_orchard.sdf \
  --output-dir results/generalized/formal_v1
```

收集完30条一次性正式结果后执行：

```bash
ros2 run strawberry_benchmark generalized-harvest-acceptance \
  --matrix config/generalized_harvest_matrix_v1.json \
  --manifest results/generalized/formal_v1/materialization_manifest.json \
  --results results/generalized/formal_v1/results.json \
  --output results/generalized/formal_v1/summary.json
```

评测器会同时检查检测召回/精确率、ID切换、定位 P95、标准差、错误采未成熟果、碰撞、负样本安全停止、单目标成功率和完整场景完成率。缺失、重复或额外的场景都会直接失败。

## 尚未宣称的结果

代码、接口、单元测试、场景物化和严格 schema-v2 验收器已经完成，并不等于30种子运行已经通过。广义检测器已经通过独立资格门，且一次不依赖运行时真值控制的完整单果抓放已经证明；尚未证明的是同一批次成功采摘两颗以上，以及五场多果开发门。正式30种子没有创建 claim，也没有执行正式行为试验。只有开发门先通过，并在同一冻结提交上实际完成全部30个一次性场景、生成 `overall_pass: true` 的汇总后，才能声称达到计划中的泛化指标。现有固定场景和历史 P3/P4 结论保持不变。
