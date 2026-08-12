# 固定机械臂多植株连续采摘 v1

## 当前实现

这一扩展保留原有固定场景作为回归基线，新增一条独立的广义采摘路径：

1. 由固定种子生成 1～3 株植株、每株 2～3 颗草莓的场景，同时输出匹配的 YAML、SDF 和 SHA-256 回执。
2. 底座相机输出全部成熟和未成熟候选；RGB-D 定位节点不再只处理最高置信度目标。
3. 三维跟踪器使用 60 mm 几何门限维护跨帧稳定 `track_id`，运行时不读取 Gazebo 真值身份。
4. 选择器按成熟度、数据新鲜度、观测次数、定位标准差、果实/收集箱间隙和运动代价确定性排序。
5. 每个目标生成 12 个有限腕部视点，转换为 Panda 手部位姿；MoveIt 在运动前检查预抓取、抓取、撤离 IK 与插值碰撞状态。
6. `/strawberry/run_harvest` 依次执行观察、腕部视觉确认、抓取、放置和重新扫描。失败目标只允许一次重新观察；若重试时已不可见，也会明确标记跳过并继续扫描其他目标。

广义模式中，规划碰撞球来自 `/strawberry/tracked_targets`。Gazebo 真值只在仿真适配层中把感知选中的三维位置映射到实际可分离的模型实体，以完成 attach/detach；它不参与成熟度判断、候选排序、目标位姿修正或路径选择。

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

最近一次完整 ROS 2 回归为 `505 tests, 0 errors, 0 failures, 0 skipped`，本次改动对应的定向测试为 `31 passed`；机器可读结果见 `config/generalized_runtime_progress_v5.json`。随机开发场景运行还给出了以下结论：

1. 开发种子 `17036` 已实际经过“底座发现 → 安全选择 → 动态腕部观察 → 腕部确认 → 发起抓取”。精确真值隔离诊断证明抓取失败的直接原因是夹爪右指接触收集箱背板，而不是目标实体映射错误。选择器现已把收集箱外轮廓作为静态障碍；同一风险目标会返回安全 `NO_PICK`。
2. 现有固定场景视觉模型在额外随机开发种子上的多目标召回不足：`17037`、`17038` 各只形成一个稳定成熟轨迹且均位于收集箱风险区，`17039`、`17042` 没有形成检测。它因此尚未达到进入正式隐藏评测的开发门槛。

此后已完成全部120个开发场景，并以 v2 配置微调广义检测器。冻结运行参数为图像尺寸800、置信阈值0.20524564385414124、NMS IoU 0.50：验证集成熟果精确率96.15%、召回率90.91%；同一参数在只执行一次的独立资格集达到精确率95.65%、召回率97.78%。检测器资格门已经通过。

在线瓶颈仍在 RGB-D 定位与安全运动闭环。开发种子44001形成4条稳定轨迹并安全拒绝风险目标，同时暴露并修复了“失败果实换新ID回流”的问题。针对开发种子44003新增的只读逐框 RGB-D 诊断发现：旧逻辑会让17像素的弱深度层压过280像素的真实果实层。修复后，该成熟果的评分定位误差为4.3 mm，在线稳定成熟轨迹从1条增加到2条；风险目标仍被安全拒绝为 `NO_PICK`，没有降低安全阈值。

对开发种子44012的完整重放纠正了此前的初步判断：日志中的0.854 m是被拒绝深度层的果实尺寸一致性残差，并不是底座与腕部定位差；新鲜运行也没有复现起始状态越界。真正的问题是腕部视野同时出现多颗成熟果时，局部定位器会按最小不确定度选中另一颗果实。现在底座相机选中的三维目标会作为50 mm范围的空间提示传给腕部定位器，腕部轨迹在观察运动后清空重建，确认消息继续使用底座拥有的跟踪ID；ID不匹配、非有限值、修正过大、低置信度和高不确定度都会给出稳定拒绝原因并安全失败。

加入腕部专用的保守不确定度校准后，同一种子已经跨过腕部确认，实际记录到 `APPROACH → GRASP_POSE → RETREAT → PLACE`。但记录器在动作仍为 `PICK_SENT` 时达到墙钟超时，没有收到终态结果，所以这只能证明流水线已推进到受保护的放置阶段，仍不能宣称一次完整抓放成功。冻结的历史定位核心已恢复到清单记录的27,119字节和原SHA-256；泛化场景的支持度排序与腕部校准放在独立模块中。定向测试78项、完整回归493项全部通过。正式30种子继续封存。

随后已把临时探针升级为带启动、无进展和绝对硬上限的正式开发记录器，并将机械臂 `PLAN/APPROACH/GRASP/RETREAT/PLACE/VERIFY/DONE` 反馈纳入回执。腕部静止确认窗口从1.5秒增加到3秒，但15 mm不确定度、50 mm修正距离、置信度、碰撞和一次重试门槛均未放宽。开发种子44012的新终态回执为 `PARTIAL_SUCCESS`：目标2在首次抓取中因夹爪接触或仿真附着失败，第二次确认超时后被跳过；系统没有终止批次，而是继续自主选择目标7，完整执行抓取、撤离、放置、验证和 `DONE`，最终把目标7记录为 `HARVESTED`。这已经证明一颗感知控制草莓的完整抓放，也证明单目标失败后能继续处理其他目标；尚未证明同一批次成功采摘两颗或采完全部可达成熟果。

开发运行器同时修复了 `setsid` 后过早查询PGID的竞态：现在直接使用子进程PID作为进程组ID，只有连续确认ROS/Gazebo组为空才写入 `cleanup_probe: CLEAN`。独立就绪后清理冒烟以退出码0结束且无残留进程。正式30种子仍继续封存。

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
  --results results/generalized/formal_v1/results.json \
  --output results/generalized/formal_v1/summary.json
```

评测器会同时检查检测召回/精确率、ID切换、定位 P95、标准差、错误采未成熟果、碰撞、负样本安全停止、单目标成功率和完整场景完成率。缺失、重复或额外的场景都会直接失败。

## 尚未宣称的结果

代码、接口、单元测试、场景物化和验收器已经完成，并不等于30种子运行已经通过。广义检测器已经通过独立资格门，但运行时的腕部重定位、运动状态有效性、完整单果抓放和多果批次仍未通过开发门槛，因此30个隐藏种子没有执行正式行为试验。只有这些运行门先通过，并在同一冻结提交上实际完成全部30个场景、生成 `overall_pass: true` 的汇总后，才能声称达到计划中的泛化指标。现有固定场景和历史 P3/P4 结论保持不变。
