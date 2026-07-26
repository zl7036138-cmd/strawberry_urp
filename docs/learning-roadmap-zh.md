# 草莓 URP 项目学习路线

这份路线用于后续对话中的系统教学。目标不是只会运行脚本，而是能够解释、
修改和验证“视觉感知—三维定位—运动规划—仿真执行”完整链路。

## 学习顺序

### 1. Linux、WSL、Git 与 Python 工程基础

需要理解：

- Windows 与 WSL 文件路径的对应关系；
- Shell、环境变量、进程和退出码；
- Git 的工作区、提交、分支和远端；
- Python 包、虚拟环境、单元测试和命令行参数。

项目入口：

- `requirements/`
- `scripts/build_and_test.sh`
- 各 ROS 包的 `setup.py`、`package.xml`

建议练习：在不启动仿真的情况下运行依赖较少的单元测试，并读懂一个失败测试。

### 2. ROS 2 通信与包结构

需要理解：

- Node、Topic、Message、Service、Action；
- Publisher、Subscriber、QoS；
- ROS 时间和系统时间；
- launch 文件和参数覆盖；
- colcon 工作区及包依赖关系。

项目入口：

- `ros2_ws/src/strawberry_interfaces/`
- `ros2_ws/src/strawberry_bringup/`
- `docs/architecture.md`

建议练习：使用 `ros2 topic list`、`ros2 topic echo` 和 `ros2 node info`
观察一次不带机械臂动作的相机运行。

### 3. 坐标系、TF 与机器人模型

需要理解：

- 世界坐标系、机器人基座坐标系、相机光学坐标系；
- 齐次变换、四元数和位姿链；
- URDF/Xacro 与关节树；
- 为什么相机安装位置必须同时进入 TF 和碰撞模型。

项目入口：

- `ros2_ws/src/strawberry_sim/urdf/`
- `ros2_ws/src/strawberry_sim/launch/sim.launch.py`
- `docs/wrist-camera-development-v1.md`
- `docs/dual-camera-development-v1.md`

建议练习：手工画出 `panda_link0 → panda_hand → wrist camera optical frame`
的坐标链，再用 TF 工具核对。

### 4. Gazebo 场景、SDF 与物理仿真

需要理解：

- SDF 世界、模型、link、joint、visual 和 collision；
- 网格模型与简化碰撞体的区别；
- 传感器更新率、仿真时间和实时率；
- 草莓刚体、接触检测和附着/分离。

项目入口：

- `ros2_ws/src/strawberry_sim/worlds/strawberry_orchard.sdf`
- `ros2_ws/src/strawberry_sim/models/`
- `ros2_ws/src/strawberry_sim/config/scene.yaml`
- `docs/assets/blender-asset-provenance-v2.md`

建议练习：只启动 Gazebo，识别植株、三颗独立果实、花盆和收集箱分别由哪个
SDF 对象提供。

### 5. RGB-D 相机与三维定位

需要理解：

- RGB 图像、深度图和 CameraInfo；
- 相机内参和针孔投影；
- 从像素及深度恢复相机坐标；
- 从相机坐标通过 TF 转换到 `panda_link0`；
- 时间同步、新鲜度和定位不确定度。

项目入口：

- `ros2_ws/src/strawberry_localization/`
- `docs/architecture.md` 的 Depth localization definition
- `docs/dual-camera-sequential-observation-v1.md`

建议练习：给定像素、深度和内参，手算一次三维点，再与代码输出比较。

### 6. YOLO 检测与成熟度判断

需要理解：

- 目标检测的边界框、类别、置信度；
- `RIPE`、`UNRIPE` 和 `UNKNOWN`；
- 训练集、验证集与封存测试集；
- Precision、Recall、F1 和阈值冻结；
- 为什么工程豁免不等于模型指标通过。

项目入口：

- `ros2_ws/src/strawberry_perception/`
- `tools/perception/`
- `docs/training.md`
- `docs/testing.md`

建议练习：读取一次固定预测结果，手工计算成熟和未成熟类别的混淆矩阵与 F1。

### 7. MoveIt 运动规划与碰撞场景

需要理解：

- 逆运动学、关节空间与笛卡尔空间；
- PlanningScene 和 CollisionObject；
- 起点状态、目标位姿、规划轨迹和终点误差；
- 规划成功与执行成功的区别；
- 为什么当前预抓取 Shadow 生成轨迹后必须丢弃。

项目入口：

- `ros2_ws/src/strawberry_manipulation/strawberry_manipulation/core.py`
- `ros2_ws/src/strawberry_manipulation/strawberry_manipulation/moveit_backend.py`
- `ros2_ws/src/strawberry_manipulation/strawberry_manipulation/pregrasp_shadow.py`
- `docs/architecture.md`

建议练习：比较果实中心、手爪中心、预抓取位姿三者的偏移关系，并核对规划前后
七个碰撞对象是否完整。

### 8. 双相机顺序协作与安全状态机

需要理解：

- 底座相机为什么适合全局观察；
- 夹爪相机为什么适合精确定位；
- 顺序观察与多相机融合的区别；
- 连续帧就绪门、目标身份冻结和失败关闭；
- `pick_authorized=false` 如何贯穿选择、交接和规划阶段。

项目入口：

- `scripts/run_dual_sequential_observation.sh`
- `ros2_ws/src/strawberry_bringup/strawberry_bringup/observation_sequence.py`
- `ros2_ws/src/strawberry_manipulation/strawberry_manipulation/handoff_shadow.py`
- `docs/dual-camera-sequential-observation-v1.md`

建议练习：根据一次运行结果画出从底座观察到轨迹丢弃的状态转换图。

### 9. 测试、实验设计与可复现性

需要理解：

- 单元测试、集成测试、仿真运行和正式实验的边界；
- 新世界重复、非接受性诊断和正式门槛；
- 哈希、不可覆盖输出、日志和失败归因；
- 为什么不能删除失败运行或重新包装历史结果。

项目入口：

- `docs/testing.md`
- `docs/decisions/`
- 各包的 `test/`
- `config/` 中的冻结实验配置

建议练习：选一条失败记录，说明它是基础设施失败、感知失败、定位失败、规划失败
还是执行失败，并指出证据文件。

## 推荐教学节奏

后续可以按九个主题逐章学习。每章建议采用：

1. 概念讲解；
2. 结合本项目画出数据流；
3. 阅读一小段真实代码；
4. 完成一个安全练习；
5. 用测试验证理解；
6. 最后再进入下一章。

当前项目应保持在“安全预抓取规划完成、轨迹不执行”的冻结状态。学习期间优先使用
只读检查、单元测试和无机械臂动作的 Shadow 运行，避免在尚未理解安全边界时开放
感知驱动执行。
