# Changelog

本文件记录 UMA_Control 的重要变更，格式遵循 [Keep a Changelog 1.1.0](https://keepachangelog.com/zh-CN/1.1.0/)。

## [Unreleased]

### Added

- 新增 Piper X 双臂正装启动入口。左右机械臂分别使用 `left_arm`、`right_arm`，示教器分别使用 `left_uma`、`right_uma`，两侧共用一个采集控制节点。
- 新增 Ubuntu 24.04 原生 Jazzy 和 Python 3.12 支持。构建与启动脚本根据系统版本选择 ROS 和 Python，检查工作区平台与 Python 版本，并为 Nero、Piper、Piper X 提供 C++、Python 启动入口。
- 新增 Piper X 单臂遥操支持，包括 profile、URDF、网格文件和 C++、Python 启动脚本。`demo_piper.launch.py` 接受 `arm_type:=piper_x`，自动选择 Piper X 驱动型号和默认 profile。
- 新增全局采集请求控制。双击左手柄 K1 或右手柄 K2 切换同一请求；`recording_control` 发布 `/recording/requested`，提供 `/recording/toggle` 和 `/recording/set_requested`；实际采集由接入的 Web 采集端执行。
- 新增 `robot.firmware_version`，用于向 pyAgxArm 驱动传递机械臂固件协议版本。
- 新增 `build.sh`，提供环境检查、依赖安装和工作空间构建入口。

### Changed

- **不兼容变更**：项目更名为 `UMA_Control`。ROS 包和标识由 `tego_*` 改为 `uma_*`，依赖仓库改为 UMA_SDK；升级时需更新包名、导入路径和启动命令，手柄 CAN 设备默认名称改为 `left_uma`、`right_uma`。
- **不兼容变更**：启动参数 `pika_pose_topic` 改为 `uma_pose_topic`，默认位姿话题由 `/pika_pose` 改为 `/uma_pose`。自定义 launch 文件和话题订阅需同步更新。
- `robot.effector_type` 统一控制末端执行器开关和类型。末端类型、回零姿态和工作空间支持 `left/right/default` 配置，保留共享配置；安全空间编辑器按侧保存工作空间，并保留另一侧配置。
- `installation_pos` 的 `horizontal`、`left`、`right` 统一表示机械臂物理安装方向；`side` 用于左右 CAN、话题、服务和节点路由。Piper 重力补偿使用相同的安装方向配置。
- Nero、Piper、Piper X 的 C++ 和 Python 启动脚本自动激活 Conda 环境，并隔离用户 site-packages。默认环境名称为 `uma`，支持通过 `CONDA_BASE` 和 `CONDA_ENV` 指定安装目录与环境名称。
- 遥操默认关闭 MeshCat 可视化及逐帧显示更新。
- README 将按键操作整理为表格，补充 Anaconda 和自定义 Conda 路径排障、手柄初始化零位与上电操作、Piper X 单臂和双臂启动说明，并新增验证入口；适配指南提供中英文内容。

### Fixed

- 修复正装 Piper、Nero 遥操时末端回零姿态导致平移 x/z 方向互换的问题。
- 修复正装遥操开启时的坐标映射：根据手柄水平前向建立本次遥操的固定映射，平移与旋转共用该映射，并以实时机械臂位姿作为起点。位姿无效、反馈过期或手柄朝向接近竖直时拒绝开启。
- 修复关闭遥操或反馈超时后安装映射未清除的问题，下次开启时重新建立映射。
- 接通 Piper、Nero 夹爪命令，按 `effector_type` 执行。
- 将 `recording_control_node.py` 安装到 `uma_ros` 可执行目录，修复示例启动时找不到脚本的问题。
- 修复 IK 在零旋转等姿态下计算位姿误差导数时产生 NaN 的问题，改用前向自动微分。

## [0.1.0] - 2026-09-04

### Added

- 提供 Nero 双臂和 Piper 单臂遥操示例，以及 C++、Python 启动入口；Piper 单臂支持左右侧选择。
- 引入机械臂配置文件，集中配置 URDF、IK、回零姿态和安全约束。
- 提供离线 `arm_safety_monitor` RViz 安全空间编辑器，可调整机械臂姿态，创建、编辑并保存工作空间 Box 和虚拟墙 Plane；支持显示虚拟墙法向量、翻转法向量，并将修改写回配置文件。
- 提供 C++、Python 使用说明和机械臂适配指南。

### Changed

- 统一 `left_arm`、`right_arm` 接口及对应 launch 和启动脚本，统一 C++、Python 桥接节点的话题、服务和遥操行为。

[Unreleased]: https://github.com/agilexrobotics/UMA_Control/compare/27a49e69ef43c0b80c51bc1aafda3ef2a1e543fb...HEAD
[0.1.0]: https://github.com/agilexrobotics/UMA_Control/tree/27a49e69ef43c0b80c51bc1aafda3ef2a1e543fb
