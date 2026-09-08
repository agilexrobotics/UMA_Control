# CHANGELOG

## [0.1.0] - 2026-09-04

首个面向 Nero 与 Piper 遥操的稳定版本。

- 将 `horizontal`、`left`、`right` 定义为所有机械臂通用的物理安装方向；
  `side` 继续只负责左右 CAN、topic、service 和节点路由。
- 正装 Piper/Nero 可直接使用 `installation_pos: horizontal`，避免末端 home 姿态导致平移 x/z 互换；Piper 同时使用该值设置重力补偿。
- 反馈超时或关闭遥操时同步清除本次安装映射，下一次遥操会重新建立会话映射。
- Nero/Piper 的 C++ 和 Python 启动脚本会自动激活 `tego` Conda 环境，并隔离用户
  site-packages，避免加载错误版本的 Python 依赖。
- 统一 `left_arm` / `right_arm` 接口、launch、启动脚本和 C++/Python bridge 行为。
- 增加 Piper 单臂和 Nero 双臂示例，以及 profile 驱动的 URDF、IK、回零和安全约束。
- 提供机械臂适配指南和离线 RViz 安全空间编辑器，可编辑并保存 workspace 与virtual walls。
