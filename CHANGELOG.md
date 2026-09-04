# CHANGELOG

## [0.1.0] - 2026-09-04

首个面向 Nero 与 Piper 遥操的稳定版本。

- 统一机械臂接口命名为 `left_arm` / `right_arm`，并整理对应 launch 与启动脚本。
- 增加 Piper 单臂 C++ / Python 示例流程，支持左右侧选择。
- 统一 C++ / Python bridge 的 topic、服务和遥操行为。
- 引入基于 机械臂配置文件 的 URDF、IK、回零和安全约束配置，并明确夹爪与额外关节的处理边界。
- 补充 C++ / Python 使用说明和其他机械臂适配指南。
- 新增离线 `arm_safety_monitor` RViz 编辑流程：通过 URDF 和关节 GUI 调整姿态，通过RViz Interactive Markers编辑并保存工作空间 Box 与虚拟墙 Plane。
- 增加 RViz 安全空间交互编辑：工作空间和虚拟墙只显示名称，虚拟墙显示可随墙拖动的法向量箭头；没有任何对象时可通过绿色“安全空间设置”球创建第一个对象，已有对象后可直接使用对应 Marker 菜单；支持右键翻转法向，保存时直接写回配置文件。
