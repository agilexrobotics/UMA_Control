# CHANGELOG

## [1.0.0] - 2026-09-01

首个面向 Nero 与 Piper 遥操的稳定版本。

- 统一机械臂接口命名为 `left_arm` / `right_arm`，并整理对应 launch 与启动脚本。
- 增加 Piper 单臂 C++ / Python 示例流程，支持左右侧选择。
- 统一 C++ / Python bridge 的 topic、服务和遥操行为。
- 引入基于 机械臂配置文件 的 URDF、IK、回零和安全约束配置，并明确夹爪与额外关节的处理边界。
- 补充 C++ / Python 使用说明和其他机械臂适配指南。
