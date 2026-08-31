# tego_control

`tego_control` 与 `tego_sdk` 是两个独立 Git 仓库。集成运行时建议把它们放在同一个 ROS 2 工作空间：

```text
<workspace>/
├── src/tego_sdk/       # 独立 SDK 仓库：C++/Python 底层设备通信
└── src/tego_control/    # 本仓库：ROS 2 控制和 Nero 示例适配
```

本仓库包含的主要 ROS 包：

| 包 | 职责 | 当前定位 |
| --- | --- | --- |
| `tego_msgs` | ROS 消息与 service 定义 | 控制接口包，不承载节点 |
| `tego_ros` | SDK ↔ ROS 2 桥接、状态发布、控制转发 | 通用控制层的当前六轴示教器实现 |
| `tego_bringup` | launch、CAN 配置脚本和 udev 规则 | 当前 Nero 示例入口 |
| `agx_arm_ctrl` | Nero/AgileX 驱动、夹爪控制和遥操节点 | 示例机械臂适配层 |
| `agx_arm_msgs` | 示例机械臂、夹爪和灵巧手消息 | 示例接口 |
| `tego_description` | URDF、网格和 RViz 配置 | 示例资源 |

当前示例链路为：

```text
六轴示教器手柄
  -> tego_sdk / tego_ros
  -> /left_handle_pose、/right_handle_pose
  -> agx_arm_ctrl 遥操节点
  -> /control/joint_states_{left,right}_raw
  -> tego_ros（遥操开启时转发）
  -> /control/move_js_{left,right}
  -> Nero 示例机械臂
```

底层 SDK 代码统一使用 `tegocore`、`TegoBase`、`TegoState` 和 `Tego*` API 名称；协议消息和当前六轴示教器硬件行为保持不变。

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10（示例遥操节点需要能导入其 Python 依赖）
- Linux SocketCAN、USB-CAN 适配器和 `gs_usb` 内核模块
- 当前 Nero/AgileX 示例依赖 `pyAgxArm`、Pinocchio、CasADi、NumPy、SciPy 和 MeshCat

安装构建与 CAN 工具：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-colcon-recursive-crawl \
  python3-rosdep python3-catkin-pkg python3-pip \
  python3-numpy python3-scipy python3-tk python3-matplotlib \
  can-utils ethtool iproute2 \
  libboost-all-dev libeigen3-dev liborocos-kdl-dev \
  libnlopt-dev libnlopt-cxx-dev
```

安装当前示例的额外 Python 包（请使用运行遥操节点的同一个 `python3`）：

```bash
python3 -m pip install git+https://github.com/agilexrobotics/pyAgxArm.git
```

如果使用独立 Conda 环境运行 Pinocchio/CasADi 遥操，可按下面方式准备；工作区构建仍建议使用系统 ROS Python：

```bash
conda create -n nero_ik --override-channels -c conda-forge python=3.10 -y
conda activate nero_ik
conda install --override-channels -c conda-forge \
  "pinocchio=3.2.0" "casadi=3.6.7" scipy -y
python -m pip install lark "numpy==2.0.2" "empy==3.3.4" \
  pyyaml opencv-python ur-rtde netifaces meshcat
```

## 构建工作区

将两个仓库放到同一个工作区，例如：

```text
~/agilex_ws/tego/src/tego_sdk
~/agilex_ws/tego/src/tego_control
```

从工作区根目录执行：

```bash
cd ~/agilex_ws/tego
source /opt/ros/humble/setup.bash

rosdep update
rosdep install --from-paths src/tego_control src/tego_sdk --ignore-src -r -y
colcon list --base-paths src/tego_control src/tego_sdk
colcon build \
  --base-paths src/tego_control src/tego_sdk \
  --symlink-install \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DBUILD_TESTING=OFF

source install/setup.bash
```

`colcon list` 应至少能看到 `tego_sdk`、`tego_msgs`、`tego_ros`、`tego_bringup`、`agx_arm_ctrl`、`agx_arm_msgs` 和 `tego_description`。`tego_sdk` 的 C++ package 使用 `ament_cmake`，不是不依赖 ROS 的纯 CMake 工程；如果只构建 SDK，请先 source ROS 环境。

## 配置 CAN

当前双臂示例默认使用四个逻辑接口：

| 逻辑接口 | 用途 |
| --- | --- |
| `left_nero` / `right_nero` | 从臂左 / 右臂 |
| `left_tego` / `right_tego` | 手柄左 / 右侧 |

先查看 USB-CAN 的物理地址：

```bash
cd ~/agilex_ws/tego
bash src/tego_control/tego_bringup/scripts/find_all_can_port.sh
```

然后编辑 `src/tego_control/tego_bringup/scripts/can_config.sh` 中的 USB 地址，并执行：

```bash
bash src/tego_control/tego_bringup/scripts/can_config.sh
ip -details link show type can
```

脚本会把接口配置为 1 Mbps，并按逻辑名重命名。执行这些脚本需要相应的 `sudo` 权限。`tego_bringup/udev/99-tego-can.rules` 是系统 udev 规则文件。

## 启动当前 Nero 双臂示例

### 一体化入口

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tego_bringup demo_nero_bringup.launch.py
```

常用参数：

```bash
ros2 launch tego_bringup demo_nero_bringup.launch.py \
  left_arm_port:=left_nero \
  right_arm_port:=right_nero \
  left_port_name:=left_tego \
  right_port_name:=right_tego \
  log_level:=info
```

如果使用工作区根目录提供的 `start.sh`，它还会自动激活默认的 `nero_ik` Conda 环境；该脚本是工作区辅助脚本，不属于本仓库的 ROS package。可以通过 `CONDA_BASE` 和 `WORKSPACE_DIR` 覆盖其默认路径。

### 分终端启动

需要分别启动机械臂驱动、遥操节点和 `tego_ros` 桥接时：

```bash
# 终端 1：Nero 示例机械臂驱动
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch agx_arm_ctrl start_double_nero.launch.py

# 终端 2：Pinocchio 遥操节点
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch agx_arm_ctrl teleop_double_nero.launch.py

# 终端 3：手柄桥接和控制转发
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tego_bringup double_nero_remote.launch.py
```

首次开启遥操前确认机械臂周围无障碍物。遥操关闭或 CAN 断开时，桥接不会把 raw 关节命令转发给机械臂。

## 运行时Topic接口

以下是当前 Nero 示例的Topic接口：

| 方向 | 左侧 | 右侧 |
| --- | --- | --- |
| 手柄位姿 | `/left_handle_pose` | `/right_handle_pose` |
| IK raw 关节 | `/control/joint_states_left_raw` | `/control/joint_states_right_raw` |
| 最终运动命令 | `/control/move_js_left` | `/control/move_js_right` |
| 夹爪/关节命令 | `/control/joint_states_left` | `/control/joint_states_right` |
| 示例机械臂反馈 | `/feedback/joint_states_left` | `/feedback/joint_states_right` |
| 遥操状态 | `/left_teleop_enable` | `/right_teleop_enable` |
| 遥操开关 service | `/left_teleop_trigger` | `/right_teleop_trigger` |
| 手柄 IMU | `/tego_l/imu` | `/tego_r/imu` |
| 手柄状态 | `/tego_l/handle_state` | `/tego_r/handle_state` |

检查链路：

```bash
ros2 topic list
ros2 topic hz /left_handle_pose
ros2 topic echo /feedback/joint_states_left --once
ros2 service list | grep -E 'teleop_trigger|reset_control'
```

## 排障

- **找不到 CAN 接口**：检查 USB 连接、`lsmod | grep gs_usb`、`ip link show type can`，必要时执行 `sudo modprobe gs_usb`。
- **控制节点连接失败**：确认逻辑接口存在、bitrate 为 `1000000`，并检查对应的 `/tego_l/imu` 或 `/tego_r/imu`。
- **遥操没有动作**：确认 `/left_teleop_enable` 或 `/right_teleop_enable` 已开启，同时检查对应 raw 话题和 `/control/move_js_*`。
- **Python 依赖导入失败**：用启动节点实际使用的 `python3 -c 'import ...'` 检查，不要把依赖安装到另一个 Python 环境。
- **构建结果异常**：确认已经 source `/opt/ros/humble/setup.bash` 和 `install/setup.bash`；必要时在确认不会影响其它任务后清理本地 `build/`、`install/`、`log/` 再重建。
