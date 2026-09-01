# tego_control

## 概述

`tego_control` 是基于 ROS 2 的示教器主从遥操集成包，使用底层的 [tego_sdk](https://gitlab.agilex.ai/sjq01/tego_sdk) 进行 CAN 通信和协议编解码，并在 `tego_ros` 节点中把手柄状态、关节命令、IMU 与机械臂驱动桥接起来。机械臂运动学、关节映射、回零目标和安全约束由 `RobotProfile` 统一配置，当前包含 Nero 与 Piper 示例 profile。

---

## 仓库结构

本仓库与 `tego_sdk` 是两个独立 Git 仓库。集成运行时建议把它们放在同一个 ROS 2 工作空间：

```text
<workspace>/
├── src/tego_sdk/        # 六轴通用示教器 SDK 仓库：C++/Python 通信
└── src/tego_control/    # 本仓库：ROS 2 控制和 Nero/Piper 示例
```

本仓库包含的主要 ROS 包：

|包|职责|
|---|---|
|`tego_msgs`|ROS 消息与 service 定义（手柄/传感器状态、编码器控制）|
|`tego_ros`|SDK ↔ ROS 2 桥接、状态发布、控制转发；提供手柄与机械臂命令桥接|
|`tego_bringup`|launch、CAN 配置脚本、Nero/Piper 示例入口、机械臂 profile 配置|
|`tego_description`|URDF、网格、RViz 配置|
|`tego_kinematics/agx_arm_ctrl`|Nero/AgileX 示例机械臂驱动、Pinocchio 遥操节点、夹爪/灵巧手控制|
|`tego_kinematics/agx_arm_msgs`|示例机械臂、夹爪和灵巧手消息定义|

当前 Nero 双臂示例链路：

```text
六轴示教器手柄 (left_tego / right_tego)
  -> tego_ros
  -> /left_handle_pose、/right_handle_pose
  -> agx_arm_ctrl Pinocchio 遥操节点
  -> /control/joint_states_{left,right}_raw
  -> tego_ros（遥操开启时转发）
  -> /control/move_js_{left,right}
  -> Nero 示例机械臂 (left_arm / right_arm)
```

---

## 快速开始

### 1. 创建工作空间并克隆

```bash
mkdir -p ~/tego/src
cd ~/tego/src
git clone https://gitlab.agilex.ai/sjq01/tego_control.git
git clone https://gitlab.agilex.ai/sjq01/tego_sdk.git
```

两个仓库需要位于同一个 ROS 2 工作空间中：

```text
~/tego/
├── src/tego_sdk/        # C++ / Python 示教器 SDK
└── src/tego_control/    # ROS 2 控制层和示例
```

### 2. 公共系统依赖

C++ 和 Python 流程都需要 ROS 2 Humble、`colcon`、SocketCAN、Conda，以及 Pinocchio/CasADi 遥操环境。先安装系统依赖：

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

两种 bridge 都会启动 `agx_arm_ctrl` 的 Python IK 节点，因此都需要 `nero_ik` 环境：

```bash
conda create -n nero_ik --override-channels -c conda-forge python=3.10 -y
conda activate nero_ik
conda install --override-channels -c conda-forge \
  "pinocchio=3.2.0" "casadi=3.6.7" scipy -y
python -m pip install lark "numpy==2.0.2" "empy==3.3.4" \
  pyyaml opencv-python ur-rtde netifaces meshcat
```

如果使用仓库内的 Nero/Piper 示例驱动，还需要安装其驱动依赖：

```bash
python3 -m pip install git+https://github.com/agilexrobotics/pyAgxArm.git
```

> `nero_ik` 只负责 IK 和安全约束，不代表机械臂驱动已经连接。真实机械臂是否能使能、回零和执行命令，取决于驱动层。

### 3. 依赖与构建

#### 3.1 C++

C++ 用户使用 `tego_ros_node` 作为手柄 bridge。它链接 `tego_sdk/c++`，**不需要**安装 Python `tego_sdk` 或 `python-can`。但仍然需要上一节的 `nero_ik` 环境，因为 IK 节点是 Python 实现。

在工作空间根目录构建 C++ 链路：

```bash
cd ~/tego
source /opt/ros/humble/setup.bash

rosdep update
rosdep install --from-paths src/tego_control src/tego_sdk --ignore-src -r -y

colcon build \
  --base-paths src/tego_control src/tego_sdk \
  --symlink-install \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DBUILD_TESTING=OFF

source install/setup.bash
```

这一步会构建 `tego_sdk/c++`、`tego_ros` 以及控制层所需的 ROS 包。可以用下面的命令确认 C++ SDK 和 ROS 包已被发现：

```bash
colcon list | grep -E 'tego_sdk|tego_ros|tego_bringup|agx_arm_ctrl'
ros2 pkg prefix tego_ros
```

#### 3.2. Python

Python 用户使用 `tego_ros_py_node.py` 作为手柄 bridge。Python bridge 需要额外安装 `python-can` 和 `tego_sdk/python`：

```bash
conda activate nero_ik
python -m pip install python-can
python -m pip install -e ~/tego/src/tego_sdk/python
python -c "import can, tego_sdk; print('Python SDK dependencies are ready')"
```

Python 用户仍然需要构建 ROS 工作空间：`tego_ros` 的 ROS 包、launch、消息和控制节点都必须先安装；工作空间构建也会同时构建 `tego_sdk/c++`，这是 `tego_ros` 的 CMake 依赖。构建命令与 C++ 用户相同：

```bash
cd ~/tego
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src/tego_control src/tego_sdk --ignore-src -r -y
colcon build \
  --base-paths src/tego_control src/tego_sdk \
  --symlink-install \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DBUILD_TESTING=OFF
source install/setup.bash
```

`start_*_python.sh` 会自动激活 `nero_ik`，并把 `src/tego_sdk/python` 加入 `PYTHONPATH` 作为源码导入兜底；手动启动 Python launch 时，请确保当前终端已激活 `nero_ik` 且能导入 `can`、`tego_sdk`。

---

## 使用说明

### 1. 激活 CAN 模块（C++ / Python 共用）

启动任何 bridge 前，先把 USB-CAN 配置为项目约定的逻辑接口，并设置为 **1 Mbps**。脚本位于 `tego_bringup/scripts/`：

```bash
cd ~/tego
bash src/tego_control/tego_bringup/scripts/find_all_can_port.sh
# 根据输出的 USB bus-info，修改 can_config.sh 中的 USB 地址
bash src/tego_control/tego_bringup/scripts/can_config.sh
ip -details link show type can
```

`can_config.sh` 会创建以下逻辑接口；执行配置脚本需要 `sudo` 权限：

|逻辑接口|用途|
|---|---|
|`left_tego` / `right_tego`|左 / 右主端手柄|
|`left_arm` / `right_arm`|左 / 右从臂驱动|

可选地，将 `tego_bringup/udev/99-tego-can.rules` 安装到 `/etc/udev/rules.d/`，使 USB-CAN 的 bus-info 在热插拔后保持稳定。

### 2. C++ 流程

C++ bridge 节点为 `tego_ros_node`。C++ 和 Python bridge **只能二选一**，不要在同一组手柄/CAN 接口上同时启动。

#### Nero 双臂

推荐使用已经移动到 `tego_bringup/scripts/` 的启动脚本：

```bash
cd ~/tego
bash src/tego_control/tego_bringup/scripts/start_dual_nero_cpp.sh
```

脚本默认启动 `demo_nero_bringup.launch.py`，对应链路为：

```text
left_tego / right_tego -> tego_ros_node -> Nero 双臂控制层
left_arm  / right_arm  -> Nero 用户驱动或示例驱动
```

如果需要覆盖 launch 参数，可以直接追加 ROS launch 参数：

```bash
bash src/tego_control/tego_bringup/scripts/start_dual_nero_cpp.sh \
  left_arm_port:=left_arm right_arm_port:=right_arm
```

#### Piper 单臂

Piper C++ 脚本通过 `SIDE` 选择左右侧，默认是右臂：

```bash
cd ~/tego
# 默认右臂
bash src/tego_control/tego_bringup/scripts/start_single_piper_cpp.sh

# 左臂
SIDE=left bash src/tego_control/tego_bringup/scripts/start_single_piper_cpp.sh
```

不使用脚本时，请先 source ROS 2 和工作空间，再直接启动对应入口：

```bash
source /opt/ros/humble/setup.bash
source ~/tego/install/setup.bash

# 右臂(默认)
ros2 launch tego_bringup demo_piper.launch.py
# 左臂
ros2 launch tego_bringup demo_piper.launch.py side:=left
```

使用自己的机械臂驱动时，关闭仓库内示例驱动：

```bash
ros2 launch tego_bringup demo_piper.launch.py \
  start_arm:=false \
  arm_port:=left_arm
```

#### C++ 流程的分层调试

需要拆分节点调试时，可以分别启动示例驱动、IK 和 C++ bridge：

```bash
source /opt/ros/humble/setup.bash
source ~/tego/install/setup.bash

ros2 launch agx_arm_ctrl start_double_arm.launch.py
conda activate nero_ik
ros2 launch agx_arm_ctrl teleop_double_arm.launch.py \
  profile_path:=$(ros2 pkg prefix tego_bringup)/share/tego_bringup/config/nero_profile.yaml
ros2 launch tego_bringup demo_nero_bringup.launch.py start_arm:=false start_teleop:=false
```

### 3. Python 流程

Python bridge 节点为 `tego_ros_py_node.py`。启动前确认：

```bash
conda activate nero_ik
python -c "import can, tego_sdk; print('Python SDK dependencies are ready')"
```

#### Nero 双臂

```bash
cd ~/tego
bash src/tego_control/tego_bringup/scripts/start_dual_nero_python.sh
```

脚本默认以 Python 后端启动 `demo_nero_bringup.launch.py`。如果直接使用 launch，请先 source 工作空间并激活环境：

```bash
source /opt/ros/humble/setup.bash
source ~/tego/install/setup.bash
conda activate nero_ik
ros2 launch tego_bringup demo_nero_bringup.launch.py backend:=python
```

#### Piper 单臂

Python Piper 脚本同样通过 `SIDE` 选择左右侧，默认是右臂：

```bash
cd ~/tego
# 默认右臂
bash src/tego_control/tego_bringup/scripts/start_single_piper_python.sh

# 左臂
SIDE=left bash src/tego_control/tego_bringup/scripts/start_single_piper_python.sh
```

也可以直接启动：

```bash
source /opt/ros/humble/setup.bash
source ~/tego/install/setup.bash
conda activate nero_ik
ros2 launch tego_bringup demo_piper.launch.py backend:=python             # 右臂
ros2 launch tego_bringup demo_piper.launch.py side:=left backend:=python  # 左臂
```

使用自有 Piper 驱动时：

```bash
ros2 launch tego_bringup demo_piper.launch.py \
  start_arm:=false \
  arm_port:=left_arm
```

> `demo_piper.launch.py` 默认启动仓库内的 Piper 示例驱动；`start_arm:=false` 只关闭该示例驱动，不会替用户启动或适配厂商驱动。用户驱动仍需发布 `/feedback/joint_states_left`（或 right），并订阅 `/control/move_js_left`（或 right）。

### 4. 启动脚本与调试约定

当前启动脚本全部位于 `tego_bringup/scripts/`：

```text
start_dual_nero_cpp.sh
start_dual_nero_python.sh
start_single_piper_cpp.sh
start_single_piper_python.sh
```

四个脚本都会自动 source ROS 2 和工作空间、激活 `nero_ik`，并在退出时清理本次启动的控制进程。Piper 脚本支持 `SIDE=left/right`；Python 脚本还支持 `LAUNCH_FILE` 覆盖默认入口，所有脚本支持 `WORKSPACE_DIR` 和 `CONDA_BASE`：

```bash
WORKSPACE_DIR=~/tego CONDA_BASE=~/miniconda3 \
  SIDE=left bash ~/tego/src/tego_control/tego_bringup/scripts/start_single_piper_python.sh
```

停止流程使用 `Ctrl+C`。不要同时运行 C++ 和 Python bridge，也不要让两个启动脚本争用同一组 CAN 接口。

### 5. 控制层与底层驱动边界

本仓库不实现各品牌机械臂的 SDK、CAN、串口或 EtherCAT 适配。接入其他机械臂时，驱动层由用户自行开发；本仓库只约定 ROS topic、消息、profile 和 launch 接口。用户驱动至少需要：

```text
发布  /feedback/joint_states_left（或 right）
订阅  /control/move_js_left（或 right）
```

关节反馈和命令使用 `sensor_msgs/msg/JointState`，arm 关节单位为 rad。厂商 SDK 初始化、关节名/方向/零位转换、使能、夹爪和额外关节处理都属于用户驱动层。接入其他机械臂前请阅读：[机械臂适配指南](机械臂适配指南.md)。

### 6. 遥操手柄操作说明

手柄内侧按钮（例子：左手柄即为右侧按钮，右手柄即为左侧按钮）交互逻辑：

|动作|功能描述|
|:---|:---|
|**双击**|**开启/关闭遥操，默认关闭**|
|**长按**|**非遥操模式下，初始化示教器和机械臂回零**|

### 7. 机械臂参数

机械臂配置文件位于 `tego_bringup/config/`，当前预设**侧装 Nero 双臂**和**正装 Piper 单臂**两份配置：

|文件|用途|
|---|---|
|`nero_profile.yaml`|侧装 Nero 双臂|
|`piper_profile.yaml`|正装 Piper 单臂|

`robot.arm_home_state` 是回零命令的目标值列表，不是 URDF 自由度定义。列表长度决定回零命令包含的关节/末端通道数量，必须与驱动反馈或命令 `JointState.position` 的长度一致；列表元素顺序也必须与驱动 `JointState.position` 的顺序一致。它不会自动增加或减少 URDF 中的 arm 关节。

单臂配置示例：

```yaml
robot:
  arm_home_state:
    # 单臂：default 对应当前单臂的全部回零通道
    default: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

双臂配置示例：

```yaml
robot:
  arm_home_state:
    # 双臂：left/right 分别对应一侧，不能填写两侧通道数之和
    left: [0.1, 1.5708, -1.5708, 1.6707, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
    right: [-0.1, 1.5708, 1.5708, 1.6707, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
```

例如 Piper 当前 `default` 长度为 6；Nero 双臂每个 `left/right` 列表长度为 10，其中前 7 个是 arm 关节，后 3 个是夹爪/末端通道。具体通道数量和顺序以你的驱动实际发布/接收的 `JointState` 为准。

### 8. 启动参数

#### `demo_nero_bringup.launch.py`（C++ bridge）

|参数|类型|默认值|说明|
|---|---|---|---|
|`left_arm_port`|string|`left_arm`|左从臂 SocketCAN 接口名|
|`right_arm_port`|string|`right_arm`|右从臂 SocketCAN 接口名|
|`left_port_name`|string|`left_tego`|左手柄 SocketCAN 接口名|
|`right_port_name`|string|`right_tego`|右手柄 SocketCAN 接口名|
|`gripper_width_min`|float|`0.00`|夹爪宽度下限 (m)|
|`gripper_width_max`|float|`0.07`|夹爪宽度上限 (m)|
|`pub_rate`|float|`200.0`|桥接节点发布频率 (Hz)|
|`log_level`|string|`info`|日志级别|
|`start_arm`|bool|`true`|是否启动机械臂驱动|
|`start_teleop`|bool|`true`|是否启动 Pinocchio 遥操节点|

#### `demo_nero_bringup.launch.py`（C++/Python bridge 二合一）

`backend:=cpp/python` 选择桥接实现（默认 cpp）；其余参数与 C++ 入口一致，默认 CAN 接口为 `left_tego/right_tego` 和 `left_arm/right_arm`。

#### `demo_piper.launch.py`（Piper 单臂）

`side:=left/right`（默认 right）、`backend:=cpp/python`（默认 cpp）；CAN 接口用 `arm_port`（默认 `right_arm`）、`port_name`（默认 `right_tego`）。支持：

|参数|类型|默认值|说明|
|---|---|---|---|
|`profile_path`|string|`piper_profile.yaml`|Piper 的 URDF/IK/安全约束 profile|
|`start_arm`|bool|`true`|是否启动仓库内 Piper 示例驱动；使用自有驱动时设为 `false`|
|`start_teleop`|bool|`true`|是否启动 Pinocchio 遥操节点|
|`gripper_width_min` / `gripper_width_max`|float|`0.00` / `0.07`|手柄夹爪宽度映射范围 (m)，实际执行由驱动负责|

> 低层 `teleop_double_arm.launch.py`、`run_single_node.launch.py` 适合拆分调试和二次集成；日常使用优先选择上面的 `demo_*` 入口。自定义机械臂驱动不要求修改 `agx_arm_ctrl`，只需按[机械臂适配指南](机械臂适配指南.md)提供接口。

#### `teleop_double_arm.launch.py`（低层遥操节点）

|参数|类型|默认值|说明|
|---|---|---|---|
|`profile_path`|string|无（必填）|机器人 profile YAML 路径；独立启动时必须显式传入|

> 高层 `demo_nero_bringup.launch.py`、`left_arm.launch.py`、`right_arm.launch.py` 会自动选择 `nero_profile.yaml`。低层 launch 不默认查找 `tego_bringup`，因此不会形成反向包依赖。

#### `run_single_node.launch.py`（单手柄节点高级参数）

|参数|类型|默认值|说明|
|---|---|---|---|
|`port_name`|string|`can0`|SocketCAN 接口名|
|`node_name`|string|`tego_ros_node`|节点名（影响 topic 前缀，例如 `tego_l` / `tego_r`）|
|`pika_pose_topic`|string|`/pika_pose`|主端位姿输出 topic（会被重映射到 `/left_handle_pose` 等）|
|`remote_joint_topic_in`|string|`/joint_states_raw`|遥操 IK 输入 topic（Nero 链路里是 `/control/joint_states_{left,right}_raw`）|
|`remote_joint_topic_out`|string|`/joint_states`|最终关节命令输出 topic（Nero 链路里是 `/control/move_js_{left,right}`）|
|`remote_joint_single_topic`|string|`joint_states_single`|机械臂单臂反馈 topic（remap 到 `/feedback/joint_states_{left,right}`）|
|`teleop_enable_topic`|string|`teleop_enable`|遥操开关 topic（remap 到 `/left_teleop_enable` 等）|
|`remote_service_name`|string|`teleop_trigger`|遥操触发 service（remap 到 `left_teleop_trigger` 等）|
|`arm_home_state`|string|由 robot profile 提供|非遥操状态下回零目标值列表 (rad)；元素数量必须与桥接节点反馈/命令 `JointState.position` 的通道数量一致，顺序也必须一致。它不会自动定义 URDF 的 arm DOF；双臂时 `left`/`right` 分别对应一侧。|
|`imu_zero_rpy`|string|`0, 0, 0`|手柄 IMU 零位 RPY (deg)|
|`base_to_imu_roll` / `_pitch` / `_yaw`|float|`93.0` / `5.0` / `-6.0`|手柄基座到 IMU 的 RPY 偏移 (deg)|
|`gripper_length`|float|`0.14`|末端执行器长度 (m)，用于位姿补偿|
|`zoom_factor`|float|`2.0`|位姿缩放系数|
|`angle_value`|float|`0.35`|手柄偏角阈值 (rad)|
|`is_nero`|bool|`false`|是否使用 Nero 示例机械臂|
|`nero_gripper_topic_name`|string|`gripper_states`|Nero 夹爪控制 topic|
|`check_get`|bool|`true`|启动时等待拿到有效状态|
|`wait_feedback`|bool|`true`|下发命令前等待机械臂反馈|
|`publish_sim_joint`|bool|`true`|是否发布 `/joint_states_single` 模拟关节状态|
|`use_piper_mode`|bool|`false`|使用 Piper 模式（影响 seg 缩放语义）|
|`handle_seg0`|double[]|`[0, 0, 0.123, 0, 0, 0]`|手柄 joint_base→joint0 变换 [x,y,z,r,p,y]，**出厂标定，默认无需修改**|
|`handle_seg1`|double[]|`[0, 0, 0, 1.5708, -0.1359, 3.1416]`|手柄 joint0→joint1 变换，**出厂标定，默认无需修改**|
|`handle_seg2`|double[]|`[0.28503, 0, 0, 0, 0, -1.7939]`|手柄 joint1→joint2 变换，**出厂标定，默认无需修改**|
|`handle_seg3`|double[]|`[-0.022, -0.25075, 0, 1.5708, 0, 0]`|手柄 joint2→joint3 变换，**出厂标定，默认无需修改**|
|`use_pose_dt_zoom`|bool|`false`|基于 dt 的位姿缩放（避免抖动）|
|`use_trigger_pose`|bool|`false`|扳机按下后才发送位姿|
|`angle_reverse` / `base_reverse` / `imu_reverse`|bool|`false` / `false` / `false`|关节/基座/IMU 反转开关|
|`show_imu_data`|bool|`false`|日志里打印 IMU 调试数据|

> 所有参数都可以在 launch 命令行用 `key:=value` 覆盖，例如 `ros2 launch tego_bringup demo_nero_bringup.launch.py left_port_name:=can_l`。

---

## ROS 2 话题与服务

### 手柄节点（命名空间前缀 `tego_l` / `tego_r`）

#### `/tego_l/imu` & `/tego_r/imu`

消息类型：`sensor_msgs/Imu`

|字段|类型|说明|
|---|---|---|
|`orientation`|`geometry_msgs/Quaternion`|手柄姿态四元数（经基座偏移和 IMU 零位校正）|
|`angular_velocity`|`geometry_msgs/Vector3`|角速度 (rad/s)|
|`linear_acceleration`|`geometry_msgs/Vector3`|线加速度 (m/s²)|

#### `/tego_l/handle_state` & `/tego_r/handle_state`

消息类型：`tego_msgs/HandleState`

|字段|类型|说明|
|---|---|---|
|`joystick_x` / `joystick_y`|`int32`|摇杆 X / Y 原始值|
|`trigger_press_percentage`|`int32`|扳机按压百分比 (0~100)|
|`key1_click_type` / `key2_click_type`|`uint8`|按键 1 / 2 点击类型（单击 / 双击 / 长按）|
|`handle_receive_count`|`uint32`|手柄报文接收计数|
|`key1_receive_count` / `key2_receive_count`|`uint32`|按键 1 / 2 接收计数|

#### `/tego_l/mt_sensor_status` & `/tego_r/mt_sensor_status`

消息类型：`tego_msgs/MtSensorStatus`

|字段|类型|说明|
|---|---|---|
|`name`|`string[]`|编码器/传感器名称列表|
|`data`|`SensorData[]`|与 `name` 对应的传感器数据|
|`receive_count`|`uint32[]`|与 `name` 对应的接收计数|

其中 `SensorData`：

|字段|类型|说明|
|---|---|---|
|`angle_deg`|`float64`|传感器角度 (deg)|
|`angle_rad`|`float64`|传感器角度 (rad)|
|`voltage_x` / `voltage_y`|`float64`|传感器供电/原始电压|

#### `/tego_l/joint` & `/tego_r/joint`

消息类型：`sensor_msgs/JointState`，由 SDK 解码后的手柄侧关节状态。

### 主从位姿

#### `/left_handle_pose` & `/right_handle_pose`

消息类型：`geometry_msgs/PoseStamped`

|字段|类型|说明|
|---|---|---|
|`pose.position`|`geometry_msgs/Point`|主端手柄位置 (m)|
|`pose.orientation`|`geometry_msgs/Quaternion`|主端手柄姿态（四元数）|

`tego_ros` 把内部 `/pika_pose` 重映射到这两个 topic。

### 关节命令链路

#### `/control/joint_states_left_raw` & `/control/joint_states_right_raw`

消息类型：`sensor_msgs/JointState`

遥操节点（`agx_arm_ctrl/teleop_single_arm_node.py`）发布；`tego_ros` 订阅，作为未滤波的关节目标。

#### `/control/move_js_left` & `/control/move_js_right`

消息类型：`sensor_msgs/JointState`

`tego_ros` 在遥操开启时把 `/control/joint_states_*_raw` 转发到这个 topic，机械臂驱动订阅。

> 该链路同样适用于 `agx_arm_ctrl` 暴露的其他控制 topic，例如 `/control/move_j{_left/_right}`、`/control/move_p{_left/_right}`、`/control/move_mit{_left/_right}`、`/control/gripper{_left/_right}` 等，详细字段见 [agx_arm_ros README](https://github.com/agilexrobotics/agx_arm_ros)。

### 机械臂反馈

#### `/feedback/joint_states_left` & `/feedback/joint_states_right`

消息类型：`sensor_msgs/JointState`，由仓库示例驱动或用户自己的机械臂驱动发布。

遥操开启前必须先收到实时反馈；遥操首次建立参考位姿时，通过这份反馈计算当前末端 FK，并以当前机械臂姿态作为遥操起点。反馈超过超时时间后，遥操会自动停止，避免继续使用过期关节状态。

`/feedback/tcp_pose{_left/_right}`、`/feedback/arm_status{_left/_right}`、`/feedback/arm_ctrl_states{_left/_right}`、`/feedback/gripper_status{_left/_right}`、`/feedback/hand_status{_left/_right}` 是仓库示例驱动提供的扩展反馈 topic；用户驱动可按自身能力提供，不是控制层接入其他机械臂的最小要求。

### 遥操控制

#### `/left_teleop_enable` & `/right_teleop_enable`

消息类型：`std_msgs/Bool`

遥操开关状态；`tego_ros` 订阅。`true` 时把 `/control/joint_states_*_raw` 转发到 `/control/move_js_*`，`false` 时桥接不下发命令。

#### `/left_teleop_trigger` & `/right_teleop_trigger`

消息类型：`std_srvs/Trigger`

服务：触发开启/关闭遥操。`tego_ros` 通过此服务请求 `agx_arm_ctrl` 切换遥操状态。

### 控制类服务

|服务|类型|说明|
|---|---|---|
|`/reset_pose`|`std_srvs/Empty`|请求重置手柄位姿|
|`/reset_control`|`tego_msgs/ResetControl`|请求手柄控制复位（含 `RESET_ZERO` / `RESET_FACTORY`）|
|`/enable_handle_motor`|`std_srvs/SetBool`|启/停手柄电机|
|`/enable_reverse`|`std_srvs/SetBool`|开关 IMU 方向反转|

`tego_msgs/ResetControl` 字段：

|字段|类型|说明|
|---|---|---|
|`reset_mode`|`uint8`|`0` = `RESET_ZERO`，`1` = `RESET_FACTORY`|

### 检查链路

```bash
ros2 topic list
ros2 topic hz /left_handle_pose
ros2 topic echo /feedback/joint_states_left --once
ros2 service list | grep -E 'teleop_trigger|reset_control'
```

---

## 参数限制

### 夹爪 (Gripper)

|参数|范围|单位|默认|说明|
|---|---|---|---|---|
|`gripper_width_min`|`[0.0, 0.10]`|m|`0.00`|夹爪宽度下限，传给 `tego_ros` 的限位参数|
|`gripper_width_max`|`[0.0, 0.10]`|m|`0.07`|夹爪宽度上限|

> ⚠️ `gripper_width_max` 必须 ≥ `gripper_width_min`；超出该范围的位姿会被钳制。

### 位姿缩放

|参数|范围|单位|默认|说明|
|---|---|---|---|---|
|`zoom_factor`|`[0.1, 5.0]`|无量纲|`2.0`|主从位姿缩放系数，值越大从臂动作幅度越大|
|`gripper_length`|`[0.0, 0.30]`|m|`0.14`|末端执行器长度，用于对齐主从位姿的法兰→TCP 偏移|

> ⚠️ `zoom_factor` 过大（≥ 3.0）时，靠近机械臂奇异点会出现关节跳变，请谨慎调高。

### 手柄零位

|参数|范围|单位|默认|说明|
|---|---|---|---|---|
|`base_to_imu_roll` / `_pitch` / `_yaw`|任意|deg|`93.0` / `5.0` / `-6.0`|基座到 IMU 的安装偏移；与实际硬件一致才能保证姿态零位正确|
|`imu_zero_rpy`|任意|deg|`0, 0, 0`|手柄启动时的 IMU 零位偏置|

---

## 注意事项

### CAN 通信

- 启动前**必须**先激活 CAN 模块
- 波特率：**1000000 bps**
- 若出现 `SendCanMessage failed` 或 `tego_ros` 状态长时间不更新，请检查：
  - `lsmod | grep gs_usb`（需要加载 `gs_usb` 内核模块）
  - `ip -details link show type can` 的 bitrate 和状态
  - 逻辑接口名是否与 launch 参数一致（`left_tego` / `right_tego` / `left_arm` / `right_arm`）

### ⚠️ 安全警告

- **保持安全距离**：机械臂运动时，请勿进入其工作空间
- **首次开启遥操前**：确认机械臂周围无障碍物，关节处于安全零位附近
- **遥操关闭时**：`tego_ros` **不会**把 raw 关节命令转发给机械臂；CAN 断开时同理
- **奇异点风险**：靠近运动学奇异点时，关节可能发生突然大幅运动
- **位姿缩放**：调高 `zoom_factor` 后运动更剧烈，请逐步调高并保持低速测试

### 排障

- **找不到 CAN 接口**：检查 USB 连接、`lsmod | grep gs_usb`、`ip link show type can`；必要时 `sudo modprobe gs_usb`
- **`tego_ros` 连接失败**：确认对应逻辑接口存在、bitrate 为 `1000000`，并检查 `/tego_l/imu` 或 `/tego_r/imu` 是否有数据
- **遥操没有动作**：确认 `/left_teleop_enable` 或 `/right_teleop_enable` 已开启；检查对应 raw 话题 `/control/joint_states_*_raw` 和 `/control/move_js_*` 是否有数据
- **CasADi/IPOPT 报 `Infeasible_Problem_Detected`**：例如日志中出现 `Solver failed`、`return_status is 'Infeasible_Problem_Detected'`。这表示当前 IK 优化在当前关节状态、目标位姿和安全约束组合下没有找到可接受解，**不一定是 CAN 或通信故障**。按以下顺序排查：
  1. 确认 `profile_path` 指向实际使用的 profile，并检查 profile 的 `urdf_path`、`base_frame`、`ee_frame` / `ee_offset` 以及 `safety_frames` 是否与 URDF 一致。
  2. 检查关节上下限，以及 `ik.enable_hard_delta_q_constraint` 和 `ik.max_delta_q_rad`；单帧目标变化过大或最大关节增量过小，都可能使可行域为空。
  3. 可能是工作空间过小，检查 `safety.workspace` 的 `min`、`max`、`margin` 和 `protected_frames`，以及 `safety.virtual_walls` 的方向、`keep_side`、`margin` 和保护点；重点关注日志中的 `wall_dist`。
  4. 从当前位置做小幅度手柄动作，适当降低 `zoom_factor`；目标距离过远、接近奇异姿态或姿态变化过大时，可能导致求解器无法在实时迭代次数内收敛。
  5. 让机械臂回到安全姿态后重试，并检查日志中是否同时出现 `wall_dist`、workspace、frame、collision 或其他约束相关信息。`collision_pairs` 当前用于求解后的自碰撞检测，并不是 Opti 中的硬约束；如果出现自碰撞告警，应单独检查碰撞几何和姿态。
- **Python 依赖导入失败**：用启动节点实际使用的 `python3 -c 'import ...'` 检查，不要把依赖装到另一个 Python 环境
- **构建结果异常**：确认已经 source `/opt/ros/humble/setup.bash` 和 `install/setup.bash`；必要时清理本地 `build/`、`install/`、`log/` 再重建
