# tego_control

## 概述

`tego_control` 是基于 ROS 2 的六轴示教器主从遥操集成包，使用底层的 [tego_sdk](https://gitlab.agilex.ai/sjq01/tego_sdk) 进行 CAN 通信和协议编解码，并在 `tego_ros` 节点中把手柄状态、关节命令、IMU 与机械臂驱动桥接起来。

---

## 仓库结构

本仓库与 `tego_sdk` 是两个独立 Git 仓库。集成运行时建议把它们放在同一个 ROS 2 工作空间：

```text
<workspace>/
├── src/tego_sdk/       # 独立 SDK 仓库：C++/Python 底层设备通信
└── src/tego_control/    # 本仓库：ROS 2 控制和 Nero 示例适配
```

本仓库包含的主要 ROS 包：

|包|职责|
|---|---|
|`tego_msgs`|ROS 消息与 service 定义（手柄/传感器状态、编码器控制）|
|`tego_ros`|SDK ↔ ROS 2 桥接、状态发布、控制转发；当前六轴示教器实现|
|`tego_bringup`|launch、CAN 配置脚本、Nero 示例入口、机械臂参数配置|
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
  -> Nero 示例机械臂 (left_nero / right_nero)
```

---

## 快速开始

### 1. 安装底层 SDK

```bash
git clone https://gitlab.agilex.ai/sjq01/tego_sdk.git
```

按 [tego_sdk README](https://gitlab.agilex.ai/sjq01/tego_sdk) 完成 colcon 构建（SDK 自身用 `ament_cmake`，需要先 source ROS 环境）。

### 2. 创建工作空间并克隆

```bash
mkdir -p ~/agilex_ws/tego/src
cd ~/agilex_ws/tego/src
git clone https://gitlab.agilex.ai/sjq01/tego_control.git
# 把 SDK 仓库也放到 src/ 下
git clone https://gitlab.agilex.ai/sjq01/tego_sdk.git
```

### 3. 安装依赖

apt 依赖：

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

Python 依赖（启动遥操节点实际使用的 `python3`）：

```bash
python3 -m pip install git+https://github.com/agilexrobotics/pyAgxArm.git
```

Pinocchio/CasADi 遥操环境（如果只用 `tego_ros` 桥接可跳过；使用 `agx_arm_ctrl/teleop_*_nero` 时需要）：

```bash
conda create -n nero_ik --override-channels -c conda-forge python=3.10 -y
conda activate nero_ik
conda install --override-channels -c conda-forge \
  "pinocchio=3.2.0" "casadi=3.6.7" scipy -y
python -m pip install lark "numpy==2.0.2" "empy==3.3.4" \
  pyyaml opencv-python ur-rtde netifaces meshcat
```

### 4. 编译并 source 工作空间

```bash
cd ~/agilex_ws/tego
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

`colcon list` 应至少能看到 `tego_sdk`、`tego_msgs`、`tego_ros`、`tego_bringup`、`tego_description`、`agx_arm_ctrl`、`agx_arm_msgs`。

---

## 使用说明

### 激活 CAN 模块

启动前**必须**先激活 CAN 接口。脚本会把 USB-CAN 绑定到逻辑名并按 1 Mbps 配置。

```bash
cd ~/agilex_ws/tego
bash src/tego_control/tego_bringup/scripts/find_all_can_port.sh
# 把 find_all_can_port.sh 输出的 USB 地址填到 src/tego_control/tego_bringup/scripts/can_config.sh
bash src/tego_control/tego_bringup/scripts/can_config.sh
ip -details link show type can
```

执行以上脚本需要 `sudo` 权限。`tego_bringup/udev/99-tego-can.rules` 是系统 udev 规则，复制到 `/etc/udev/rules.d/` 后可让 USB-CAN 端口名稳定。

启动 `demo_nero_bringup.launch.py` 时，逻辑接口命名约定为：

|逻辑接口|绑定硬件|
|---|---|
|`left_tego` / `right_tego`|主端手柄（左 / 右）|
|`left_nero` / `right_nero`|从臂示例机械臂（左 / 右）|

### 启动 Nero 双臂示例

> **重要提示：启动前必读**
> - **`left_arm_port` / `right_arm_port`**：从臂 CAN 逻辑接口名，默认 `left_nero` / `right_nero`
> - **`left_port_name` / `right_port_name`**：主端手柄 CAN 逻辑接口名，默认 `left_tego` / `right_tego`
> - **`start_arm`**：是否启动示例机械臂驱动 (`true` / `false`)
> - **`start_teleop`**：是否启动 Pinocchio 遥操节点 (`true` / `false`)
> - **`log_level`**：日志级别 (`debug` / `info` / `warn` / `error`)
>
> 全部参数的默认值与说明见下方 **[启动参数](#启动参数)**。

**一体化启动：**

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/tego/install/setup.bash
ros2 launch tego_bringup demo_nero_bringup.launch.py
```

**带参数启动：**

```bash
ros2 launch tego_bringup demo_nero_bringup.launch.py \
  left_arm_port:=left_nero \
  right_arm_port:=right_nero \
  left_port_name:=left_tego \
  right_port_name:=right_tego \
  log_level:=info
```

**分终端启动**（机械臂驱动、遥操节点、`tego_ros` 桥接分开调试）：

```bash
# 终端 1：从臂示例机械臂驱动
ros2 launch agx_arm_ctrl start_double_nero.launch.py

# 终端 2：Pinocchio 遥操节点（默认在 nero_ik Conda 环境里跑）
ros2 launch agx_arm_ctrl teleop_double_nero.launch.py

# 终端 3：手柄桥接和控制转发
ros2 launch tego_bringup double_nero_remote.launch.py
```

启动参数和 topic/service 路径与一体化启动完全一致；分终端适合在某个节点需要单独重启调试时使用。

### 遥操手柄操作说明

手柄扳机交互逻辑：

| 动作     | 功能描述           |
| :------- | :----------------- |
| **双击** | **开启/关闭遥操，默认关闭** |
| **三击** | **非遥操模式下，让nero臂回到预设零位** |
| **长按** | **非遥操模式下，初始化小臂和nero回零** |

### 启动参数

#### `demo_nero_bringup.launch.py`

|参数|类型|默认值|说明|
|---|---|---|---|
|`left_arm_port`|string|`left_nero`|左从臂 SocketCAN 接口名|
|`right_arm_port`|string|`right_nero`|右从臂 SocketCAN 接口名|
|`left_port_name`|string|`left_tego`|左手柄 SocketCAN 接口名|
|`right_port_name`|string|`right_tego`|右手柄 SocketCAN 接口名|
|`gripper_width_min`|float|`0.00`|夹爪宽度下限 (m)|
|`gripper_width_max`|float|`0.07`|夹爪宽度上限 (m)|
|`pub_rate`|float|`200.0`|桥接节点发布频率 (Hz)|
|`log_level`|string|`info`|日志级别|
|`start_arm`|bool|`true`|是否启动机械臂驱动|
|`start_teleop`|bool|`true`|是否启动 Pinocchio 遥操节点|

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
|`arm_zero_joint`|string|`0, 0, 0, 0, 0.099, 0, 0.0`|机械臂零位关节角 (rad)|
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
|`use_piper_modle`|bool|`false`|使用 Piper 模式|
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

遥操节点（`agx_arm_ctrl/teleop_single_nero_node.py`）发布；`tego_ros` 订阅，作为未滤波的关节目标。

#### `/control/move_js_left` & `/control/move_js_right`

消息类型：`sensor_msgs/JointState`

`tego_ros` 在遥操开启时把 `/control/joint_states_*_raw` 转发到这个 topic，机械臂驱动订阅。

> 该链路同样适用于 `agx_arm_ctrl` 暴露的其他控制 topic，例如 `/control/move_j{_left/_right}`、`/control/move_p{_left/_right}`、`/control/move_mit{_left/_right}`、`/control/gripper{_left/_right}` 等，详细字段见 [agx_arm_ros README](https://github.com/agilexrobotics/agx_arm_ros)。

### 机械臂反馈

#### `/feedback/joint_states_left` & `/feedback/joint_states_right`

消息类型：`sensor_msgs/JointState`，由 `agx_arm_ctrl` 发布的当前关节状态。

`/feedback/tcp_pose{_left/_right}`、`/feedback/arm_status{_left/_right}`、`/feedback/arm_ctrl_states{_left/_right}`、`/feedback/gripper_status{_left/_right}`、`/feedback/hand_status{_left/_right}` 也由 `agx_arm_ctrl` 发布，字段定义见对应 msg。

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
  - 逻辑接口名是否与 launch 参数一致（`left_tego` / `right_tego` / `left_nero` / `right_nero`）

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
- **Python 依赖导入失败**：用启动节点实际使用的 `python3 -c 'import ...'` 检查，不要把依赖装到另一个 Python 环境
- **构建结果异常**：确认已经 source `/opt/ros/humble/setup.bash` 和 `install/setup.bash`；必要时清理本地 `build/`、`install/`、`log/` 再重建
