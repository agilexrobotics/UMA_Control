# tego_control

[English](#english) | [中文](#中文)

`tego_control` integrates a six-axis teaching handle with ROS 2. It provides
the ROS bridge, Nero/Piper examples, profile-driven Pinocchio IK, and safety
constraints. Low-level CAN communication is provided by [`tego_sdk`](https://github.com/agilexrobotics/tego_sdk).

`tego_control` 将六轴示教器接入 ROS 2，提供 ROS 桥接、Nero/Piper 遥操示例、
基于 profile 配置的 Pinocchio 逆运动学和安全约束。底层 CAN 通信由
[`tego_sdk`](https://github.com/agilexrobotics/tego_sdk) 提供。

<a id="english"></a>
## English

<details>
<summary><strong>Workspace, dependencies and build</strong></summary>

Place the two independent repositories in one ROS 2 workspace:

```text
~/tego/
└── src/
    ├── tego_control/
    └── tego_sdk/
```

For a new workspace:

```bash
mkdir -p ~/tego/src
cd ~/tego/src
git clone https://github.com/agilexrobotics/tego_control.git
git clone https://github.com/agilexrobotics/tego_sdk.git
cd ~/tego
```

| Package / directory | Responsibility |
| --- | --- |
| `tego_bringup` | Demo launches, CAN scripts and robot profiles |
| `tego_ros` | C++ / Python handle bridges |
| `tego_msgs` | Handle messages and control service definitions |
| `tego_kinematics/agx_arm_ctrl` | Profile loading, IK and AGX example drivers |
| `tego_kinematics/arm_safety_monitor` | Offline RViz safety-space editor |
| `tego_description` | Example URDFs, meshes and RViz configuration |
| `../tego_sdk` | Independent repository for the handle CAN protocol |

ROS 2 Humble, colcon, SocketCAN, Eigen, Boost, KDL, and NLopt are required.
The IK and example arm drivers also require a Python 3.10 environment with
Pinocchio, CasADi, SciPy, and `pyAgxArm`.

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-colcon-recursive-crawl \
  python3-rosdep python3-catkin-pkg python3-pip \
  python3-numpy python3-scipy python3-tk python3-matplotlib \
  can-utils ethtool iproute2 \
  libboost-all-dev libeigen3-dev liborocos-kdl-dev \
  libnlopt-dev libnlopt-cxx-dev

CONDA_BASE="${CONDA_BASE:-$HOME/miniconda3}"
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda create -n tego --override-channels -c conda-forge python=3.10 -y
conda activate tego
conda install --override-channels -c conda-forge \
  "pinocchio=3.2.0" "casadi=3.6.7" scipy \
  typing_extensions exceptiongroup -y
python -m pip install lark "numpy==2.0.2" "empy==3.3.4" \
  pyyaml opencv-python ur-rtde netifaces meshcat
python -m pip install git+https://github.com/agilexrobotics/pyAgxArm.git
```

Both bridge backends use Python IK, so both need the `tego` environment.
Install IK and driver dependencies there; ROS 2 itself comes from the system
installation. Check that `python` points to `envs/tego/bin/python`:

```bash
python -c "import sys; print(sys.executable)"
python -c "import pinocchio, casadi, scipy, pyAgxArm; print('IK and driver dependencies OK')"
```

Build both repositories. The explicit system Python path is for ROS build
tools and message generation:

```bash
cd ~/tego
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src/tego_control src/tego_sdk \
  --ignore-src -r -y
colcon build \
  --base-paths src/tego_control src/tego_sdk \
  --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
source install/setup.bash
```

The Python bridge additionally needs:

```bash
conda activate tego
python -m pip install python-can
python -m pip install -e ~/tego/src/tego_sdk/python
python -c "import can, tego_sdk; print('Python bridge dependencies OK')"
```

The Python bridge also needs this ROS workspace build. The C++ bridge does
not need `python-can` or the Python SDK. Check package discovery with
`colcon list` and `ros2 pkg prefix tego_ros`.

</details>

<details>
<summary><strong>CAN configuration</strong></summary>

Run the following from the workspace root. Discover the USB-CAN adapters,
update their `bus-info` values in
`can_config.sh`, and activate the interfaces:

```bash
bash src/tego_control/tego_bringup/scripts/find_all_can_port.sh
bash src/tego_control/tego_bringup/scripts/can_config.sh
ip -details link show type can
```

| Interface | Device |
| --- | --- |
| `left_tego` / `right_tego` | Left / right teaching handle |
| `left_arm` / `right_arm` | Left / right example arm driver |

A single-arm setup only needs one pair, such as `right_tego + right_arm`.
Enable the matching lines in `can_config.sh` and comment out unused devices;
the current script enables the right pair. Dual-arm operation needs all four.
The USB addresses are machine-specific; keep the bitrate at 1 Mbps.

The optional `tego_bringup/udev/99-tego-can.rules` brings matching adapters up
at 1 Mbps on insertion. It does not assign the logical names above; use
`can_config.sh` for that.

</details>

<details>
<summary><strong>Start and operate teleoperation</strong></summary>

All four `start_*.sh` scripts automatically activate the `tego` Conda
environment. Set `CONDA_BASE` when Conda is not installed under
`~/miniconda3`; no manual `conda activate` is required.

```bash
# Nero dual arm
bash src/tego_control/tego_bringup/scripts/start_dual_nero_cpp.sh
bash src/tego_control/tego_bringup/scripts/start_dual_nero_python.sh

# Piper single arm; the right route is the default
bash src/tego_control/tego_bringup/scripts/start_single_piper_cpp.sh
bash src/tego_control/tego_bringup/scripts/start_single_piper_python.sh

# Left CAN pair / 左侧 CAN 接口
SIDE=left bash src/tego_control/tego_bringup/scripts/start_single_piper_cpp.sh \
  arm_port:=left_arm port_name:=left_tego
```

`SIDE` selects the single-arm ROS route. With left-side CAN names, also pass
`arm_port:=left_arm port_name:=left_tego`; changing `SIDE` alone keeps the
demo's right-side port defaults. Choose one bridge backend per interface.

| Script setting | Default / use |
| --- | --- |
| `CONDA_BASE` | `~/miniconda3`; override for another Conda installation |
| `WORKSPACE_DIR` | Auto-detected directory containing `install/setup.bash` |
| `SIDE` | Piper only: `right` or `left` |
| `LAUNCH_FILE` | Model's demo launch in `tego_bringup` |
| Trailing `key:=value` | Forwarded to `ros2 launch` |

For example, add `profile_path:=/absolute/path/my_profile.yaml` to a start
command. Scripts load Conda, ROS and the workspace, and clean up matching
stack processes on startup and exit, including Ctrl+C. This cleanup can also
stop separately launched instances with matching process names; use direct
launches for parallel debugging.

For direct `ros2 launch`, prepare each terminal first:

```bash
cd ~/tego
CONDA_BASE="${CONDA_BASE:-$HOME/miniconda3}"
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate tego
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONNOUSERSITE=1
```

Then choose one entry point:

```bash
ros2 launch tego_bringup demo_nero_bringup.launch.py backend:=cpp
ros2 launch tego_bringup demo_nero_bringup.launch.py backend:=python
ros2 launch tego_bringup demo_piper.launch.py
ros2 launch tego_bringup demo_piper.launch.py side:=left backend:=python \
  arm_port:=left_arm port_name:=left_tego
```

Set `start_arm:=false` when using an external arm driver. This only disables
the repository's example driver.

Double-click the inner side key to toggle teleoperation. Long-press it while
teleoperation is off to initialize the handle and home the arm. Homing sends
motion commands, so keep the path clear. The demo button timing is 0.30 s for
a double press and 2.0 s for a long press. Trigger pressure is also used for
gripper width in supported configurations.

</details>

<details>
<summary><strong>Profiles and installation geometry</strong></summary>

Example profiles are stored in `tego_bringup/config/`:

| Profile | Default use |
| --- | --- |
| `nero_profile.yaml` | Side-mounted Nero dual arm |
| `piper_profile.yaml` | Upright Piper single arm |

`side` and `installation_pos` have separate responsibilities:

- `side: left/right` selects topics, services, node names and the inner
  handle button. CAN ports are separate launch arguments.
- `installation_pos: horizontal/left/right` describes physical mounting.

An upright arm connected through the right route therefore uses `side:=right`
with this profile value:

```yaml
robot:
  installation_pos: horizontal
```

This applies to Piper, Nero, and other arms. A dual-arm profile may select the
installation per route:

```yaml
robot:
  installation_pos:
    left: left
    right: right
```

A mapping may also provide `default`. Invalid values, or a selected side with
no matching key or default, stop startup. A missing field defaults to
`horizontal`. When migrating a side-mounted custom profile from v0.1.0,
set `installation_pos` explicitly.

Translation is mapped from handle `world` to arm `base_link`:

| Installation | `world +x` | `world +y` | `world +z` |
| --- | --- | --- | --- |
| `horizontal` | `base +x` | `base +y` | `base +z` |
| `left` | `base +y` | `base +z` | `base +x` |
| `right` | `base -y` | `base -z` | `base +x` |

The `left/right` mappings preserve the v0.1.0 Nero home-FK directions.
`horizontal` prevents the home end-effector rotation from swapping translation
axes. The correction changes translation only; orientation deltas retain the
existing path. Piper also receives the same value for gravity compensation.

`arm_home_state` remains the home command and FK reference; the live session
origin comes from current joint feedback. Each list contains one arm's driver
channels, with the first `nq` entries in URDF arm order. The bundled Piper
profile has 6 entries; Nero has 10 per side (7 arm + 3 end-effector channels).
These lengths must match the actual driver feedback used for homing.

| Profile block | What to configure |
| --- | --- |
| `robot` | URDF, base/tool frames, locked joints, home state and mounting |
| `ik` | Per-step joint change limit, posture and singularity penalties |
| `handle` | `zoom_factor`: Nero 1.4, Piper 1.1 in the bundled profiles |
| `safety` | Workspace box, virtual walls and protected points |

The examples use `ik.max_delta_q_rad: 0.1`. It limits the joint change per IK
step, not motor speed. `robot.speed_percent` in the current YAML is **not read
by the demo launches**. To set speed, launch the example driver separately
with `speed_percent:=30` and use `start_arm:=false` for bringup, as below.

See the [Arm Adaptation Guide](机械臂适配指南.md) for complete profile and driver
examples, including custom arms and grippers.

</details>

<details>
<summary><strong>Runtime interfaces and arm integration</strong></summary>

Main data path:

```text
Teaching-handle CAN
  -> tego_ros
  -> /left_handle_pose or /right_handle_pose
  -> Pinocchio IK
  -> /control/joint_states_{left,right}_raw
  -> tego_ros, while teleoperation is enabled
  -> /control/move_js_{left,right}
  -> arm driver
```

| Interface | Type | Purpose |
| --- | --- | --- |
| `/left_handle_pose`, `/right_handle_pose` | `geometry_msgs/msg/PoseStamped` | Handle end pose |
| `/feedback/joint_states_{left,right}` | `sensor_msgs/msg/JointState` | Live arm feedback |
| `/control/joint_states_{left,right}_raw` | `sensor_msgs/msg/JointState` | Raw IK target |
| `/control/move_js_{left,right}` | `sensor_msgs/msg/JointState` | Target forwarded to the driver |
| `/left_teleop_enable`, `/right_teleop_enable` | `std_msgs/msg/Bool` | Teleoperation state |
| `/left_teleop_trigger`, `/right_teleop_trigger` | `std_srvs/srv/Trigger` | Toggle teleoperation |

An external driver must publish `/feedback/joint_states_{side}` and subscribe
to `/control/move_js_{side}`. Joint values use radians. Joint naming,
direction, zero offsets, enabling, grippers, and vendor transport belong to
the driver.

Fresh joint feedback is required before teleoperation starts. A feedback
timeout stops the session and clears its installation mapping. Double-click
the inner handle button to toggle teleoperation. Long-press it while disabled
to initialize the handle and home the arm.

Demo launch arguments:

| Argument | Default | Purpose |
| --- | --- | --- |
| `backend` | `cpp` | Handle bridge implementation |
| `side` | `right` for Piper | Single-arm communication route |
| `profile_path` | Example profile | URDF, IK, mounting, and safety configuration |
| `start_arm` | `true` | Start the example arm driver |
| `start_teleop` | `true` | Start the IK teleoperator |
| `gripper_width_min/max` | `0.00/0.07` | Gripper range in metres |
| `pub_rate` | `200.0` | Bridge publication rate in Hz |
| `log_level` | `info` | Bridge and example-driver logging |
| `arm_port`, `port_name` | `right_arm`, `right_tego` | Piper arm and handle CAN ports |
| `left_arm_port`, `right_arm_port` | `left_arm`, `right_arm` | Nero arm CAN ports |
| `left_port_name`, `right_port_name` | `left_tego`, `right_tego` | Nero handle CAN ports |

</details>

<details>
<summary><strong>Separate launches and diagnostics</strong></summary>

For a Piper on the right route, load the manual environment in each terminal
and run these commands from the workspace root:

```bash
# Terminal 1: example driver / 终端 1：示例驱动
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  arm_type:=piper can_port:=right_arm topics_end:=_right \
  installation_pos:=horizontal speed_percent:=30

# Terminal 2: IK / 终端 2：IK
ros2 launch agx_arm_ctrl teleop_right_arm.launch.py \
  profile_path:=$PWD/src/tego_control/tego_bringup/config/piper_profile.yaml \
  feedback_timeout_sec:=1.0

# Terminal 3: handle bridge / 终端 3：示教器 bridge
ros2 launch tego_bringup right_piper.launch.py port_name:=right_tego \
  profile_path:=$PWD/src/tego_control/tego_bringup/config/piper_profile.yaml
```

Use `right_piper_python.launch.py` for the Python bridge. For Nero, the
corresponding driver/IK entries are `start_double_arm.launch.py` and
`teleop_double_arm.launch.py`; a bridge-only demo uses
`start_arm:=false start_teleop:=false`.

The low-level IK launches require `profile_path`; the single-arm entry also
accepts `side` and `node_name`. `feedback_timeout_sec` defaults to 1.0 s.
Keep the driver mounting value consistent with the profile.

`run_single_node.launch.py` is the advanced C++ bridge entry. It requires
`arm_home_state` as a comma-separated string and explicit topic mappings.
Its defaults differ from the model wrappers: `port_name=can0`,
`gripper_width_max=0.10`, `zoom_factor=2.0`, `use_pose_dt_zoom=false` and
`use_trigger_pose=false`. Demo wrappers supply the model geometry and enable
the last two options. Use the [adaptation example](机械臂适配指南.md) when
constructing your own bridge launch.

</details>

<details>
<summary><strong>Handle telemetry, services and link checks</strong></summary>

| Handle suffix | Type | Useful fields |
| --- | --- | --- |
| `/joint` | `sensor_msgs/msg/JointState` | Handle joints, not arm feedback |
| `/imu` | `sensor_msgs/msg/Imu` | Orientation, angular velocity, acceleration |
| `/handle_state` | `tego_msgs/msg/HandleState` | `joystick_x/y`, `trigger_press_percentage`, key events and receive counts |
| `/mt_sensor_status` | `tego_msgs/msg/MtSensorStatus` | `name`, `data`, `receive_count`; each sensor has `angle_deg/rad`, `voltage_x/y` |

Prefix these with `/tego_l` or `/tego_r`, for example `/tego_r/handle_state`.
A connected CAN socket alone does not prove data is arriving; inspect receive
counts and topic rates.

| Service suffix | Type | Action |
| --- | --- | --- |
| `/reset_pose` | `std_srvs/srv/Empty` | Recapture the handle pose reference |
| `/reset_control` | `tego_msgs/srv/ResetControl` | Encoder reset: `reset_mode` 0 = zero, 1 = factory |
| `/enable_handle_motor` | `std_srvs/srv/SetBool` | Start / stop handle motor |
| `/enable_reverse` | `std_srvs/srv/SetBool` | Change handle base reversal |

These services also use `/tego_l` or `/tego_r`. They are distinct from the IK
services `/left_teleop_trigger` and `/right_teleop_trigger`, which **toggle**
the state on each call. Reset and homing are different operations.

Read-only checks for the right route (replace `right` / `tego_r` for the left):

```bash
ros2 topic hz /right_handle_pose
ros2 topic echo /tego_r/handle_state --once
ros2 topic echo /feedback/joint_states_right --once
ros2 topic echo /right_teleop_enable --once
ros2 topic echo /control/joint_states_right_raw --once
ros2 topic echo /control/move_js_right --once
ros2 service list
```

The last two topics may be quiet while teleoperation is off. Trace handle
pose → feedback and enable state → raw IK output → final command.
The bridge also sends homing commands while teleoperation is disabled.

</details>

<details>
<summary><strong>Safety-space editor</strong></summary>

Edit profile workspace and virtual-wall constraints offline:

```bash
ros2 launch arm_safety_monitor safety_monitor.launch.py
ros2 launch arm_safety_monitor safety_monitor.launch.py \
  profile_name:=nero_profile.yaml
```

The launch starts RViz, `robot_state_publisher`, `joint_state_publisher_gui`
and the editor. It loads the URDF from the profile; no arm or CAN feedback is
needed. InteractiveMarkers uses `/arm_safety/update`.

1. Set RViz **Fixed Frame** to `robot.base_frame` (normally `base_link`).
2. Move the joint GUI sliders to inspect arm poses.
3. Right-click the green base marker to create a Box or Plane. Existing
   objects have the same editing menu.
4. Drag a Box to move its centre; use its menu to resize or delete it.
5. Move a Plane and adjust its green normal arrow. The wall menu can flip
   the normal. Check `keep_side` and the protected frames in the profile.
6. Choose **Save profile**, then restart the teleoperation node to apply it.

Saving atomically updates the `safety` block and preserves other blocks and
their comments. Source profiles are preferred; installed package data is the
fallback. Use `TEGO_PROFILE_DIR` to choose a writable directory. The Box menu
step is controlled by `box_resize_step` (0.05 m); `plane_size` (0.8 m) changes
only the displayed wall size. The constraint is an infinite plane.

Constraints apply to the origins of `protected_frames`, not every robot link.
If the profile uses `tool0` instead of a generated `ee` frame, also update the
editor's new-object default `protected_frames: [ee]` before using that profile.

</details>

<details>
<summary><strong>Troubleshooting</strong></summary>

Before connecting motion output, inspect the IK target in the visualization.
Then test at low hardware speed: verify all three translation axes, no
first-frame jump, orientation behavior and Piper gravity compensation.

| Symptom | Check |
| --- | --- |
| No CAN data | `bus-info`, 1 Mbps bitrate, and `ip -details link show type can` |
| No motion | Teleop enable, live feedback, raw target, and `move_js` topics |
| Session stops immediately | Feedback rate and `feedback_timeout_sec` |
| Upright x/z are swapped | `installation_pos: horizontal` and the current install space |
| IK is infeasible | URDF frames, limits, workspace, walls, collisions, and `zoom_factor` |
| RViz has missing TF | URDF, `base_frame`, state publishers, and Fixed Frame |

For `Infeasible_Problem_Detected`, first confirm the loaded profile and URDF
frames. Then check joint limits, the Box bounds/margin, wall normal/keep side
and protected points; inspect `wall_dist` in the logs. Reduce handle scaling
and restart from a reachable pose. `collision_pairs` checks self-collision
after solving; it is not an Opti hard constraint.

For import failures, inspect `which python3` and rerun the dependency checks in
the `tego` environment. Rebuild and source `install/setup.bash` if ROS packages
or updated launch files are missing. Use hardware speed limits and the arm's
own stop mechanism during commissioning.

</details>

<a id="中文"></a>
## 中文

<details>
<summary><strong>工作空间、依赖与构建</strong></summary>

`tego_control` 和 `tego_sdk` 是两个独立 Git 仓库，建议放在同一个 ROS 2 工作空间：

```text
~/tego/
└── src/
    ├── tego_control/
    └── tego_sdk/
```

首次创建工作空间：

```bash
mkdir -p ~/tego/src
cd ~/tego/src
git clone https://github.com/agilexrobotics/tego_control.git
git clone https://github.com/agilexrobotics/tego_sdk.git
cd ~/tego
```

| 包 / 目录 | 用途 |
| --- | --- |
| `tego_bringup` | 示例 launch、CAN 脚本和机械臂 profile |
| `tego_ros` | C++ / Python 示教器 bridge |
| `tego_msgs` | 手柄消息和控制服务定义 |
| `tego_kinematics/agx_arm_ctrl` | Profile 加载、IK 和 AGX 示例驱动 |
| `tego_kinematics/arm_safety_monitor` | RViz 离线安全空间编辑器 |
| `tego_description` | 示例 URDF、mesh 和 RViz 配置 |
| `../tego_sdk` | 独立仓库，提供示教器 CAN 协议 |

系统需要 ROS 2 Humble、colcon、SocketCAN、Eigen、Boost、KDL 和 NLopt。IK
与示例机械臂驱动还需要 Python 3.10、Pinocchio、CasADi、SciPy 和 `pyAgxArm`。

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-colcon-recursive-crawl \
  python3-rosdep python3-catkin-pkg python3-pip \
  python3-numpy python3-scipy python3-tk python3-matplotlib \
  can-utils ethtool iproute2 \
  libboost-all-dev libeigen3-dev liborocos-kdl-dev \
  libnlopt-dev libnlopt-cxx-dev

CONDA_BASE="${CONDA_BASE:-$HOME/miniconda3}"
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda create -n tego --override-channels -c conda-forge python=3.10 -y
conda activate tego
conda install --override-channels -c conda-forge \
  "pinocchio=3.2.0" "casadi=3.6.7" scipy \
  typing_extensions exceptiongroup -y
python -m pip install lark "numpy==2.0.2" "empy==3.3.4" \
  pyyaml opencv-python ur-rtde netifaces meshcat
python -m pip install git+https://github.com/agilexrobotics/pyAgxArm.git
```

两种 bridge 都使用 Python IK，因此都需要 `tego` 环境。IK 和驱动依赖安装在该
环境中，ROS 2 使用系统安装。确认 `python` 指向 `envs/tego/bin/python`：

```bash
python -c "import sys; print(sys.executable)"
python -c "import pinocchio, casadi, scipy, pyAgxArm; print('IK and driver dependencies OK')"
```

构建两个仓库；显式指定的系统 Python 用于 ROS 构建工具和消息生成：

```bash
cd ~/tego
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src/tego_control src/tego_sdk \
  --ignore-src -r -y
colcon build \
  --base-paths src/tego_control src/tego_sdk \
  --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
source install/setup.bash
```

Python bridge 还需安装：

```bash
conda activate tego
python -m pip install python-can
python -m pip install -e ~/tego/src/tego_sdk/python
python -c "import can, tego_sdk; print('Python bridge dependencies OK')"
```

Python bridge 同样需要构建 ROS 工作空间；C++ bridge 不需要 `python-can` 和
Python SDK。可用 `colcon list` 和 `ros2 pkg prefix tego_ros` 检查包是否被发现。

</details>

<details>
<summary><strong>CAN 配置</strong></summary>

在工作空间根目录查找 USB-CAN，更新 `can_config.sh` 中的 `bus-info`，再启动接口：

```bash
bash src/tego_control/tego_bringup/scripts/find_all_can_port.sh
bash src/tego_control/tego_bringup/scripts/can_config.sh
ip -details link show type can
```

| 接口名 | 设备 |
| --- | --- |
| `left_tego` / `right_tego` | 左 / 右示教器 |
| `left_arm` / `right_arm` | 左 / 右示例机械臂驱动 |

单臂只需一对接口，例如 `right_tego + right_arm`。在 `can_config.sh` 中启用对应
行，注释未连接的设备；当前脚本启用右侧一对，双臂需要启用四路。USB 地址按本机
修改，bitrate 保持 1 Mbps。

可选的 `tego_bringup/udev/99-tego-can.rules` 会在插入匹配设备时以 1 Mbps 启动
接口。它不分配上述逻辑名称，命名仍由 `can_config.sh` 完成。

</details>

<details>
<summary><strong>启动与手柄操作</strong></summary>

四个 `start_*.sh` 会自动激活 `tego` Conda 环境，无需手动执行
`conda activate`。如果 Conda 不在 `~/miniconda3`，启动时设置 `CONDA_BASE`。

```bash
# Nero 双臂
bash src/tego_control/tego_bringup/scripts/start_dual_nero_cpp.sh
bash src/tego_control/tego_bringup/scripts/start_dual_nero_python.sh

# Piper 单臂，默认右侧通信链路
bash src/tego_control/tego_bringup/scripts/start_single_piper_cpp.sh
bash src/tego_control/tego_bringup/scripts/start_single_piper_python.sh

# Left CAN pair / 左侧 CAN 接口
SIDE=left bash src/tego_control/tego_bringup/scripts/start_single_piper_cpp.sh \
  arm_port:=left_arm port_name:=left_tego
```

`SIDE` 选择单臂 ROS 路由。使用左侧 CAN 名称时还需传入
`arm_port:=left_arm port_name:=left_tego`，只改 `SIDE` 会保留 demo 的右侧端口
默认值。同一接口选择一种 bridge 即可。

| 脚本设置 | 默认值 / 用途 |
| --- | --- |
| `CONDA_BASE` | `~/miniconda3`；可指定其他 Conda 安装目录 |
| `WORKSPACE_DIR` | 自动查找含 `install/setup.bash` 的工作空间 |
| `SIDE` | 仅 Piper：`right` 或 `left` |
| `LAUNCH_FILE` | `tego_bringup` 中对应机型的 demo launch |
| 末尾的 `key:=value` | 透传给 `ros2 launch` |

例如在启动命令末尾添加 `profile_path:=/绝对路径/my_profile.yaml`。脚本会加载
Conda、ROS 和工作空间，并在启动前及退出时清理匹配的进程，支持 Ctrl+C 退出。
清理也可能结束单独启动的同名实例；并行调试时使用下面的直接 launch。

手动执行 `ros2 launch` 前，每个终端先加载环境：

```bash
cd ~/tego
CONDA_BASE="${CONDA_BASE:-$HOME/miniconda3}"
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate tego
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONNOUSERSITE=1
```

再选择一个入口：

```bash
ros2 launch tego_bringup demo_nero_bringup.launch.py backend:=cpp
ros2 launch tego_bringup demo_nero_bringup.launch.py backend:=python
ros2 launch tego_bringup demo_piper.launch.py
ros2 launch tego_bringup demo_piper.launch.py side:=left backend:=python \
  arm_port:=left_arm port_name:=left_tego
```

使用自有机械臂驱动时传入 `start_arm:=false`，该参数只关闭仓库示例驱动。

双击内侧按键切换遥操；遥操关闭时长按该键，初始化示教器并让机械臂回零。
回零会发送运动命令，操作前留出运动空间。Demo 的双击窗口为 0.30 s，长按时间为
2.0 s。支持夹爪的配置还会使用扳机按压量映射夹爪宽度。

</details>

<details>
<summary><strong>Profile 和安装方向</strong></summary>

示例 profile 位于 `tego_bringup/config/`：

| Profile | 默认用途 |
| --- | --- |
| `nero_profile.yaml` | Nero 左右侧装双臂 |
| `piper_profile.yaml` | Piper 正装单臂 |

`side` 与 `installation_pos` 的职责不同：

- `side: left/right` 选择 topic、service、节点名和示教器内侧按键；CAN 端口单独指定。
- `installation_pos: horizontal/left/right` 描述机械臂的物理安装方向。

接在右侧通信链路的正装机械臂使用 `side:=right`，profile 配置为：

```yaml
robot:
  installation_pos: horizontal
```

这套规则适用于 Piper、Nero 和其他机械臂。双臂可按通信侧分别配置：

```yaml
robot:
  installation_pos:
    left: left
    right: right
```

映射还可提供 `default`。配置值非法，或当前 `side` 缺少对应项且没有 `default`
时，节点会拒绝启动。字段完全缺失时默认使用 `horizontal`；从 v0.1.0 迁移侧装
自定义 profile 时，应显式补上安装方向。

安装矩阵把示教器 `world` 平移转换到机械臂 `base_link`：

| 安装方向 | `world +x` | `world +y` | `world +z` |
| --- | --- | --- | --- |
| `horizontal` | `base +x` | `base +y` | `base +z` |
| `left` | `base +y` | `base +z` | `base +x` |
| `right` | `base -y` | `base -z` | `base +x` |

`left/right` 保留 v0.1.0 Nero 两侧 `arm_home_state` 的实际 FK 方向。
`horizontal` 避免末端 home 姿态错误交换平移轴。补偿只作用于目标平移，原有姿态
增量路径不变。Piper 还会收到同一个值，用于控制器重力补偿。

`arm_home_state` 继续作为回零命令和 FK 参考；本次遥操起点来自实时关节反馈。
每个列表只填一侧的驱动通道，前 `nq` 项按 URDF 臂关节顺序排列。示例 Piper 为
6 项，Nero 每侧为 10 项（7 个臂关节 + 3 个末端通道），回零列表长度需与实际驱动
反馈一致。

| Profile 区块 | 配置内容 |
| --- | --- |
| `robot` | URDF、基座/末端 frame、锁定关节、回零状态和安装方向 |
| `ik` | 单帧关节变化限制、构型和奇异性惩罚 |
| `handle` | `zoom_factor`，示例 Nero 为 1.4，Piper 为 1.1 |
| `safety` | 工作空间 Box、虚拟墙和保护点 |

示例的 `ik.max_delta_q_rad: 0.1` 限制 IK 每步关节变化，不等于电机速度。当前
高层 demo **未读取 YAML 中的 `robot.speed_percent`**。调速时可单独启动示例
驱动并传入 `speed_percent:=30`，再用 `start_arm:=false` 启动其余链路，见下文。

完整 profile、驱动和夹爪示例见[机械臂适配指南](机械臂适配指南.md)。

</details>

<details>
<summary><strong>运行接口与机械臂适配</strong></summary>

主数据链路：

```text
示教器 CAN
  -> tego_ros
  -> /left_handle_pose 或 /right_handle_pose
  -> Pinocchio IK
  -> /control/joint_states_{left,right}_raw
  -> tego_ros（仅在遥操开启时转发）
  -> /control/move_js_{left,right}
  -> 机械臂驱动
```

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/left_handle_pose`, `/right_handle_pose` | `geometry_msgs/msg/PoseStamped` | 示教器末端位姿 |
| `/feedback/joint_states_{left,right}` | `sensor_msgs/msg/JointState` | 机械臂实时反馈 |
| `/control/joint_states_{left,right}_raw` | `sensor_msgs/msg/JointState` | IK 原始目标 |
| `/control/move_js_{left,right}` | `sensor_msgs/msg/JointState` | 转发给驱动的目标 |
| `/left_teleop_enable`, `/right_teleop_enable` | `std_msgs/msg/Bool` | 遥操开关状态 |
| `/left_teleop_trigger`, `/right_teleop_trigger` | `std_srvs/srv/Trigger` | 切换遥操 |

自有驱动至少需要发布 `/feedback/joint_states_{side}`，并订阅
`/control/move_js_{side}`。关节单位为 rad。关节名、方向、零位、使能、夹爪和厂商
通信由驱动负责。

开启遥操前必须有新鲜关节反馈。反馈超时后，节点会停止本次会话并清除安装映射。
双击内侧按键切换遥操；遥操关闭时长按该键，可初始化示教器并让机械臂回零。

Demo launch 参数：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `backend` | `cpp` | 示教器 bridge 实现 |
| `side` | Piper 为 `right` | 单臂通信路由 |
| `profile_path` | 对应示例 profile | URDF、IK、安装方向和安全配置 |
| `start_arm` | `true` | 启动示例机械臂驱动 |
| `start_teleop` | `true` | 启动 IK 遥操节点 |
| `gripper_width_min/max` | `0.00/0.07` | 夹爪范围，单位 m |
| `pub_rate` | `200.0` | bridge 发布频率，单位 Hz |
| `log_level` | `info` | bridge 和示例驱动的日志级别 |
| `arm_port`, `port_name` | `right_arm`, `right_tego` | Piper 机械臂、示教器 CAN 端口 |
| `left_arm_port`, `right_arm_port` | `left_arm`, `right_arm` | Nero 机械臂 CAN 端口 |
| `left_port_name`, `right_port_name` | `left_tego`, `right_tego` | Nero 示教器 CAN 端口 |

</details>

<details>
<summary><strong>分步启动与调试</strong></summary>

以右侧 Piper 为例，各终端先加载上面的手动启动环境，再从工作空间根目录执行：

```bash
# Terminal 1: example driver / 终端 1：示例驱动
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  arm_type:=piper can_port:=right_arm topics_end:=_right \
  installation_pos:=horizontal speed_percent:=30

# Terminal 2: IK / 终端 2：IK
ros2 launch agx_arm_ctrl teleop_right_arm.launch.py \
  profile_path:=$PWD/src/tego_control/tego_bringup/config/piper_profile.yaml \
  feedback_timeout_sec:=1.0

# Terminal 3: handle bridge / 终端 3：示教器 bridge
ros2 launch tego_bringup right_piper.launch.py port_name:=right_tego \
  profile_path:=$PWD/src/tego_control/tego_bringup/config/piper_profile.yaml
```

Python bridge 改用 `right_piper_python.launch.py`。Nero 对应的驱动和 IK 入口
为 `start_double_arm.launch.py`、`teleop_double_arm.launch.py`；只启动示教器
bridge 时，给 Nero demo 传入 `start_arm:=false start_teleop:=false`。

低层 IK launch 必须提供 `profile_path`；单臂入口还支持 `side` 和 `node_name`。
`feedback_timeout_sec` 默认 1.0 s。单独启动驱动时，安装方向应与 profile 一致。

`run_single_node.launch.py` 是高级 C++ bridge 入口，需要逗号分隔的
`arm_home_state` 和显式 topic 映射。它的默认值与机型 wrapper 不同：
`port_name=can0`、`gripper_width_max=0.10`、`zoom_factor=2.0`、
`use_pose_dt_zoom=false`、`use_trigger_pose=false`。Demo wrapper 会提供手柄
几何参数并启用后两项。自建 bridge launch 可参考[适配示例](机械臂适配指南.md)。

</details>

<details>
<summary><strong>手柄状态、服务与链路检查</strong></summary>

| 手柄 topic 后缀 | 类型 | 常用字段 |
| --- | --- | --- |
| `/joint` | `sensor_msgs/msg/JointState` | 示教器关节，不是机械臂反馈 |
| `/imu` | `sensor_msgs/msg/Imu` | 姿态、角速度、加速度 |
| `/handle_state` | `tego_msgs/msg/HandleState` | `joystick_x/y`、`trigger_press_percentage`、按键事件和接收计数 |
| `/mt_sensor_status` | `tego_msgs/msg/MtSensorStatus` | `name`、`data`、`receive_count`；每个传感器含 `angle_deg/rad`、`voltage_x/y` |

以上加 `/tego_l` 或 `/tego_r` 前缀，例如 `/tego_r/handle_state`。CAN socket
连接成功不代表收到数据，可结合接收计数和 topic 频率确认。

| 服务后缀 | 类型 | 动作 |
| --- | --- | --- |
| `/reset_pose` | `std_srvs/srv/Empty` | 重新采集示教器位姿参考 |
| `/reset_control` | `tego_msgs/srv/ResetControl` | 编码器复位，`reset_mode` 0 = 零位，1 = 出厂 |
| `/enable_handle_motor` | `std_srvs/srv/SetBool` | 启停手柄电机 |
| `/enable_reverse` | `std_srvs/srv/SetBool` | 切换手柄基座反向 |

服务同样加 `/tego_l` 或 `/tego_r` 前缀。IK 的 `/left_teleop_trigger` 和
`/right_teleop_trigger` 则每次调用都会**切换**遥操状态。复位服务与机械臂回零
是不同操作。

右侧链路的只读检查命令；左侧替换 `right` 和 `tego_r`：

```bash
ros2 topic hz /right_handle_pose
ros2 topic echo /tego_r/handle_state --once
ros2 topic echo /feedback/joint_states_right --once
ros2 topic echo /right_teleop_enable --once
ros2 topic echo /control/joint_states_right_raw --once
ros2 topic echo /control/move_js_right --once
ros2 service list
```

遥操关闭时，最后两个 topic 可能没有数据。按“手柄位姿 → 反馈与开关 → IK 原始
输出 → 最终命令”的顺序检查。遥操关闭期间，bridge 仍可发送回零命令。

</details>

<details>
<summary><strong>安全空间编辑器</strong></summary>

离线编辑 profile 中的 workspace 和 virtual walls：

```bash
ros2 launch arm_safety_monitor safety_monitor.launch.py
ros2 launch arm_safety_monitor safety_monitor.launch.py \
  profile_name:=nero_profile.yaml
```

Launch 会启动 RViz、`robot_state_publisher`、`joint_state_publisher_gui` 和
编辑器，并从 profile 加载 URDF；不需要机械臂或 CAN 反馈。InteractiveMarkers
的 Update Topic 为 `/arm_safety/update`。

1. 将 RViz **Fixed Frame** 设为 `robot.base_frame`，通常是 `base_link`。
2. 拖动关节 GUI 滑块，检查不同姿态。
3. 右键基座处绿色球新建 Box 或 Plane；已有对象也可直接右键编辑。
4. 拖动 Box 移动中心，用菜单调整尺寸或删除。
5. 移动 Plane，调整绿色法向箭头；菜单可翻转法向。结合 profile 检查
   `keep_side` 和 `protected_frames`。
6. 选择**保存 profile**，重启遥操节点使约束生效。

保存时原子更新 `safety` 块，保留其他区块及其注释。优先写源码 profile，找不到
时回退到安装目录；可用 `TEGO_PROFILE_DIR` 指定可写目录。`box_resize_step`
控制 Box 菜单的调整步长，默认 0.05 m；`plane_size` 默认 0.8 m，只影响墙面显示
尺寸，实际约束是无限平面。

约束只保护 `protected_frames` 的原点，不会自动覆盖所有连杆。若 profile 使用
`tool0` 而没有生成 `ee`，编辑器新建对象后还需把默认的 `protected_frames: [ee]`
改为实际 frame。

</details>

<details>
<summary><strong>排障</strong></summary>

先在可视化中检查 IK 目标，再以硬件低速接入运动。检查三个平移轴、首帧无跳位、
姿态行为，以及 Piper 重力补偿方向。

| 现象 | 检查项 |
| --- | --- |
| CAN 无数据 | `bus-info`、1 Mbps bitrate、`ip -details link show type can` |
| 遥操无动作 | 遥操开关、实时反馈、raw 目标和 `move_js` topic |
| 开启后立即停止 | 反馈频率和 `feedback_timeout_sec` |
| 正装 x/z 互换 | `installation_pos: horizontal` 和当前 install 空间 |
| IK infeasible | URDF frame、关节限制、workspace、墙、碰撞对和 `zoom_factor` |
| RViz 缺 TF | URDF、`base_frame`、状态发布节点和 Fixed Frame |

遇到 `Infeasible_Problem_Detected`，先确认加载的 profile 和 URDF frame，再查
关节限位、Box 范围/余量、墙的法向/保留侧和保护点，结合日志中的 `wall_dist`
定位。降低手柄缩放，从可达姿态重新开始。`collision_pairs` 是求解后的自碰撞
检测，不是 Opti 硬约束。

导入失败时检查 `which python3`，并在 `tego` 环境重做依赖检查。ROS 包或新 launch
不可见时，重新构建并加载 `install/setup.bash`。真机调试的速度限制和停止机制由
机械臂驱动及硬件提供。

</details>
