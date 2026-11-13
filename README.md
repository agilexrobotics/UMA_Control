# UMA_Control

[English](#english) | [中文](#中文)

## Quick start / 快速开始

**English**

1. From `~/uma`, run `bash src/UMA_Control/build.sh`; create the workspace and clone both repositories first when needed.
2. Run `bash src/UMA_Control/uma_bringup/scripts/can_config.sh` to configure the CAN interfaces.
3. Choose a startup command from the [English](#english) section.
4. Run the checks in [Verification](#验证).

**中文**

1. 在 `~/uma` 工作空间运行 `bash src/UMA_Control/build.sh`；首次使用时先按“工作空间、依赖与构建”创建目录并克隆仓库。
2. 运行 `bash src/UMA_Control/uma_bringup/scripts/can_config.sh` 配置 CAN 接口。
3. 从[中文](#中文)选择一组启动命令。
4. 按[验证](#验证)检查构建、接口、启动日志和真机运动。

<a id="english"></a>
## English

<details>
<summary><strong>Workspace, dependencies and build</strong></summary>

Create `~/uma` as the ROS 2 workspace, with both independent repositories under `~/uma/src`:

```bash
mkdir -p ~/uma/src
cd ~/uma/src
git clone https://github.com/agilexrobotics/UMA_Control.git
git clone https://github.com/agilexrobotics/UMA_SDK.git
cd ~/uma
bash src/UMA_Control/build.sh
```

Choose a mode when the script prompts:

- **Full build:** install dependencies, build the workspace and verify it. Select the bridge backend when prompted.
- **Rebuild:** delete `build/ install/ log/`, rebuild the workspace and verify it.

Both bridge backends use Python IK and require the `uma` Conda environment. The `python` backend also needs `python-can` and the Python SDK. Colcon builds the C++ packages for either selection.

| Ubuntu | Native ROS 2 | Conda Python |
| --- | --- | --- |
| 22.04 | Humble | 3.10 |
| 24.04 | Jazzy | 3.12 |

Install the matching ROS distribution using the [official instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) before running `build.sh`. The build and startup scripts select the distribution from the operating system inside the current environment. The native 24.04 installation uses Jazzy directly. Pinocchio 3.2.0 and CasADi 3.6.7 remain pinned.

For migration, use a fresh workspace containing the two source repositories. Build artifacts contain absolute paths and Python extensions specific to the ROS distribution. If an existing `uma` environment uses Python 3.10, create a separate environment through the full build:

```bash
CONDA_ENV=uma_jazzy bash src/UMA_Control/build.sh
CONDA_ENV=uma_jazzy bash src/UMA_Control/uma_bringup/scripts/start_single_piper_cpp.sh
```

Use the same `CONDA_ENV` for subsequent starts. An incompatible Python version or workspace platform record stops startup before launching nodes. Use the rebuild mode when migrating an existing workspace after preparing the matching Conda environment.

| Package / directory | Responsibility |
| --- | --- |
| `uma_bringup` | Demo launches, CAN scripts and robot profiles |
| `uma_ros` | C++ / Python handle bridges |
| `uma_msgs` | Handle messages and control service definitions |
| `uma_kinematics/agx_arm_ctrl` | Profile loading, IK and AGX example drivers |
| `uma_kinematics/arm_safety_monitor` | Offline RViz safety-space editor |
| `uma_description` | Example URDFs, meshes and RViz configuration |
| `../UMA_SDK` | Independent repository for the handle CAN protocol |

</details>

<details>
<summary><strong>CAN configuration</strong></summary>

Run these commands from the workspace root, one step at a time:

1. Connect one USB-CAN adapter and run `find_all_can_port.sh`. Record its device and `bus-info`. Repeat for each adapter.
2. Enter those values in `can_config.sh`.
3. Run `can_config.sh`, then check the interface names and bitrate.

```bash
bash src/UMA_Control/uma_bringup/scripts/find_all_can_port.sh
bash src/UMA_Control/uma_bringup/scripts/can_config.sh
ip -details link show type can
```

| Interface | Device |
| --- | --- |
| `left_uma` / `right_uma` | Left / right teaching handle |
| `left_arm` / `right_arm` | Left / right example arm driver |

A single-arm setup needs one pair, such as `right_uma + right_arm`. The current `can_config.sh` enables all four interfaces; comment out unused devices for a single arm. USB addresses depend on the computer. Keep the bitrate at 1 Mbps.

The optional `uma_bringup/udev/99-uma-can.rules` brings matching adapters up at 1 Mbps on insertion. Use `can_config.sh` to assign the interface names above.

</details>

<details>
<summary><strong>Start and operate teleoperation</strong></summary>

For upright Piper or Nero mounting (`horizontal`):

1. Set `installation_pos: horizontal`. Use `side` to select the ROS route and the port arguments to select CAN interfaces. The arm's `base_link` defines +X forward and +Z up.
2. Place the handle level in its prescribed zero pose before starting the bridge or resetting its IMU. The bridge uses this pose as its orientation reference. Level the handle manually; the bridge does not detect gravity for this reference.
3. **Before each enable, hold the handle level and point it forward, then double-click the inner key.** That horizontal direction maps to arm +X; left maps to +Y and up to +Z. No arm-to-handle yaw measurement is needed. If the handle points nearly vertically, enabling is rejected. Level it and retry.

The mapping stays fixed until teleoperation is disabled. To change the forward direction, disable teleoperation, reposition the handle and enable it again. Motion starts from the arm's current pose.

Keep the handle base stable throughout the session; base movement is not compensated. Never reset the IMU during active teleoperation.

Choose one startup command for the arm and bridge backend. The `start_*.sh` scripts activate `uma` automatically. For Anaconda or a custom installation, set `CONDA_BASE` as described in [Conda path troubleshooting](#conda-path-en).

```bash
# Nero dual arm
bash src/UMA_Control/uma_bringup/scripts/start_dual_nero_cpp.sh
bash src/UMA_Control/uma_bringup/scripts/start_dual_nero_python.sh

# Piper single arm (horizontal mounting); the right route is the default
bash src/UMA_Control/uma_bringup/scripts/start_single_piper_cpp.sh
bash src/UMA_Control/uma_bringup/scripts/start_single_piper_python.sh

# Piper X single arm
bash src/UMA_Control/uma_bringup/scripts/start_single_piper_x_cpp.sh
bash src/UMA_Control/uma_bringup/scripts/start_single_piper_x_python.sh

# Piper X dual arm, both mounted upright
bash src/UMA_Control/uma_bringup/scripts/start_dual_piper_x_cpp.sh

# Left CAN pair / 左侧 CAN 接口
SIDE=left bash src/UMA_Control/uma_bringup/scripts/start_single_piper_cpp.sh \
  arm_port:=left_arm port_name:=left_uma
```

`SIDE` selects the single-arm ROS route. With left-side CAN names, also pass `arm_port:=left_arm port_name:=left_uma`; changing `SIDE` alone keeps the demo's right-side port defaults. Choose one bridge backend per interface.

The single-arm Piper X scripts select `arm_type:=piper_x` and `piper_x_profile.yaml`. They use the same `SIDE`, `CONDA_BASE`, `CONDA_ENV` and port arguments as the Piper scripts. For a direct launch, run `ros2 launch uma_bringup demo_piper.launch.py arm_type:=piper_x`.

The dual-arm script connects the arms through `left_arm` and `right_arm`, and the handles through `left_uma` and `right_uma`. Both sides use `piper_x_profile.yaml` with upright mounting (`installation_pos: horizontal`). Override the interfaces with `left_arm_port`, `right_arm_port`, `left_port_name` and `right_port_name`. A custom `profile_path` must specify upright mounting and the workspace for each side.

| Script setting | Default / use |
| --- | --- |
| `CONDA_BASE` | `~/miniconda3`; Conda installation root |
| `CONDA_ENV` | `uma`; environment name to activate |
| `WORKSPACE_DIR` | Auto-detected directory containing `install/setup.bash` |
| `SIDE` | Piper / Piper X single arm: `right` or `left` |
| `LAUNCH_FILE` | Model's demo launch in `uma_bringup` |
| Trailing `key:=value` | Forwarded to `ros2 launch` |

To use a custom profile, append `profile_path:=/absolute/path/my_profile.yaml` to the start command.

Scripts load Conda, ROS and the workspace. They clean up matching processes at startup and exit, including Ctrl+C. This also stops separately launched instances with matching process names. Use direct launches when debugging parallel instances.

For direct `ros2 launch`, prepare each terminal first:

```bash
cd ~/uma
CONDA_BASE="${CONDA_BASE:-$HOME/miniconda3}"
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate uma
source src/UMA_Control/platform.sh
WORKSPACE_DIR="$PWD"
uma_source_environment
export PYTHONNOUSERSITE=1
```

Then choose one entry point:

```bash
ros2 launch uma_bringup demo_nero_bringup.launch.py backend:=cpp
ros2 launch uma_bringup demo_nero_bringup.launch.py backend:=python
ros2 launch uma_bringup demo_piper.launch.py
ros2 launch uma_bringup demo_piper.launch.py arm_type:=piper_x
ros2 launch uma_bringup demo_dual_piper_x.launch.py
ros2 launch uma_bringup demo_dual_piper_x.launch.py backend:=python
ros2 launch uma_bringup demo_piper.launch.py side:=left backend:=python \
  arm_port:=left_arm port_name:=left_uma
```

Set `start_arm:=false` when using an external arm driver. This only disables the repository's example driver.

<a id="handle-controls-en"></a>

| Control | Operation | Condition | Result |
| --- | --- | --- | --- |
| Inner key: left handle K2 / right handle K1 | Double-click | Fresh arm feedback is available before enabling; follow the mounting instructions above | Toggle teleoperation for that arm |
| Inner key: left handle K2 / right handle K1 | Long-press | Teleoperation is off | Initialize the handle and home the arm |
| Outer key: left handle K1 / right handle K2 | Double-click | `recording_control` is running; collection requires a connected web collector | Toggle the shared recording request for the whole session |
| Trigger | Press or release | The configuration supports a gripper | Map trigger pressure to gripper width |

**Homing sends motion commands. Clear the arm's path before a long press.** The demo configures `double_press_timeout` as 0.30 s and `long_press_timeout` as 2.0 s. Single clicks, long presses and releases of the outer key leave the recording request unchanged.

</details>

<details>
<summary><strong>Profiles and installation geometry</strong></summary>

Example profiles are stored in `uma_bringup/config/`:

| Profile | Default use |
| --- | --- |
| `nero_profile.yaml` | Side-mounted Nero dual arm |
| `piper_profile.yaml` | Upright Piper single arm |
| `piper_x_profile.yaml` | Upright Piper X single arm, with its own URDF |

The Piper X profile has six arm joints and three end-effector channels. It defaults to `agx_gripper`, `firmware_version: default` and `zoom_factor: 1.25`. Set the firmware version for the connected arm and calibrate the workspace Box for the installation. The model and meshes are included under `uma_description/urdf/piper_x/`.

`side: left/right` selects ROS routing; CAN ports are separate launch arguments. `installation_pos: horizontal/left/right` describes mounting. Upright Piper/Nero uses `horizontal` on either route.

`installation_pos`, `effector_type` and `arm_home_state` accept `left/right/default` entries. `safety.workspace` accepts a shared Box or those same side entries. The loader uses the selected side, then `default`; it reports an error if neither exists. Scalar end-effector settings and shared workspaces are also supported.

```yaml
robot:
  installation_pos: {left: left, right: right}
  effector_type: {left: agx_gripper, right: none}
```

`effector_type` accepts `none`, `agx_gripper` or `revo2`. The Nero profile lists both sides explicitly; Piper uses the shared form. Calibrate each Nero workspace Box for the actual installation with the safety editor. Full YAML examples are in the [adaptation guide](arm_adaptance_guide.md).

Side-mounted translation maps handle `world` axes to arm `base_link` axes:

| Installation | `world +x` | `world +y` | `world +z` |
| --- | --- | --- | --- |
| `left` | `base +y` | `base +z` | `base +x` |
| `right` | `base -y` | `base -z` | `base +x` |

For `left/right`, the URDF root must coincide with `base_frame`, as in the bundled models. The home forward-kinematics reference and orientation-increment calculation are compatible with Nero v0.1.0.

For `horizontal`, each enable maps the handle's horizontal forward direction to base +X, with +Z up. Translation and rotation increments share this mapping. The axes stay fixed during the session, including when the TCP rotates. Follow the startup steps above to set the handle zero pose.

Piper also uses `installation_pos` for controller gravity compensation.

`arm_home_state` supplies the home command and side-mount forward-kinematics reference. Each teleoperation session starts from current joint feedback.

Each list contains one arm's driver channels, with the first `nq` entries in URDF arm-joint order. Piper has 9 entries (6 arm + 3 end-effector channels); Nero has 10 per side (7 arm + 3 end-effector channels). The list length must match the driver feedback.

`firmware_version` specifies the CAN protocol variant for pyAgxArm. It must match the arm's actual firmware or CAN parsing will fail. Nero: `default` (≤1.10), `v111` (1.11), `v112` (1.12), `v120` (1.20), `v121` (≥1.21). Piper: `default` (≤S-V1.8-2), `v183` (S-V1.8-3~7), `v188` (S-V1.8-8), `v189` (≥S-V1.8-9). Check your firmware version before deployment.

| Profile block | What to configure |
| --- | --- |
| `robot` | URDF, base/tool frames, end effector, firmware version, home state and mounting |
| `ik` | Per-step joint change limit, posture and singularity penalties |
| `handle` | `zoom_factor`: Nero 1.4, Piper / Piper X 1.25 in the bundled profiles |
| `safety` | Workspace box, virtual walls and protected points |

The examples use `ik.max_delta_q_rad: 0.1` to limit joint change per IK step. Motor speed is set through the driver. The demo launches do not read `robot.speed_percent` from YAML. To set speed, launch the example driver separately with `speed_percent:=30` and use `start_arm:=false` for bringup.

See the [Arm Adaptation Guide](arm_adaptance_guide.md) for complete profile and driver examples, including custom arms and grippers.

</details>

<details>
<summary><strong>Runtime interfaces and arm integration</strong></summary>

`uma_ros` receives handle CAN data and publishes `/left_handle_pose` or `/right_handle_pose`. Pinocchio IK converts the pose into `/control/joint_states_{left,right}_raw`. While teleoperation is enabled, `uma_ros` forwards these targets to `/control/move_js_{left,right}` for the arm driver.

| Interface | Type | Purpose |
| --- | --- | --- |
| `/left_handle_pose`, `/right_handle_pose` | `geometry_msgs/msg/PoseStamped` | Handle end pose |
| `/feedback/joint_states_{left,right}` | `sensor_msgs/msg/JointState` | Live arm feedback |
| `/control/joint_states_{left,right}_raw` | `sensor_msgs/msg/JointState` | Raw IK target |
| `/control/move_js_{left,right}` | `sensor_msgs/msg/JointState` | Target forwarded to the driver |
| `/left_teleop_enable`, `/right_teleop_enable` | `std_msgs/msg/Bool` | Teleoperation state |
| `/left_teleop_trigger`, `/right_teleop_trigger` | `std_srvs/srv/Trigger` | Toggle teleoperation |
| `/recording/toggle` | `std_msgs/msg/Empty` | Either outer key requests one global recording toggle |
| `/recording/requested` | `std_msgs/msg/Bool` | Global collection request, initially false; transient-local latest state |
| `/recording/set_requested` | `std_srvs/srv/SetBool` | Set or clear the global collection request from the web collector |

An external driver must publish `/feedback/joint_states_{side}` and subscribe to `/control/move_js_{side}`. Joint values use radians. Joint naming, direction, zero offsets, enabling, grippers, and vendor transport belong to the driver.

Fresh joint feedback is required before teleoperation starts. A feedback timeout stops the session and clears its installation mapping. See [handle controls](#handle-controls-en) for toggling teleoperation and homing.

Both outer keys control one shared recording request. Both C++ and Python bridges publish toggle events to one `recording_control` node, which publishes `/recording/requested`. The web collector subscribes to that flag and starts or stops its collection launch. It can correct the request through `/recording/set_requested` if collection fails.

The Nero, Piper and Piper X demos start one recording-control node by default. When combining launch entries, start it only once and pass `start_recording_control:=false` to the others. For manually launched handle bridges, run `ros2 run uma_ros recording_control_node.py` once. Subscribers using transient-local durability receive the latest flag even when they join later. The web collector is responsible for running collection launches.

Demo launch arguments:

| Argument | Default | Purpose |
| --- | --- | --- |
| `backend` | `cpp` | Handle bridge implementation |
| `arm_type` | `piper` in `demo_piper.launch.py` | Single-arm model: `piper` or `piper_x`; also selects the default profile |
| `side` | `right` for Piper / Piper X | Single-arm communication route |
| `profile_path` | Example profile | URDF, IK, mounting, and safety configuration |
| `start_arm` | `true` | Start the example arm driver |
| `start_teleop` | `true` | Start the IK teleoperator |
| `start_recording_control` | `true` | Start the shared recording-control node |
| `gripper_width_min/max` | `0.00/0.07` | Gripper range in metres |
| `pub_rate` | `200.0` | Bridge publication rate in Hz |
| `log_level` | `info` | Bridge and example-driver logging |
| `arm_port`, `port_name` | `right_arm`, `right_uma` | Piper / Piper X single-arm CAN ports |
| `left_arm_port`, `right_arm_port` | `left_arm`, `right_arm` | Nero / Piper X dual-arm CAN ports |
| `left_port_name`, `right_port_name` | `left_uma`, `right_uma` | Nero / Piper X dual-arm handle CAN ports |

</details>


<details>
<summary><strong>Handle telemetry, services and link checks</strong></summary>

| Handle suffix | Type | Useful fields |
| --- | --- | --- |
| `/joint` | `sensor_msgs/msg/JointState` | Teaching-handle joints |
| `/imu` | `sensor_msgs/msg/Imu` | Orientation, angular velocity, acceleration |
| `/handle_state` | `uma_msgs/msg/HandleState` | `joystick_x/y`, `trigger_press_percentage`, key events and receive counts |
| `/mt_sensor_status` | `uma_msgs/msg/MtSensorStatus` | `name`, `data`, `receive_count`; each sensor has `angle_deg/rad`, `voltage_x/y` |

Prefix these with `/uma_l` or `/uma_r`, for example `/uma_r/handle_state`. After connecting CAN, inspect receive counts and topic rates to confirm that data is arriving.

| Service suffix | Type | Action |
| --- | --- | --- |
| `/reset_pose` | `std_srvs/srv/Empty` | Recapture the handle pose reference |
| `/reset_control` | `uma_msgs/srv/ResetControl` | Encoder reset: `reset_mode` 0 = zero, 1 = factory |
| `/enable_handle_motor` | `std_srvs/srv/SetBool` | Start / stop handle motor |
| `/enable_reverse` | `std_srvs/srv/SetBool` | Change handle base reversal |

These services also use `/uma_l` or `/uma_r`. The IK services `/left_teleop_trigger` and `/right_teleop_trigger` toggle teleoperation on each call. Handle-reset services update the handle reference or encoders; arm homing uses the inner-key long press.

Read-only checks for the right route (replace `right` / `uma_r` for the left):

```bash
ros2 topic hz /right_handle_pose
ros2 topic echo /uma_r/handle_state --once
ros2 topic echo /feedback/joint_states_right --once
ros2 topic echo /right_teleop_enable --once
ros2 topic echo /control/joint_states_right_raw --once
ros2 topic echo /control/move_js_right --once
ros2 service list
```

Run each check separately. Stop `ros2 topic hz` with Ctrl+C before the next command. The two motion-target topics may be quiet while teleoperation is off. Check handle pose first, then arm feedback and enable state, raw IK output, and the final command. The bridge can send homing commands while teleoperation is disabled.

</details>

<details>
<summary><strong>Safety-space editor</strong></summary>

Edit workspace and walls offline. For side-specific workspaces, select `side:=left` or `side:=right`; save and close one editor before editing the other side. Specifying a side on a shared Box creates an independent override, preserving the shared default. Walls remain shared.

```bash
ros2 launch arm_safety_monitor safety_monitor.launch.py
ros2 launch arm_safety_monitor safety_monitor.launch.py \
  profile_name:=nero_profile.yaml side:=left
```

The launch starts RViz, `robot_state_publisher`, `joint_state_publisher_gui` and the editor. It loads the URDF from the profile; no arm or CAN feedback is needed. InteractiveMarkers uses `/arm_safety/update`.

1. Set RViz **Fixed Frame** to `robot.base_frame` (normally `base_link`). Move the joint GUI sliders to inspect arm poses.
2. Right-click the green base marker to create a Box or Plane. Existing objects have the same editing menu.
3. Drag a Box to move its centre; use its menu to resize or delete it.
4. Move a Plane and adjust its green normal arrow. The wall menu can flip the normal. Check `keep_side` and `protected_frames` in the profile.
5. Choose **Save profile**, then restart the teleoperation node to apply it.

Saving atomically updates the `safety` block and preserves other blocks and their comments. Source profiles are preferred; installed package data is the fallback. Use `UMA_PROFILE_DIR` to choose a writable directory. The Box menu step is controlled by `box_resize_step` (0.05 m); `plane_size` (0.8 m) changes only the displayed wall size. The constraint is an infinite plane.

Constraints protect the origins listed in `protected_frames`. Coverage is limited to those points. If the profile uses `tool0` and has no generated `ee` frame, change the editor's new-object default `protected_frames: [ee]` to the actual frame before using the profile.

</details>

<details>
<summary><strong>Troubleshooting</strong></summary>

<a id="conda-path-en"></a>

### Conda installation path or environment cannot be found

If a startup script reports `找不到 Conda: /home/xxx/miniconda3`, check the actual installation directory.

1. Check that Anaconda has its activation script and list its environments:

   ```bash
   ls "$HOME/anaconda3/etc/profile.d/conda.sh"
   "$HOME/anaconda3/bin/conda" env list
   ```

   If the file is missing, use the actual installation path. `CONDA_BASE` must be the directory containing `etc/profile.d/conda.sh`. `CONDA_ENV` is the environment name shown by `conda env list`.

2. Start from the workspace root with that installation:

   ```bash
   cd ~/uma
   CONDA_BASE="$HOME/anaconda3" bash src/UMA_Control/uma_bringup/scripts/start_dual_nero_cpp.sh
   ```

   This activates `uma`. To use another existing environment, replace `actual_environment_name` below with its name:

   ```bash
   CONDA_BASE="$HOME/anaconda3" CONDA_ENV="actual_environment_name" \
     bash src/UMA_Control/uma_bringup/scripts/start_dual_nero_cpp.sh
   ```

   The same variables work with all startup scripts. Use the script for the intended arm and backend. If activation reports a missing environment, check the name against the output of `conda env list`.

3. To use the installation in future terminals, add this line to `~/.bashrc`, or update the existing `CONDA_BASE` assignment:

   ```bash
   export CONDA_BASE="$HOME/anaconda3"
   ```

   For a custom environment name, also add `export CONDA_ENV="actual_environment_name"`. Load the settings in the current terminal:

   ```bash
   source ~/.bashrc
   ```

### Motion and communication checks

Before connecting motion output, inspect the IK target in the visualization. Then test at low hardware speed. Check all three translation axes, the initial pose for a jump, orientation behavior and Piper gravity compensation.

| Symptom | Check |
| --- | --- |
| No CAN data | Match `bus-info` to the connected device. Check 1 Mbps bitrate and interface state with `ip -details link show type can`. |
| No motion | Check the teleoperation state, live feedback, raw IK target and `move_js` topics in that order. |
| Session stops immediately | Check feedback frequency and `feedback_timeout_sec`. |
| Upright x/z are swapped | Check `installation_pos: horizontal` and confirm the terminal loaded the intended workspace. |
| IK is infeasible | Check URDF frames, joint limits, workspace, walls, collision pairs and `zoom_factor`. |
| RViz has missing TF | Check the URDF, `base_frame`, state-publisher nodes and RViz Fixed Frame. |

For `Infeasible_Problem_Detected`, confirm the loaded profile and URDF frames first. Check joint limits, Box bounds and margin, wall normals, `keep_side` and protected points. Inspect `wall_dist` in the logs. Reduce handle scaling and restart from a reachable pose. `collision_pairs` performs self-collision checks after solving; Opti does not enforce these pairs as hard constraints.

For import failures, inspect `which python3` and rerun the dependency checks in the `uma` environment. Rebuild and source `install/setup.bash` if ROS packages or updated launch files are missing. Use hardware speed limits and the arm's own stop mechanism during commissioning.

</details>

<a id="中文"></a>
## 中文

<details>
<summary><strong>工作空间、依赖与构建</strong></summary>

创建 ROS 2 工作空间 `~/uma`，将 `UMA_Control` 和 `UMA_SDK` 两个独立仓库放入 `~/uma/src`：

```bash
mkdir -p ~/uma/src
cd ~/uma/src
git clone https://github.com/agilexrobotics/UMA_Control.git
git clone https://github.com/agilexrobotics/UMA_SDK.git
cd ~/uma
bash src/UMA_Control/build.sh
```

根据脚本提示选择构建模式：

- **完整构建**：安装依赖、构建工作空间并校验，过程中需要选择桥接实现。
- **重新构建**：删除 `build/ install/ log/`，重新构建并校验。

C++ 和 Python 桥接都使用 Python 逆运动学（IK），因此都需要 `uma` Conda 环境。选择 `python` 时还会安装 `python-can` 和 Python SDK。两种选择都会通过 colcon 构建 C++ 包。

| Ubuntu | 原生 ROS 2 | Conda Python |
| --- | --- | --- |
| 22.04 | Humble | 3.10 |
| 24.04 | Jazzy | 3.12 |

运行 `build.sh` 前，按 [ROS 官方说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)安装对应发行版。构建和启动脚本根据当前运行环境内的系统版本选择 ROS 与 Python。Ubuntu 24.04 原生使用 Jazzy，IK 继续使用 Pinocchio 3.2.0 和 CasADi 3.6.7。

迁移到 24.04 时，在新工作空间中放入两个源码仓库后构建。构建产物包含绝对路径及对应 ROS/Python 版本的扩展模块。如果现有 `uma` 环境使用 Python 3.10，可以指定新的环境名称：

```bash
CONDA_ENV=uma_jazzy bash src/UMA_Control/build.sh
CONDA_ENV=uma_jazzy bash src/UMA_Control/uma_bringup/scripts/start_single_piper_cpp.sh
```

构建时选择“完整构建”，后续启动使用相同的 `CONDA_ENV`。Python 版本或工作区平台不匹配时，脚本会在启动节点前报错。迁移已有工作空间时，先准备好匹配的 Conda 环境，再选择“重新构建”，重新生成 `build/ install/ log/`。

| 包 / 目录 | 用途 |
| --- | --- |
| `uma_bringup` | 示例启动文件、CAN 脚本和机械臂配置文件（profile） |
| `uma_ros` | C++ / Python 示教器桥接节点 |
| `uma_msgs` | 手柄消息和控制服务定义 |
| `uma_kinematics/agx_arm_ctrl` | Profile 加载、IK 和 AGX 示例驱动 |
| `uma_kinematics/arm_safety_monitor` | RViz 离线安全空间编辑器 |
| `uma_description` | 示例 URDF、模型网格和 RViz 配置 |
| `../UMA_SDK` | 独立仓库，提供示教器 CAN 协议 |

</details>

<details>
<summary><strong>CAN 配置</strong></summary>

在工作空间根目录依次操作：

1. 接入一个 USB-CAN 适配器，执行 `find_all_can_port.sh`，记录对应设备和 `bus-info`。对每个适配器重复此操作。
2. 将记录的 `bus-info` 填入 `can_config.sh`。
3. 执行 `can_config.sh`，检查接口名称和通信速率。

```bash
bash src/UMA_Control/uma_bringup/scripts/find_all_can_port.sh
bash src/UMA_Control/uma_bringup/scripts/can_config.sh
ip -details link show type can
```

| 接口名 | 设备 |
| --- | --- |
| `left_uma` / `right_uma` | 左 / 右示教器 |
| `left_arm` / `right_arm` | 左 / 右示例机械臂驱动 |

可选的 `uma_bringup/udev/99-uma-can.rules` 会在插入匹配设备时以 1 Mbps 启动接口。上述接口名称由 `can_config.sh` 分配。

</details>

<details>
<summary><strong>启动与手柄操作</strong></summary>

Piper 或 Nero 正装时，按以下顺序准备：

1. 设置 `installation_pos: horizontal`。用 `side` 选择 ROS 路由，用端口参数指定 CAN 接口，例如 `right_arm` 和 `right_uma`。机械臂 `base_link` 的 +X 为前方，+Z 为上方。
2. 启动桥接节点或重置 IMU 前，将手柄按规定零位放平。桥接节点以此建立姿态参考。手柄需要手动放平，节点不会自动识别重力方向。
3. **每次开启遥操前，将手柄放平并指向目标前方，再双击内侧按键。**该水平朝向映射为 +X，左方为 +Y，上方为 +Z。手柄接近竖直时会拒绝开启，需要放平后重试。

一次遥操期间，方向映射保持固定。需要更换前方朝向时，关闭遥操、调整手柄朝向，再重新开启。机械臂从当前位姿继续运动。

遥操期间保持示教器底座稳定，并且不要重置 IMU。软件不补偿底座移动。

按机械臂型号和桥接实现选择一条启动命令。`start_*.sh` 都会自动激活 `uma` Conda 环境。使用 Anaconda 或自定义安装目录时，按[Conda 路径排障](#conda-path-zh)设置 `CONDA_BASE`。

```bash
# Nero 双臂
bash src/UMA_Control/uma_bringup/scripts/start_dual_nero_cpp.sh
bash src/UMA_Control/uma_bringup/scripts/start_dual_nero_python.sh

# Piper 单臂
bash src/UMA_Control/uma_bringup/scripts/start_single_piper_cpp.sh
bash src/UMA_Control/uma_bringup/scripts/start_single_piper_python.sh

# Piper X 单臂
bash src/UMA_Control/uma_bringup/scripts/start_single_piper_x_cpp.sh
bash src/UMA_Control/uma_bringup/scripts/start_single_piper_x_python.sh

# Piper X 双臂，两侧均正装
bash src/UMA_Control/uma_bringup/scripts/start_dual_piper_x_cpp.sh

# Left CAN pair / 左侧 CAN 接口
SIDE=left bash src/UMA_Control/uma_bringup/scripts/start_single_piper_cpp.sh \
  arm_port:=left_arm port_name:=left_uma
```

Piper X 单臂脚本会选择 `arm_type:=piper_x` 和 `piper_x_profile.yaml`，并支持 `SIDE`、`CONDA_BASE`、`CONDA_ENV` 及 CAN 端口参数。使用左侧接口时，设置 `SIDE=left`，同时传入 `arm_port:=left_arm port_name:=left_uma`。手动启动可执行 `ros2 launch uma_bringup demo_piper.launch.py arm_type:=piper_x`。

双臂脚本使用 `left_arm`、`right_arm` 连接机械臂，使用 `left_uma`、`right_uma` 连接示教器。两侧默认加载正装配置 `piper_x_profile.yaml`（`installation_pos: horizontal`）。可通过 `left_arm_port`、`right_arm_port`、`left_port_name`、`right_port_name` 修改接口名称；自定义 `profile_path` 时，请为两侧设置正装方向和各自的工作空间。

手动执行 `ros2 launch` 前，每个终端先加载环境：

```bash
cd ~/uma
CONDA_BASE="${CONDA_BASE:-$HOME/miniconda3}"
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate uma
source src/UMA_Control/platform.sh
WORKSPACE_DIR="$PWD"
uma_source_environment
export PYTHONNOUSERSITE=1
```

再选择一个入口：

```bash
ros2 launch uma_bringup demo_nero_bringup.launch.py backend:=cpp
ros2 launch uma_bringup demo_nero_bringup.launch.py backend:=python
ros2 launch uma_bringup demo_piper.launch.py
ros2 launch uma_bringup demo_piper.launch.py arm_type:=piper_x
ros2 launch uma_bringup demo_dual_piper_x.launch.py
ros2 launch uma_bringup demo_dual_piper_x.launch.py backend:=python
ros2 launch uma_bringup demo_piper.launch.py side:=left backend:=python \
  arm_port:=left_arm port_name:=left_uma
```

使用自有机械臂驱动时传入 `start_arm:=false`，该参数只关闭仓库示例驱动。

<a id="handle-controls-zh"></a>

| 控件 | 操作 | 生效条件 | 执行结果 |
| --- | --- | --- | --- |
| 内侧按键：左手柄 K2 / 右手柄 K1 | 双击 | 开启前有实时机械臂反馈，并完成上方安装与上电准备 | 切换对应机械臂的遥操状态 |
| 内侧按键：左手柄 K2 / 右手柄 K1 | 长按 | 遥操已关闭 | 初始化示教器，并让机械臂回零 |
| 外侧按键：左手柄 K1 / 右手柄 K2 | 双击 | `recording_control` 正在运行；实际采集还需 Web 采集端接入 | 切换整个系统共用的采集请求 |
| 扳机 | 按压或释放 | 当前配置支持夹爪 | 根据扳机按压量控制夹爪宽度 |

**回零会发送运动命令，长按前请留出机械臂运动空间。**

</details>

<details>
<summary><strong>Profile 和安装方向</strong></summary>

示例 profile 位于 `uma_bringup/config/`：

| Profile | 默认用途 |
| --- | --- |
| `nero_profile.yaml` | Nero 左右侧装双臂 |
| `piper_profile.yaml` | Piper 正装单臂 |
| `piper_x_profile.yaml` | Piper X 正装单臂，使用独立 URDF |

Piper X 配置包含 6 个机械臂关节和 3 个末端通道，默认使用 `agx_gripper`、`firmware_version: default` 和 `zoom_factor: 1.25`。部署时按实际机械臂设置固件版本，并标定工作空间 Box。模型和网格文件位于 `uma_description/urdf/piper_x/`。

`side: left/right` 选择 ROS 路由，CAN 端口通过启动参数单独指定。`installation_pos: horizontal/left/right` 描述安装方向。正装 Piper/Nero 使用 `horizontal`，左右路由均可。

`installation_pos`、`effector_type`、`arm_home_state` 支持 `left/right/default` 配置项。`safety.workspace` 支持共享 Box，也支持按这三个配置项分别设置。加载时优先读取当前侧，再读取 `default`；两者都缺失时会报错。也支持字符串形式的末端类型和共享工作空间。

```yaml
robot:
  installation_pos: {left: left, right: right}
  effector_type: {left: agx_gripper, right: none}
```

`effector_type` 可选 `none`、`agx_gripper`、`revo2`。Nero 示例分别配置左右两侧，Piper 使用共享配置。Nero 左右工作空间 Box 需要按实际安装分别标定，可使用安全空间编辑器调整。完整 YAML 示例见[适配指南](arm_adaptance_guide.md)。

侧装时，示教器 `world` 坐标轴与机械臂 `base_link` 坐标轴的平移映射如下：

| 安装方向 | `world +x` | `world +y` | `world +z` |
| --- | --- | --- | --- |
| `left` | `base +y` | `base +z` | `base +x` |
| `right` | `base -y` | `base -z` | `base +x` |

`left/right` 要求 URDF 根坐标系与 `base_frame` 重合，仓库示例满足此条件。回零姿态的正运动学参考和姿态增量计算兼容 Nero v0.1.0。

`horizontal` 每次开启时，将手柄水平前向映射到机械臂基座 +X，并保持 +Z 向上。平移和旋转增量共用该映射。一次遥操期间，映射保持固定，TCP 旋转时操作坐标轴也保持固定。手柄零位要求见上方启动步骤。

Piper 控制器还使用 `installation_pos` 设置重力补偿方向。

`arm_home_state` 用于机械臂回零和侧装时的正运动学参考。每次遥操的起点来自实时关节反馈。

每个列表填写一侧的驱动通道，前 `nq` 项按 URDF 机械臂关节顺序排列。Piper 示例为 9 项（6 个机械臂关节 + 3 个末端通道），Nero 每侧为 10 项（7 个机械臂关节 + 3 个末端通道）。列表长度必须与实际驱动反馈一致。

`firmware_version` 指定 pyAgxArm 的 CAN 协议版本，必须与机械臂实际固件一致，否则 CAN 解析失败。Nero：`default`（≤1.10）、`v111`（1.11）、`v112`（1.12）、`v120`（1.20）、`v121`（≥1.21）。Piper：`default`（≤S-V1.8-2）、`v183`（S-V1.8-3~7）、`v188`（S-V1.8-8）、`v189`（≥S-V1.8-9）。部署前请确认固件版本。

| Profile 区块 | 配置内容 |
| --- | --- |
| `robot` | URDF、基座与末端坐标系、末端执行器、固件版本、回零状态和安装方向 |
| `ik` | 单帧关节变化限制、构型和奇异性惩罚 |
| `handle` | `zoom_factor`，示例 Nero 为 1.4，Piper / Piper X 为 1.25 |
| `safety` | 工作空间 Box、虚拟墙和保护点 |

示例中的 `ik.max_delta_q_rad: 0.1` 限制 IK 每步关节变化。电机速度由驱动设置，示例启动文件不会读取 YAML 中的 `robot.speed_percent`。调速时，单独启动示例驱动并传入 `speed_percent:=30`，再用 `start_arm:=false` 启动其余节点。

完整 profile、驱动和夹爪示例见[机械臂适配指南](arm_adaptance_guide.md)。

</details>

<details>
<summary><strong>运行接口与机械臂适配</strong></summary>

`uma_ros` 接收示教器 CAN 数据，发布 `/left_handle_pose` 或 `/right_handle_pose`。Pinocchio IK 将位姿转换为 `/control/joint_states_{left,right}_raw` 关节目标。遥操开启后，`uma_ros` 将目标转发至 `/control/move_js_{left,right}`，由机械臂驱动执行。

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/left_handle_pose`, `/right_handle_pose` | `geometry_msgs/msg/PoseStamped` | 示教器末端位姿 |
| `/feedback/joint_states_{left,right}` | `sensor_msgs/msg/JointState` | 机械臂实时反馈 |
| `/control/joint_states_{left,right}_raw` | `sensor_msgs/msg/JointState` | IK 原始目标 |
| `/control/move_js_{left,right}` | `sensor_msgs/msg/JointState` | 转发给驱动的目标 |
| `/left_teleop_enable`, `/right_teleop_enable` | `std_msgs/msg/Bool` | 遥操开关状态 |
| `/left_teleop_trigger`, `/right_teleop_trigger` | `std_srvs/srv/Trigger` | 切换遥操 |
| `/recording/toggle` | `std_msgs/msg/Empty` | 任一外侧录制键请求一次全局采集切换 |
| `/recording/requested` | `std_msgs/msg/Bool` | 全局采集请求，初始 false；transient-local 保留最新标志 |
| `/recording/set_requested` | `std_srvs/srv/SetBool` | Web 采集端设置或清除全局采集请求 |

自有驱动至少需要发布 `/feedback/joint_states_{side}`，并订阅 `/control/move_js_{side}`。关节单位为 rad。关节名称、方向、零位、使能、夹爪和厂商通信由驱动负责。

开启遥操前必须收到未超时的关节反馈。反馈超时后，节点停止本次遥操并清除安装映射。切换遥操和回零的操作见[按键控制表](#handle-controls-zh)。

两个外侧按键共用一个采集请求。C++ 和 Python 桥接节点将切换事件发送给同一个 `recording_control` 节点，由该节点发布 `/recording/requested`。Web 采集端订阅此标志，负责启动或关闭整体采集；采集失败时，可通过 `/recording/set_requested` 校正请求状态。

Nero、Piper 和 Piper X 示例默认启动一个采集控制节点。组合多个启动入口时，只启动一个，其余入口传入 `start_recording_control:=false`。单独启动手柄桥接节点时，另执行一次 `ros2 run uma_ros recording_control_node.py`。使用 transient-local QoS 的订阅者稍后接入时，也能取得最新请求状态。实际采集由 Web 采集端执行。

示例启动参数：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `backend` | `cpp` | 示教器桥接实现 |
| `arm_type` | `demo_piper.launch.py` 中为 `piper` | 单臂型号，支持 `piper`、`piper_x`，同时选择默认 profile |
| `side` | Piper / Piper X 为 `right` | 单臂通信路由 |
| `profile_path` | 对应示例 profile | URDF、IK、安装方向和安全配置 |
| `start_arm` | `true` | 启动示例机械臂驱动 |
| `start_teleop` | `true` | 启动 IK 遥操节点 |
| `start_recording_control` | `true` | 启动全局采集控制节点 |
| `gripper_width_min/max` | `0.00/0.07` | 夹爪范围，单位 m |
| `pub_rate` | `200.0` | 桥接节点发布频率，单位 Hz |
| `log_level` | `info` | 桥接节点和示例驱动的日志级别 |
| `arm_port`, `port_name` | `right_arm`, `right_uma` | Piper / Piper X 单臂机械臂、示教器 CAN 端口 |
| `left_arm_port`, `right_arm_port` | `left_arm`, `right_arm` | Nero / Piper X 双臂机械臂 CAN 端口 |
| `left_port_name`, `right_port_name` | `left_uma`, `right_uma` | Nero / Piper X 双臂示教器 CAN 端口 |

</details>


<details>
<summary><strong>手柄状态、服务与链路检查</strong></summary>

| 手柄话题后缀 | 类型 | 常用字段 |
| --- | --- | --- |
| `/joint` | `sensor_msgs/msg/JointState` | 示教器关节状态 |
| `/imu` | `sensor_msgs/msg/Imu` | 姿态、角速度、加速度 |
| `/handle_state` | `uma_msgs/msg/HandleState` | `joystick_x/y`、`trigger_press_percentage`、按键事件和接收计数 |
| `/mt_sensor_status` | `uma_msgs/msg/MtSensorStatus` | `name`、`data`、`receive_count`；每个传感器含 `angle_deg/rad`、`voltage_x/y` |

话题名称需要加上 `/uma_l` 或 `/uma_r` 前缀，例如 `/uma_r/handle_state`。CAN 连接后，检查接收计数和话题频率，确认数据是否持续到达。

| 服务后缀 | 类型 | 动作 |
| --- | --- | --- |
| `/reset_pose` | `std_srvs/srv/Empty` | 重新采集示教器位姿参考 |
| `/reset_control` | `uma_msgs/srv/ResetControl` | 编码器复位，`reset_mode` 0 = 零位，1 = 出厂 |
| `/enable_handle_motor` | `std_srvs/srv/SetBool` | 启停手柄电机 |
| `/enable_reverse` | `std_srvs/srv/SetBool` | 切换手柄基座反向 |

服务名称同样需要加上 `/uma_l` 或 `/uma_r` 前缀。IK 的 `/left_teleop_trigger` 和 `/right_teleop_trigger` 每次调用都会切换遥操状态。示教器复位服务更新位姿参考或编码器；机械臂回零通过长按内侧按键触发。

右侧链路的只读检查命令；左侧替换 `right` 和 `uma_r`：

```bash
ros2 topic hz /right_handle_pose
ros2 topic echo /uma_r/handle_state --once
ros2 topic echo /feedback/joint_states_right --once
ros2 topic echo /right_teleop_enable --once
ros2 topic echo /control/joint_states_right_raw --once
ros2 topic echo /control/move_js_right --once
ros2 service list
```

逐条执行检查命令。`ros2 topic hz` 会持续运行，使用 Ctrl+C 结束后再执行下一条。遥操关闭时，两个运动目标话题可能没有数据。检查顺序为手柄位姿、机械臂反馈与遥操状态、IK 原始目标、最终运动命令。遥操关闭期间，桥接节点仍可发送回零命令。

</details>

<details>
<summary><strong>安全空间编辑器</strong></summary>

编辑器支持离线调整工作空间和虚拟墙。分别配置左右工作空间时，指定 `side:=left/right`，保存并关闭当前编辑器后再编辑另一侧。编辑共享 Box 时指定 `side`，会生成该侧的独立配置，并保留共享默认值。虚拟墙使用共享配置。

```bash
ros2 launch arm_safety_monitor safety_monitor.launch.py
ros2 launch arm_safety_monitor safety_monitor.launch.py \
  profile_name:=nero_profile.yaml side:=left
```

启动文件会运行 RViz、`robot_state_publisher`、`joint_state_publisher_gui` 和编辑器，并从 profile 加载 URDF。离线编辑不需要机械臂或 CAN 反馈。InteractiveMarkers 的 Update Topic 为 `/arm_safety/update`。

1. 将 RViz **Fixed Frame** 设为 `robot.base_frame`，通常是 `base_link`。拖动关节 GUI 滑块，检查不同姿态。
2. 右键基座处的绿色球，创建 Box 或 Plane；已有对象也可通过右键菜单编辑。
3. 拖动 Box 移动中心，通过菜单调整尺寸或删除。
4. 移动 Plane，调整绿色法向箭头；菜单支持翻转法向。结合 profile 检查 `keep_side` 和 `protected_frames`。
5. 选择**保存 profile**，重启遥操节点使约束生效。

保存时会原子更新 `safety` 区块，保留其他区块及其注释。编辑器优先写入源码中的 profile；未找到时写入安装目录。可用 `UMA_PROFILE_DIR` 指定可写目录。

`box_resize_step` 控制 Box 菜单的尺寸调整步长，默认 0.05 m。`plane_size` 默认 0.8 m，只控制墙面的显示尺寸；实际约束是无限平面。

约束只保护 `protected_frames` 中列出的坐标系原点。若 profile 使用 `tool0` 且未生成 `ee`，创建对象后需将默认的 `protected_frames: [ee]` 改为实际坐标系。

</details>

<details>
<summary><strong>排障</strong></summary>

<a id="conda-path-zh"></a>

### 找不到 Conda 安装目录或环境

启动脚本提示 `找不到 Conda: /home/xxx/miniconda3` 时，先确认实际安装目录。

1. 检查 Anaconda 的激活脚本，并列出已有环境：

   ```bash
   ls "$HOME/anaconda3/etc/profile.d/conda.sh"
   "$HOME/anaconda3/bin/conda" env list
   ```

   如果文件不存在，将路径改为实际安装目录。`CONDA_BASE` 填写包含 `etc/profile.d/conda.sh` 的安装根目录，`CONDA_ENV` 填写 `conda env list` 显示的环境名称。

2. 在工作空间根目录指定安装路径后启动：

   ```bash
   cd ~/uma
   CONDA_BASE="$HOME/anaconda3" bash src/UMA_Control/uma_bringup/scripts/start_dual_nero_cpp.sh
   ```

   此命令会激活 `uma`。环境名称不同时，将下方的 `实际环境名称` 替换为已有环境名称：

   ```bash
   CONDA_BASE="$HOME/anaconda3" CONDA_ENV="实际环境名称" \
     bash src/UMA_Control/uma_bringup/scripts/start_dual_nero_cpp.sh
   ```

   所有启动脚本都支持这两个变量，按机械臂型号和桥接实现选择脚本。如果激活时提示环境不存在，使用 `conda env list` 的输出核对名称。

3. 需要在以后打开的终端中自动使用该路径时，在 `~/.bashrc` 中添加这一行；已有 `CONDA_BASE` 配置时修改原有配置：

   ```bash
   export CONDA_BASE="$HOME/anaconda3"
   ```

   使用自定义环境名称时，同时添加 `export CONDA_ENV="实际环境名称"`。在当前终端加载配置：

   ```bash
   source ~/.bashrc
   ```

### 运动与通信检查

先在可视化中检查 IK 目标，再以硬件低速接入运动。依次检查三个平移方向、首次运动是否跳变、姿态响应和 Piper 重力补偿方向。

| 现象 | 检查项 |
| --- | --- |
| CAN 无数据 | 确认 `bus-info` 对应当前设备，用 `ip -details link show type can` 检查接口状态和 1 Mbps 通信速率。 |
| 遥操无动作 | 依次检查遥操状态、实时反馈、IK 原始目标和 `move_js` 话题。 |
| 开启后立即停止 | 检查反馈频率和 `feedback_timeout_sec`。 |
| 正装时 x/z 互换 | 确认 `installation_pos: horizontal`，并检查终端加载的工作空间是否正确。 |
| IK 无可行解 | 检查 URDF 坐标系、关节限位、工作空间、虚拟墙、碰撞对和 `zoom_factor`。 |
| RViz 缺少 TF | 检查 URDF、`base_frame`、状态发布节点和 RViz Fixed Frame。 |

遇到 `Infeasible_Problem_Detected`，先确认加载的 profile 和 URDF 坐标系。再检查关节限位、Box 范围与余量、墙面法向、`keep_side` 和保护点，并查看日志中的 `wall_dist`。降低手柄缩放比例，从可达姿态重新开启遥操。`collision_pairs` 在求解后执行自碰撞检测，Opti 不会将这些碰撞对作为硬约束。

真机调试的速度限制和停止机制由机械臂驱动及硬件提供。

</details>

<a id="验证"></a>
## 验证

按改动范围执行对应检查，并在交付说明中写明结果：

1. 文档、配置或 profile 改动：运行 `git diff --check -- README.md CHANGELOG.md arm_adaptance_guide.md`，检查 Markdown 代码块、链接、YAML 示例和命令是否完整。
2. 构建脚本或依赖改动：从 `~/uma` 运行 `bash src/UMA_Control/build.sh`，选择“完整构建”或“重新构建”，确认工作空间构建和启动校验通过。
3. 启动文件或 ROS 接口改动：按[运动与通信检查](#运动与通信检查)检查话题、服务和启动日志，确认虚拟目标可达。
4. 真机控制改动：确认急停可用，设置硬件低速，再测试平移、旋转、回零、夹爪和反馈超时。
