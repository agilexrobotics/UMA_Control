# Arm Adaptation Guide / 机械臂适配指南

[English](#english) | [中文](#中文)

Connect a new arm to `UMA_Control` with a driver, URDF/profile and a launch that includes the handle bridge. Nero/Piper examples provide reference integrations; other brands use the same ROS 2 interfaces. See [README](README.md) for environment setup, CAN configuration and handle operation.

将新机械臂接入 `UMA_Control`，需要准备驱动、URDF/profile 和包含示教器 bridge 的 启动入口。本仓库提供 Nero/Piper 示例；其他品牌通过相同的 ROS 2 接口接入。环境安装、CAN 配置和手柄操作见 [README](README.md)。

<a id="english"></a>
## English

> [!WARNING]
> This guide and its example profiles are for integration reference only; they do not certify a new arm or configuration as safe for hardware operation. Incorrect joint mapping, limits, units, mounting, communication or control settings can cause unexpected motion, collision, injury or equipment damage. Before commanding a real arm, verify the model and feedback against the hardware, provide a working emergency stop, and test small movements at low speed in a clear workspace. You are responsible for validating the complete system for your robot and application.

<details>
<summary><strong>1. Control flow and responsibilities</strong></summary>

```text
Handle CAN → uma_ros bridge → /{side}_handle_pose → Pinocchio IK
                                                        ↓
Your arm driver ← /control/move_js_{side} ← bridge ← /control/joint_states_{side}_raw
      ↓                                   ↑
/feedback/joint_states_{side} ──────────────┴────────────→ IK
```

`side` is `left` or `right`; a single arm can use either route. The profile's `installation_pos` describes physical mounting independently of that route.

| Layer | Responsibility |
| --- | --- |
| Your driver | Vendor SDK, CAN/serial/EtherCAT, enable, speed, emergency stop, hardware limits and unit conversion |
| IK | URDF kinematics, live feedback, target mapping, joint solving and configured safety constraints |
| Handle bridge | Handle data, enable button, homing and final command forwarding |
| Profile | Robot model, home state, installation, IK and safety settings |

Keep vendor communication in your own driver package. Before reusing IK, check that the bridge's joint and gripper conventions match your robot. Replacing the URDF alone does not complete an adaptation.

</details>

<details>
<summary><strong>2. ROS interfaces and joint conventions</strong></summary>

Examples use the left route; replace `left` with `right` for the right route.

| Direction | Topic | Type | Purpose |
| --- | --- | --- | --- |
| Driver → IK / bridge | `/feedback/joint_states_left` | `sensor_msgs/msg/JointState` | Required continuous joint feedback |
| Bridge → driver | `/control/move_js_left` | `sensor_msgs/msg/JointState` | Required final joint targets |
| IK → bridge | `/control/joint_states_left_raw` | `sensor_msgs/msg/JointState` | Internal IK output; not the default driver input |
| Bridge → IK | `/left_handle_pose` | `geometry_msgs/msg/PoseStamped` | Handle end pose |
| IK → bridge | `/left_teleop_enable` | `std_msgs/msg/Bool` | Current teleoperation state |

`/left_teleop_trigger` is a `std_srvs/srv/Trigger` service that toggles teleoperation. Additional TCP pose or driver status feedback is optional for the minimum IK integration.

### JointState

- `name` and `position` must have equal lengths, unique names and finite positions. Arm joint positions use radians.
- Profiles derive names from URDF revolute joints. Match driver names, directions and zeros to the URDF; do not rely on a custom YAML `joints` block to rename them.
- IK reads feedback by name and outputs commands in URDF arm order. The driver can map those names to vendor channels.
- Homing interpolates by array index. Keep feedback order stable and aligned with `arm_home_state`: the first `nq` entries are arm joints in URDF order, followed by any extra channels.
- Publish feedback continuously and implement command timeout and hardware stop handling in the driver. Stopping publications does not cancel a controller's previous motion command.

A six-joint feedback/home channel layout can be:

```text
name:     [joint1, joint2, joint3, joint4, joint5, joint6]
position: [q1,     q2,     q3,     q4,     q5,     q6    ]  # rad
```

</details>

<details>
<summary><strong>3. URDF and a minimal profile</strong></summary>

### URDF requirements

Place the URDF and meshes in your description package, for example `my_robot_description/urdf/my_robot.urdf`. The current integration expects a fixed base and single-DoF revolute arm joints. Continuous joints, joints with multiple DoFs and mobile bases need additional adaptation.

1. Ensure `base_frame` exists and URDF axes, zeros and limits match the hardware.
2. Supply an existing `ee_frame`, or create a tool frame with `ee_offset`.
3. Exclude movable non-arm joints through `locked_joints`; do not simply
truncate the IK output.
4. Install and source the ROS packages referenced by `package://`, and check
that mesh paths resolve.

### Minimal configuration

This template is for an **upright six-joint arm without gripper channels**. Replace the model path, home angles and workspace limits, then save it as `my_robot_profile.yaml`. Values below illustrate the fields.

```yaml
robot:
  name: my_robot
  urdf_path: package://my_robot_description/urdf/my_robot.urdf
  base_frame: base_link
  ee_frame: tool0
  locked_joints: []
  installation_pos: horizontal
  effector_type: none
  arm_home_state:
    default: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

ik:
  enable_hard_delta_q_constraint: true
  max_delta_q_rad: 0.1
  enable_psi_constraint: false
  enable_elbow_side_penalty: false
  singularity_weight: 0.02

handle:
  zoom_factor: 1.0

safety:
  virtual_walls: []
  workspace:
    frame: base_link
    type: box
    min: [-0.5, -0.5, 0.0]
    max: [0.5, 0.5, 0.8]
    margin: 0.02
    protected_frames: [tool0]
```

`ee_frame: tool0` requires that frame in the URDF. Otherwise remove `ee_frame` and use:

```yaml
robot:
  ee_offset:
    parent_joint: joint6
    xyz: [0.0, 0.0, 0.10]  # Relative to joint origin, m
    rpy: [0.0, 0.0, 0.0]   # rad
```

`ee_offset` creates a tool frame named `ee`; replace `tool0` with `ee` in the safety settings. Choose one tool definition. These `xyz/rpy` values describe the tool offset in the model, not installation compensation.

`effector_type` selects both enablement and type: `none`, `agx_gripper`, or `revo2`. Piper/Nero demos pass it to their example driver. Your driver must read this field or implement an equivalent mapping.


### Per-side end effectors and workspaces

Use `robot.effector_type: {left: agx_gripper, right: none}` for different end effectors. Home keys name routes (`left/right/default`), not models. Both arms in one profile share the URDF and joint layout; use separate profiles for different models.

```yaml
safety:
  workspace:
    left:
      frame: base_link
      type: box
      min: [-0.5, -0.4, 0.0]
      max: [0.5, 0.2, 0.8]
      margin: 0.02
      protected_frames: [ee]
    right:
      frame: base_link
      type: box
      min: [-0.5, -0.2, 0.0]
      max: [0.5, 0.4, 0.8]
      margin: 0.02
      protected_frames: [ee]
```

These bounds illustrate syntax, not validated robot limits. Keep `ee` for the tool tip, or select another protected frame. Each side is a complete Box; fields are not merged. A `default` Box supplies missing sides; without it, a missing side is an error. `{}` explicitly disables that side's workspace (virtual walls remain active). Old shared Box settings still apply to both arms. `side` does not mirror coordinates: each Box uses its declared frame.

Edit with `ros2 launch arm_safety_monitor safety_monitor.launch.py profile_name:=nero_profile.yaml side:=left`; use `side:=right` for the other arm. Saving or deleting a Box affects only the selected side. Editing a shared/default Box creates a side override. Save and close before opening another editor on the same file; virtual walls are shared.

### Home state

`arm_home_state` supplies bridge homing commands and the side-mount FK direction reference. Each teleoperation session starts from live joint feedback; enabling does not command the configured home pose.

Use `default` for one arm, or provide complete channels for each side. Do not combine two arms into one list:

```yaml
robot:
  arm_home_state:
    left:  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    right: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

IK selects the `side` entry, falling back to `default`. Piper and Nero bridge wrappers use the same side/default selection. Piper's example has 9 channels (6 arm + 3 end-effector); Nero has 10 per side (7 arm + 3 end-effector). These are example driver layouts, not requirements for every robot. Home lists must match the actual feedback channels.

</details>

<details>
<summary><strong>4. Upright and side mounting</strong></summary>

`robot.installation_pos` accepts `horizontal` (upright), `left`, or `right`. These definitions apply to Nero, Piper and other arms.

### Upright: align at each enable

`horizontal` uses `base_frame` +X as forward and +Z as up, independently of the current TCP orientation or `arm_home_state` rotation. URDF provides axes and transforms, but cannot infer what a user means by "forward". Confirm these model axes suit the intended operation. If the URDF root differs from `base_frame`, horizontal mode converts the live FK anchor into `base_frame`.

1. Before starting the bridge, place the handle level in its prescribed zero
pose, with its forward axis along handle `world +X`. Do the same when resetting the IMU. Its orientation is relative to that zero; resetting at an arbitrary heading does not establish a correct coordinate reference.
2. Before each double-click to enable, hold the handle roughly level and
point it in your intended forward direction. That direction's horizontal projection maps to arm +X, horizontal left to +Y, and `world +Z` to arm +Z. No measurement of arm-to-handle installation yaw is needed.
3. During the session the mapping stays fixed. Rotating the handle changes
target orientation without turning the translation axes. Disable and re-enable to select another heading, anchored at the arm's current pose.

Enabling is rejected when the handle points within 15° of vertical, or when handle pose/joint feedback is invalid or stale. Level the handle and retry. Vertical rotation is allowed after enabling. Here "vertical" means calibrated bridge `world +Z`: the feature does not estimate gravity or compensate for handle-base motion. Keep the base stable and do not reset the IMU while active.

Translation and rotation share the mapping `C` frozen at enable. `a0` and `h0` denote the initial arm and handle poses:

```text
p_target = p_a0 + C (p_h - p_h0)
R_target = C (R_h R_h0^T) C^T R_a0
```

### Side mounting: existing mapping

| Installation | Handle `world +x` | Handle `world +y` | Handle `world +z` |
| --- | --- | --- | --- |
| `left` | `base +y` | `base +z` | `base +x` |
| `right` | `base -y` | `base -z` | `base +x` |

`base` is the profile's `base_frame`. `left/right` retain the v0.1.0 Nero home-FK directions and the existing orientation-increment path. This legacy path requires the URDF root to coincide with `base_frame`, as in the bundled models.

### Configuration and driver

An upright single arm can keep the right CAN/ROS route:

```yaml
robot:
  installation_pos: horizontal
```

There is no need to set Piper's installation to `right` to correct control direction. Configure sides separately when their mounting differs:

```yaml
robot:
  installation_pos:
    left: left
    right: horizontal
```

Mappings support a `default` fallback. A missing field defaults to `horizontal`; invalid values, empty mappings, or a missing side with no default are rejected. When migrating a side-mounted custom profile from v0.1.0, set the installation explicitly.

Drivers that support installation settings must use the same value. The Piper demo sends it for controller gravity compensation. The current Nero driver has no such setting, but teleoperation mapping still applies. For your own driver, configure installation and payload through its hardware interface; working IK mapping does not imply configured gravity compensation.

</details>

<details>
<summary><strong>5. IK, speed and safety settings</strong></summary>

| Setting | Purpose / convention |
| --- | --- |
| `ik.max_delta_q_rad` | Maximum per-step joint change; scalar or `nq` values, example 0.1 rad |
| `ik.enable_hard_delta_q_constraint` | Enforce that step limit as a hard constraint |
| `ik.enable_psi_constraint` | Elbow configuration retention; requires shoulder/elbow/wrist reference frames |
| `ik.enable_joint_reference_penalty` | Soft penalty for departing from the previous IK joint state |
| `ik.enable_elbow_side_penalty` | Elbow side penalty; also requires shoulder/elbow/wrist frames |
| `ik.singularity_weight` | Singularity penalty weight, example 0.02 |
| `handle.zoom_factor` | Handle translation scale; start at 1.0 |
| `feedback_timeout_sec` | IK launch parameter, default 1.0 s; feedback timeout ends the session |

Configure IK step limits and hardware speed separately. High-level demos currently **do not read** `robot.speed_percent` from the profile. To set speed with the example driver, start it in a separate terminal:

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  arm_type:=piper can_port:=right_arm topics_end:=_right \
  installation_pos:=horizontal speed_percent:=30
```

Then start IK and bridge with `demo_piper.launch.py start_arm:=false`. The driver's `speed_percent` ranges from 0 to 100 and defaults to 100. For your own driver, use the vendor's speed interface.

### Protected points, workspace and virtual walls

`protected_frames` constrains frame origins, not entire links. Select tool, elbow and wrist points separately; use `safety_frames` for auxiliary frames:

```yaml
safety:
  safety_frames:
    elbow_point: joint3
    wrist_point:
      parent_joint: joint5
      xyz: [0.0, 0.0, 0.05]
      rpy: [0.0, 0.0, 0.0]
  virtual_walls:
    - name: table
      type: plane
      frame: base_link
      point: [0.0, 0.0, 0.0]
      normal: [0.0, 0.0, 1.0]
      keep_side: positive
      margin: 0.02
      protected_frames: [tool0, wrist_point]
```

Merge this into the existing `safety` block, retaining the desired workspace; do not duplicate top-level YAML keys. This example keeps points on the normal side of the plane, at least 0.02 m away. `keep_side: negative` selects the other side. `normal` must be nonzero and `margin` nonnegative.

Each box `min` must be below `max`, with reachable space after accounting for `margin`. Virtual walls are infinite planes; their RViz display width is not a collision boundary. `collision_pairs` performs post-solve self-collision checks, not Opti hard constraints. Verify geometry indices for your model rather than copying Nero's indices.

Edit a custom profile offline:

```bash
UMA_PROFILE_DIR=/absolute/path/to/config \
  ros2 launch arm_safety_monitor safety_monitor.launch.py \
  profile_name:=my_robot_profile.yaml
```

Drag Box/Plane markers in RViz, right-click to save, then restart the teleop node to load changes. New editor objects default to `protected_frames: [ee]`; for this guide's template, change that to the actual `tool0` frame after saving. See [README](README.md) for the full safety editor workflow.

</details>

<details>
<summary><strong>6. Launch integration: driver + IK + bridge</strong></summary>

Load Conda, ROS and the workspace as described in README. Your launch must connect all three components and supply IK and bridge with the same profile's home and scale settings.

This complete launch template uses the six-joint, no-gripper profile from section 3 and handle geometry from the current wrapper. Replace the driver package/launch names and install it as `my_robot_bringup/launch/teleop.launch.py`:

```python
from agx_arm_ctrl.robot_profile import format_arm_home_state, load_profile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include_launch(package, filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", filename])
        ),
        launch_arguments=arguments.items(),
    )


def build(context):
    side = LaunchConfiguration("side").perform(context)
    path = LaunchConfiguration("profile_path").perform(context)
    profile = load_profile(path, validate_robot=False)
    bridge_args = {
        "port_name": LaunchConfiguration("port_name"),
        "node_name": f"uma_{side[0]}",
        "uma_pose_topic": f"/{side}_handle_pose",
        "remote_joint_topic_in": f"/control/joint_states_{side}_raw",
        "remote_joint_topic_out": f"/control/move_js_{side}",
        "remote_joint_single_topic": f"/feedback/joint_states_{side}",
        "teleop_enable_topic": f"/{side}_teleop_enable",
        "remote_service_name": f"{side}_teleop_trigger",
        "arm_home_state": format_arm_home_state(profile, side),
        "angle_reverse": str(side == "right").lower(),
        "publish_sim_joint": "false",
        "is_nero": "false",
        "use_piper_mode": "false",
        "use_pose_dt_zoom": "true",
        "use_trigger_pose": "true",
        "zoom_factor": str(profile.handle.get("zoom_factor", 1.0)),
        "gripper_length": "0.0",
        "handle_seg0": "[0.0, 0.0, 0.0534, 0.0, 0.0, 0.0]",
        "handle_seg1": "[0.0, 0.0, 0.0, 1.5707963, 0.0, 3.1415926]",
        "handle_seg2": "[0.220, 0.0, 0.0, 0.0, 0.0, -1.5707963]",
        "handle_seg3": "[-0.04804, -0.19115, 0.05487, 1.5707963, 0.0, 0.0]",
    }
    return [
        include_launch("my_robot_driver", "my_robot_driver.launch.py", {
            "side": side,
            "installation_pos": profile.installation_pos_for(side),
        }),
        include_launch("agx_arm_ctrl", "teleop_single_arm.launch.py", {
            "side": side,
            "node_name": f"teleop_{side}_arm_node",
            "profile_path": path,
        }),
        include_launch("uma_bringup", "run_single_node.launch.py", bridge_args),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile_path"),
        DeclareLaunchArgument("side", default_value="left", choices=["left", "right"]),
        DeclareLaunchArgument("port_name", default_value="left_uma"),
        OpaqueFunction(function=build),
    ])
```

The template assumes your driver launch accepts `side` and `installation_pos` and configures ROS interfaces and hardware accordingly. Adapt those names to your driver. Configure its CAN/serial address there; `port_name` refers only to the teaching handle's CAN interface. If the hardware has no installation setting, use the driver's supported configuration; IK profile mapping applies independently.

```bash
ros2 launch my_robot_bringup teleop.launch.py \
  profile_path:=/absolute/path/my_robot_profile.yaml \
  side:=left port_name:=left_uma
```

For the right route, use `side:=right port_name:=right_uma`. Launch one instance per route for dual arms, with unique node names and driver interfaces. Do not start both the high-level demo's example driver and your own driver.

</details>

<details>
<summary><strong>7. Grippers and extra channels</strong></summary>

IK outputs only URDF arm joints `q`. The current bridge retains some model conventions; check seven-axis arms and extra end-effector channels separately before reusing the six-joint template.

| Existing path | Behavior | Check for a new arm |
| --- | --- | --- |
| `agx_gripper` | Keeps the arm array intact and publishes a separate gripper JointState topic | Gripper subscription, units and control fields |
| `revo2` | Initializes the hand driver; does not map one trigger to multiple fingers | Send commands through the hand-specific interface |
| Homing | Interpolates complete `arm_home_state` against feedback | Extra channel count, order, units and home meaning |

Six-axis IK does not append gripper channels. With `effector_type: agx_gripper`, the Piper/Nero bridge publishes gripper commands separately. Other drivers must implement the corresponding subscription. Set width limits in metres, with minimum no greater than maximum, within the hardware stroke.

Piper/Nero wrappers contain their handle geometry, modes and home selection. Check [bridge_launch.py](uma_bringup/uma_bringup/bridge_launch.py) and bridge output handling before reuse. Common installation settings do not imply common end-effector protocols.

</details>

<details>
<summary><strong>8. Acceptance checks and troubleshooting</strong></summary>

Validate the model, inspect data, then test slow hardware motion. Commands below use the left route.

**Load the model:** substitute your profile path and run in the prepared `uma` environment:

```bash
python - <<'PY'
from agx_arm_ctrl.robot_profile import load_profile
profile = load_profile('/absolute/path/my_robot_profile.yaml')
print('nq:', profile.nq)
print('joints:', [joint.urdf_name for joint in profile.joints])
print('installation:', profile.installation_pos_for('left'))
PY
```

**Inspect the data path:** `--once` waits for one message. Command topics may remain quiet while teleoperation is disabled.

```bash
ros2 topic hz /left_handle_pose
ros2 topic hz /feedback/joint_states_left
ros2 topic echo /feedback/joint_states_left --once
ros2 topic echo /left_teleop_enable --once
ros2 topic echo /control/joint_states_left_raw --once
ros2 topic echo /control/move_js_left --once
```

**Hardware acceptance:**

1. Before enabling motion output, verify feedback names, units, order, `nq`
and target reachability.
2. Initialize and enable as in section 4. Test small forward/left/up movements
and rotations about all three axes; check for no initial jump. For upright arms, also test re-enabling at another heading, translating after rotating the handle, and rejection of a vertical start. For side mounting, verify the section 4 table. Upright Nero and Piper use the same rule.
3. Configure low hardware speed before enabling motion. Check homing and
gripper behavior separately; homing commands motion.
4. Verify feedback loss ends the session and re-enabling after recovery
establishes a new anchor. Check the driver's own timeout and stopping behavior. Test teaching/drag mode separately where gravity settings apply.
5. Check protected points near Box/Plane boundaries. Save and restart to
confirm the intended safety configuration is loaded.

| Symptom | Check first |
| --- | --- |
| Profile fails to load | URDF path, existing frames, `locked_joints`, home list |
| `nq` or feedback mismatch | Non-arm joints, joint names, feedback length/order |
| Upright x/z still swapped | Loaded profile, `installation_pos: horizontal`, current install space |
| Wrong forward direction / request to level handle | Correct initialization zero; disable, level, point forward and re-enable |
| IK infeasible | Limits, box margins, wall normals/kept side, protected points, `wall_dist` logs |
| Stops immediately after enable | Continuous fresh feedback and `feedback_timeout_sec` |
| Raw commands but no final commands | Bridge subscriptions, enable state, timeout |
| Final commands but no motion | Driver subscription, enable, joint mapping, vendor API/hardware state |
| YAML speed change has no effect | Demo does not read `speed_percent`; use driver parameters |
| Seven-axis final joint behaves incorrectly | Check whether bridge handling treats joint seven as gripper data |

</details>

<details>
<summary><strong>9. Piper / Nero reference launches</strong></summary>

These demos start the example driver, IK and bridge by default. Choose one:

```bash
# Piper: right route; upright mounting set in piper_profile.yaml
ros2 launch uma_bringup demo_piper.launch.py

# Piper: left route with explicit left CAN ports
ros2 launch uma_bringup demo_piper.launch.py \
  side:=left arm_port:=left_arm port_name:=left_uma backend:=python

# Nero: dual arm
ros2 launch uma_bringup demo_nero_bringup.launch.py
ros2 launch uma_bringup demo_nero_bringup.launch.py backend:=python

# Compatible driver already running: start only Piper IK and bridge
ros2 launch uma_bringup demo_piper.launch.py start_arm:=false \
  profile_path:=$PWD/src/UMA_Control/uma_bringup/config/piper_profile.yaml
```

`start_arm:=false` does not configure interfaces for your driver; it must still satisfy section 2. Starting only the driver and IK omits handle poses, button handling and final command forwarding supplied by the bridge.

</details>

<a id="中文"></a>
## 中文

> [!WARNING]
> 本指南及示例 profile 仅供适配参考，不表示新机械臂或配置已通过真机安全验证。关节映射、限位、单位、安装方向、通信或控制参数错误，可能导致意外运动、碰撞、人身伤害或设备损坏。向真机发送运动指令前，请对照硬件核实模型与反馈，确保急停可用，并在空旷环境中以低速、小幅度测试。适配者须自行验证具体机械臂及应用的完整系统。

<details>
<summary><strong>1. 接入链路与职责</strong></summary>

```text
示教器 CAN → uma_ros bridge → /{side}_handle_pose → Pinocchio IK
                                                        ↓
用户机械臂驱动 ← /control/move_js_{side} ← bridge ← /control/joint_states_{side}_raw
      ↓                                   ↑
/feedback/joint_states_{side} ──────────────┴────────────→ IK
```

`side` 为 `left` 或 `right`，单臂任选一路。机械臂实际正装还是侧装，由 profile 的 `installation_pos` 决定，与通信路由分开配置。

| 层 | 负责内容 |
| --- | --- |
| 用户驱动 | 厂商 SDK、CAN/串口/EtherCAT、使能、速度、急停、硬件限位及关节单位转换 |
| IK | URDF 运动学、实时反馈、末端目标映射、关节求解及配置的安全约束 |
| 示教器 bridge | 手柄数据、按键开关、回零命令及最终关节命令转发 |
| Profile | 机械臂模型、回零状态、安装方向、IK 与安全参数 |

厂商通信代码放在自己的驱动包中即可。复用 IK 前，还需确认 bridge 的关节和夹爪 处理与该机械臂一致，不能只替换 URDF 就视为适配完成。

</details>

<details>
<summary><strong>2. ROS 接口与关节约定</strong></summary>

以左路为例；右路将 `left` 替换为 `right`。

| 方向 | Topic | 类型 | 用途 |
| --- | --- | --- | --- |
| 驱动 → IK / bridge | `/feedback/joint_states_left` | `sensor_msgs/msg/JointState` | 必需，持续发布关节反馈 |
| bridge → 驱动 | `/control/move_js_left` | `sensor_msgs/msg/JointState` | 必需，执行最终关节目标 |
| IK → bridge | `/control/joint_states_left_raw` | `sensor_msgs/msg/JointState` | 内部 IK 输出，不作为驱动默认订阅入口 |
| bridge → IK | `/left_handle_pose` | `geometry_msgs/msg/PoseStamped` | 示教器末端位姿 |
| IK → bridge | `/left_teleop_enable` | `std_msgs/msg/Bool` | 当前遥操开关状态 |

`/left_teleop_trigger` 为 `std_srvs/srv/Trigger` 服务，每次调用切换一次遥操。TCP 位姿、驱动状态等扩展反馈可以另行提供，不是 IK 接入的最小要求。

### JointState

- `name` 与 `position` 等长，名称不重复，位置为有限数值；臂关节单位为 rad。
- 当前 profile 从 URDF 的 revolute 关节推导名称。驱动应统一关节名、方向和零位，不要依赖 YAML 中自定义 `joints` 重命名。
- IK 按名称读取反馈，按 URDF 臂关节顺序输出命令。驱动可按名字映射到厂商通道。
- 回零插值按数组位置处理，反馈顺序必须稳定，且与 `arm_home_state` 顺序一致。前 `nq` 项必须是按 URDF 顺序排列的臂关节，额外通道放在后面。
- 驱动应持续发布反馈，并自行实现命令超时和硬件停止。控制层停止发布不等于 厂商控制器已取消上一条运动。

例如，一个六关节驱动的反馈和回零通道可约定为：

```text
name:     [joint1, joint2, joint3, joint4, joint5, joint6]
position: [q1,     q2,     q3,     q4,     q5,     q6    ]  # rad
```

</details>

<details>
<summary><strong>3. URDF 和最小 profile</strong></summary>

### URDF 要求

将 URDF 和 mesh 放在自己的 description 包中，例如 `my_robot_description/urdf/my_robot.urdf`。当前链路按固定基座、单自由度 revolute 臂关节使用；continuous、多自由度或移动基座模型需要额外适配。

确认以下内容：

1. `base_frame` 存在，URDF 的轴向、零位和限位与实际硬件一致。
2. 末端已有 `ee_frame`，或者通过 `ee_offset` 创建工具 frame。
3. 通过 `locked_joints` 排除模型中的非臂可动关节；不要只截断 IK 输出数组。
4. `package://` 资源所在的 ROS 包已经安装并加载，mesh 路径可解析。

### 最小配置

以下为**六关节、无夹爪通道、正装**机械臂的模板。替换模型路径、回零角度和空间 边界后保存为 `my_robot_profile.yaml`；示例数值只用于说明字段。

```yaml
robot:
  name: my_robot
  urdf_path: package://my_robot_description/urdf/my_robot.urdf
  base_frame: base_link
  ee_frame: tool0
  locked_joints: []
  installation_pos: horizontal
  effector_type: none
  arm_home_state:
    default: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

ik:
  enable_hard_delta_q_constraint: true
  max_delta_q_rad: 0.1
  enable_psi_constraint: false
  enable_elbow_side_penalty: false
  singularity_weight: 0.02

handle:
  zoom_factor: 1.0

safety:
  virtual_walls: []
  workspace:
    frame: base_link
    type: box
    min: [-0.5, -0.5, 0.0]
    max: [0.5, 0.5, 0.8]
    margin: 0.02
    protected_frames: [tool0]
```

`ee_frame: tool0` 要求 URDF 中确有该 frame。若没有，可删除 `ee_frame`，改用：

```yaml
robot:
  ee_offset:
    parent_joint: joint6
    xyz: [0.0, 0.0, 0.10]  # 相对关节原点，m
    rpy: [0.0, 0.0, 0.0]   # rad
```

`ee_offset` 会创建名为 `ee` 的工具 frame，此时将安全配置中的 `tool0` 改为 `ee`。二者选择一种即可；`xyz/rpy` 是模型工具偏移，不是安装方向补偿。

`effector_type` 同时作为开关和类型选择：`none` 关闭末端执行器，`agx_gripper` 启用夹爪，`revo2` 启用灵巧手。Piper/Nero demo 会把该值传给示例 驱动；自有驱动需要读取该字段或做等价映射。


### 分侧末端执行器与工作空间

末端类型可写为 `robot.effector_type: {left: agx_gripper, right: none}`。回零键名使用通信路由 `left/right/default`，不使用机型名称。同一 profile 的两侧共用 URDF 和关节结构；不同机型使用独立 profile。

```yaml
safety:
  workspace:
    left:
      frame: base_link
      type: box
      min: [-0.5, -0.4, 0.0]
      max: [0.5, 0.2, 0.8]
      margin: 0.02
      protected_frames: [ee]
    right:
      frame: base_link
      type: box
      min: [-0.5, -0.2, 0.0]
      max: [0.5, 0.4, 0.8]
      margin: 0.02
      protected_frames: [ee]
```


以上边界仅示范语法，需按硬件标定；`ee` 可继续保护工具尖端，也可改为其他保护点。每侧是完整 Box，不逐字段合并；缺少当前侧时回退到 `default`，无回退则报错。`{}` 显式关闭该侧 workspace，虚拟墙仍生效。旧的共享 Box 继续作用于两侧。`side` 不会自动镜像坐标，每个 Box 使用其声明的 frame。

通过 `ros2 launch arm_safety_monitor safety_monitor.launch.py profile_name:=nero_profile.yaml side:=left` 编辑左侧，右侧改为 `side:=right`。保存、删除只影响所选侧；编辑共享或 default Box 会生成侧别覆盖项。保存并关闭后再打开另一侧编辑器，避免同时保存同一文件；虚拟墙仍为共享配置。

### 回零状态

`arm_home_state` 用于 bridge 回零命令和侧装的方向 FK 参考。本次遥操的起点 来自开启时的实时关节反馈，不会直接跳到 profile 的回零位置。

单臂可用 `default`；双臂分别填写一侧的完整通道，不能把双臂通道合并：

```yaml
robot:
  arm_home_state:
    left:  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    right: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

IK 按 `side` 取值，缺少对应项时回退到 `default`。Piper 和 Nero bridge wrapper 使用相同的 side/default 规则。Piper 示例为 9 项（6 个 臂关节 + 3 个末端通道），Nero 每侧为 10 项（7 个臂关节 + 3 个末端通道）；这些 是示例驱动配置，不是所有机器人都应使用的长度。回零列表长度需与实际反馈通道数 一致。

</details>

<details>
<summary><strong>4. 正装与侧装：统一配置</strong></summary>

`robot.installation_pos` 支持 `horizontal`（正装）、`left`（左侧装）、`right` （右侧装）。这些定义适用于 Nero、Piper 和其他机械臂。

### 正装：每次开启时对齐

`horizontal` 以 `base_frame` 的 +X 为前、+Z 为上，不依赖当前 TCP 朝向或 `arm_home_state` 旋转。URDF 能给出坐标轴与变换，但不能推断用户所说的“前方”；适配时需确认模型的这些轴符合机械臂操作方向。根坐标与 `base_frame` 不重合时，正装起点 FK 会先转换到 `base_frame`。

1. 安装并启动前，将手柄按规定零位放平，使其前向与示教器 `world +X` 一致，
再启动 bridge；重置 IMU 时同样操作。IMU 姿态是相对零位，任意朝向复位不等于 已建立正确的坐标参考。
2. 每次双击开启前，将手柄大致放平，指向希望的操作前方。该方向的水平投影映射
为机械臂 +X，水平左方为 +Y，`world +Z` 为机械臂 +Z，无需测量两者安装偏航角。
3. 开启后映射固定。转动手柄只改变目标姿态，不转动平移轴；关闭后重新开启会
重取朝向，并以机械臂当前位姿为起点。

开启时，手柄前向与竖直方向夹角不大于 15° 会被拒绝，提示放平后重试；手柄 位姿和关节反馈也必须有效、新鲜。开启后允许转到竖直姿态。这里的“竖直”基于 桥接已标定的 `world +Z`，不自动测量重力，也不补偿示教器底座移动。会话中不要 移动底座或重置 IMU。

平移和旋转共用开启时固定的映射 `C`，起点分别为机械臂 `a0` 和手柄 `h0`：

```text
p_target = p_a0 + C (p_h - p_h0)
R_target = C (R_h R_h0^T) C^T R_a0
```

### 侧装：保留原有映射

| 安装方向 | 示教器 `world +x` | 示教器 `world +y` | 示教器 `world +z` |
| --- | --- | --- | --- |
| `left` | `base +y` | `base +z` | `base +x` |
| `right` | `base -y` | `base -z` | `base +x` |

表中的 `base` 指 profile 的 `base_frame`。`left/right` 保留 v0.1.0 Nero 双臂 home FK 对应的侧装方向，姿态增量沿用现有路径。该旧路径要求 URDF 根坐标与 `base_frame` 重合，仓库示例满足此条件。

### 配置与驱动

正装单臂可继续使用右路 CAN 和 ROS 接口，只需：

```yaml
robot:
  installation_pos: horizontal
```

无需把 Piper 的安装参数改成 `right` 来补偿遥操方向。两臂安装不同时按路配置：

```yaml
robot:
  installation_pos:
    left: left
    right: horizontal
```

映射支持 `default` 作为未列出路由的回退值。字段缺失时默认为 `horizontal`；非法值、空映射，或当前路由无对应项也无 `default`，会被拒绝。从 v0.1.0 迁移 侧装自定义 profile 时，应显式填写安装方向。

支持安装配置的硬件驱动还需使用同一个值。Piper demo 会自动下发，用于控制器的 重力补偿；当前 Nero 驱动不提供该设置接口，但遥操平移映射仍生效。自有驱动的 安装与负载参数由用户下发，不能把 IK 映射生效等同于已配置控制器重力补偿。

</details>

<details>
<summary><strong>5. IK、速度和安全参数</strong></summary>

| 配置 | 用途 / 约定 |
| --- | --- |
| `ik.max_delta_q_rad` | 每步最大关节变化，标量或 `nq` 项列表；示例为 0.1 rad |
| `ik.enable_hard_delta_q_constraint` | 是否将单步变化作为硬约束 |
| `ik.enable_psi_constraint` | 肘部构型保持；需配置肩、肘、腕参考 frame |
| `ik.enable_joint_reference_penalty` | 对偏离上一轮 IK 关节状态施加软惩罚 |
| `ik.enable_elbow_side_penalty` | 肘部侧向惩罚；同样需要肩、肘、腕 frame |
| `ik.singularity_weight` | 奇异性惩罚权重，示例为 0.02 |
| `handle.zoom_factor` | 手柄映射缩放，先从 1.0 开始调试 |
| `feedback_timeout_sec` | IK launch 参数，默认 1.0 s，超时停止会话 |

IK 单步角度限制与硬件速度分别设置。当前高层 demo **不读取** profile 中的 `robot.speed_percent`；只改 YAML 不会调速。使用仓库示例驱动时，在独立终端设置：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  arm_type:=piper can_port:=right_arm topics_end:=_right \
  installation_pos:=horizontal speed_percent:=30
```

然后用 `demo_piper.launch.py start_arm:=false` 启动 IK 和 bridge。驱动 `speed_percent` 范围为 0～100，默认 100；自有驱动按厂商接口配置速度。

### 保护点、工作空间和虚拟墙

`protected_frames` 限制的是指定 frame 的原点，不会自动保护整条连杆。工具、肘部 或腕部需要分别选点；可通过 `safety_frames` 建立辅助 frame：

```yaml
safety:
  safety_frames:
    elbow_point: joint3
    wrist_point:
      parent_joint: joint5
      xyz: [0.0, 0.0, 0.05]
      rpy: [0.0, 0.0, 0.0]
  virtual_walls:
    - name: table
      type: plane
      frame: base_link
      point: [0.0, 0.0, 0.0]
      normal: [0.0, 0.0, 1.0]
      keep_side: positive
      margin: 0.02
      protected_frames: [tool0, wrist_point]
```

将此片段合入已有 `safety` 块，保留需要的 workspace；不要写两个同名顶层键。上例保留平面法向一侧，保护点到平面的距离至少 0.02 m。`keep_side: negative` 选择另一侧；`normal` 不可为零向量，`margin` 不可为负。

Box 的每轴 `min` 必须小于 `max`，并留出 `margin` 后仍有可达空间。虚拟墙是 无限平面，RViz 中显示的宽度不是碰撞边界。`collision_pairs` 用于求解后的 自碰撞检测，不是 Opti 硬约束；几何索引需按本机型检查，不能照抄 Nero 的索引。

离线编辑自定义 profile：

```bash
UMA_PROFILE_DIR=/absolute/path/to/config \
  ros2 launch arm_safety_monitor safety_monitor.launch.py \
  profile_name:=my_robot_profile.yaml
```

在 RViz 中拖动 Box/Plane，右键保存，重启遥操节点后生效。编辑器新建对象默认 使用 `protected_frames: [ee]`；本指南的 `tool0` 模板需在保存后改成实际 frame。完整操作见 [README 的安全空间编辑器](README.md)。

</details>

<details>
<summary><strong>6. 启动集成：驱动 + IK + bridge</strong></summary>

先按 README 加载 Conda、ROS 和工作空间。自己的 launch 必须接齐三部分，并让 IK 与 bridge 使用同一份 profile 的回零和缩放设置。

下面是第 3 节六关节、无夹爪配置的完整 launch 模板。示教器几何取自当前示例 wrapper；替换自己的驱动包/launch 名称后，安装为 `my_robot_bringup/launch/teleop.launch.py`。

```python
from agx_arm_ctrl.robot_profile import format_arm_home_state, load_profile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include_launch(package, filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", filename])
        ),
        launch_arguments=arguments.items(),
    )


def build(context):
    side = LaunchConfiguration("side").perform(context)
    path = LaunchConfiguration("profile_path").perform(context)
    profile = load_profile(path, validate_robot=False)
    bridge_args = {
        "port_name": LaunchConfiguration("port_name"),
        "node_name": f"uma_{side[0]}",
        "uma_pose_topic": f"/{side}_handle_pose",
        "remote_joint_topic_in": f"/control/joint_states_{side}_raw",
        "remote_joint_topic_out": f"/control/move_js_{side}",
        "remote_joint_single_topic": f"/feedback/joint_states_{side}",
        "teleop_enable_topic": f"/{side}_teleop_enable",
        "remote_service_name": f"{side}_teleop_trigger",
        "arm_home_state": format_arm_home_state(profile, side),
        "angle_reverse": str(side == "right").lower(),
        "publish_sim_joint": "false",
        "is_nero": "false",
        "use_piper_mode": "false",
        "use_pose_dt_zoom": "true",
        "use_trigger_pose": "true",
        "zoom_factor": str(profile.handle.get("zoom_factor", 1.0)),
        "gripper_length": "0.0",
        "handle_seg0": "[0.0, 0.0, 0.0534, 0.0, 0.0, 0.0]",
        "handle_seg1": "[0.0, 0.0, 0.0, 1.5707963, 0.0, 3.1415926]",
        "handle_seg2": "[0.220, 0.0, 0.0, 0.0, 0.0, -1.5707963]",
        "handle_seg3": "[-0.04804, -0.19115, 0.05487, 1.5707963, 0.0, 0.0]",
    }
    return [
        include_launch("my_robot_driver", "my_robot_driver.launch.py", {
            "side": side,
            "installation_pos": profile.installation_pos_for(side),
        }),
        include_launch("agx_arm_ctrl", "teleop_single_arm.launch.py", {
            "side": side,
            "node_name": f"teleop_{side}_arm_node",
            "profile_path": path,
        }),
        include_launch("uma_bringup", "run_single_node.launch.py", bridge_args),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile_path"),
        DeclareLaunchArgument("side", default_value="left", choices=["left", "right"]),
        DeclareLaunchArgument("port_name", default_value="left_uma"),
        OpaqueFunction(function=build),
    ])
```

这里假定自己的驱动 launch 接受 `side` 和 `installation_pos`，并据此配置 ROS 接口及硬件；实际参数名由驱动定义。驱动的 CAN/串口地址也在驱动 launch 中配置，`port_name` 仅指示教器 CAN。若硬件没有安装设置接口，在驱动端保留其实际支持的 配置即可，IK 的 profile 安装映射独立生效。

```bash
ros2 launch my_robot_bringup teleop.launch.py \
  profile_path:=/absolute/path/my_robot_profile.yaml \
  side:=left port_name:=left_uma
```

使用右路时同时改为 `side:=right port_name:=right_uma`。双臂启动两份对应路由，节点名和驱动接口必须唯一。不要同时启动高层 demo 中的示例驱动和自己的驱动。

</details>

<details>
<summary><strong>7. 夹爪与额外通道</strong></summary>

IK 只输出 URDF 臂关节 `q`。现有 bridge 仍有机型相关约定，七轴或带夹爪的新机械臂 需要单独核对，不能直接套用上面的六关节模板。

| 现有路径 | 行为 | 新机械臂需确认 |
| --- | --- | --- |
| `agx_gripper` | 保留臂关节数组，另发夹爪 JointState topic | 独立夹爪订阅、单位和控制字段是否匹配 |
| `revo2` | 初始化灵巧手驱动，不将单扳机映射为多指动作 | 按手型接口单独发送控制命令 |
| 回零 | 使用完整 `arm_home_state` 与反馈做插值 | 额外通道数量、顺序、单位和回零含义 |

六轴 IK 输出本身不追加夹爪通道。Piper/Nero 示例在 `effector_type: agx_gripper` 时由 bridge 单独发布夹爪命令；其他驱动需要实现对应订阅。设置开合范围时使用 m，确保最小值不大于最大值，并符合硬件行程。

Piper/Nero wrapper 已包含各自的手柄几何、模式和回零取值方式。复用前检查 [bridge_launch.py](uma_bringup/uma_bringup/bridge_launch.py) 与 bridge 输出逻辑；安装方向通用，不代表所有机型的末端执行器协议相同。

</details>

<details>
<summary><strong>8. 分步验收与排障</strong></summary>

先校验模型，再检查数据，最后接入低速运动。以下命令均以左路为例。

**模型加载：** 将代码中的 profile 路径替换为实际路径，在已加载的 `uma` 环境运行：

```bash
python - <<'PY'
from agx_arm_ctrl.robot_profile import load_profile
profile = load_profile('/absolute/path/my_robot_profile.yaml')
print('nq:', profile.nq)
print('joints:', [joint.urdf_name for joint in profile.joints])
print('installation:', profile.installation_pos_for('left'))
PY
```

**链路检查：** `--once` 会等待一条消息，遥操未开启时命令 topic 可能保持安静。

```bash
ros2 topic hz /left_handle_pose
ros2 topic hz /feedback/joint_states_left
ros2 topic echo /feedback/joint_states_left --once
ros2 topic echo /left_teleop_enable --once
ros2 topic echo /control/joint_states_left_raw --once
ros2 topic echo /control/move_js_left --once
```

**真机验收：**

1. 驱动输出暂不接入运动，确认反馈名称、单位、顺序和 `nq`，检查目标是否可达。
2. 按第 4 节初始化并开启遥操，小幅测试前/左/上和三个旋转方向；确认首帧无跳位。
正装另测：换朝向后重新开启、会话中转动手柄后平移、竖直握持拒绝开启。侧装按第 4 节表格核对轴向。正装 Nero 和 Piper 使用同样的映射规则。
3. 配置硬件低速后接入运动，再分别检查回零与夹爪，不把回零当成无动作的复位。
4. 验证反馈中断时会话停止，恢复后重新开启能建立新起点；验证驱动自己的超时和
停止机制。支持重力补偿配置的机械臂另测示教拖动模式。
5. 在接近 Box/Plane 边界时确认保护点约束生效；保存配置后重启，再检查生效值。

| 现象 | 优先检查 |
| --- | --- |
| Profile 加载失败 | URDF 路径、frame 是否存在、`locked_joints`、回零列表 |
| `nq` 或反馈不匹配 | URDF 非臂关节、关节名称、反馈长度及顺序 |
| 正装 x/z 仍互换 | 实际加载的 profile、`installation_pos: horizontal`、是否 source 新 install |
| 正装前向不符 / 提示放平 | 手柄初始化零位是否正确；关闭遥操后放平、指向操作前方再开启 |
| IK infeasible | 限位、Box 余量、墙面法向/保留侧、保护点；查看 `wall_dist` 日志 |
| 开启后立即停止 | 实时反馈是否持续、是否超过 `feedback_timeout_sec` |
| 有 raw、无最终命令 | Bridge 订阅映射、遥操开关、是否发生超时 |
| 有最终命令、不运动 | 驱动订阅、使能、关节映射、厂商 API 和硬件状态 |
| 改 YAML 速度无变化 | Demo 未读取 `speed_percent`，改用驱动参数 |
| 七轴末关节异常 | Bridge 是否把第七项当成夹爪覆盖 |

</details>

<details>
<summary><strong>9. Piper / Nero 参考入口</strong></summary>

以下 demo 默认启动仓库示例驱动、IK 和 bridge。选一条运行即可：

```bash
# Piper：右路，正装由 piper_profile.yaml 指定
ros2 launch uma_bringup demo_piper.launch.py

# Piper：左路，显式选择左侧 CAN 端口
ros2 launch uma_bringup demo_piper.launch.py \
  side:=left arm_port:=left_arm port_name:=left_uma backend:=python

# Nero：双臂
ros2 launch uma_bringup demo_nero_bringup.launch.py
ros2 launch uma_bringup demo_nero_bringup.launch.py backend:=python

# 已单独启动兼容驱动，只启动 Piper IK 和 bridge
ros2 launch uma_bringup demo_piper.launch.py start_arm:=false \
  profile_path:=$PWD/src/UMA_Control/uma_bringup/config/piper_profile.yaml
```

`start_arm:=false` 不会替自有驱动配置接口；仍需满足第 2 节约定。只启动驱动与 IK 而遗漏 bridge，会缺少示教器位姿、按键处理和最终命令转发。

</details>
