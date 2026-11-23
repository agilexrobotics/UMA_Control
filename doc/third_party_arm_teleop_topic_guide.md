# 使用 UMA 示教器遥操作第三方机械臂：话题与驱动适配指南

> 适用对象：非 Piper、非 Nero 的第三方机械臂。

## 1. 先看结论：真正需要替换哪些东西？

**保留 UMA 示教器通信、手柄位姿生成和 profile 驱动的 IK；替换机械臂厂商驱动、机械臂 URDF/profile，并在机械臂边界适配反馈和最终关节命令。**

第三方机械臂最小接入需要两条机械臂数据接口：

1. **实际关节反馈**：第三方驱动 → IK **和** bridge。
2. **最终关节目标**：bridge → 第三方驱动或适配节点。

本文以 `side=right` 为例。单臂也可以使用 `left`，但必须成组替换；`side` 是 ROS 路由侧，**不是机械臂型号，也不是物理安装方向**。

| UMA 示例接口 | 在第三方适配中的处理 | 谁需要连接它 |
| --- | --- | --- |
| `/feedback/joint_states_right` | 必须由第三方机械臂提供真实反馈；可保留名称并由适配层发布，也可改为第三方反馈话题 | **IK、bridge 两个订阅端** |
| `/control/move_js_right` | 必须接入第三方关节位置控制接口；消息不兼容时由适配层转换 | bridge 发布，第三方驱动/适配层订阅 |
| `/control/joint_states_right_raw` | 通常保留，作为 IK → bridge 的内部接口；**不是默认的机械臂驱动输入** | IK 发布，bridge 订阅 |
| `/right_handle_pose` | 通常保留，仍由 UMA bridge 发布手柄位姿；不要替换成机械臂 TCP 反馈 | bridge 发布，IK 订阅 |
| `/right_teleop_enable` | 通常保留，是遥操状态与转发门控，不是厂商电机使能接口 | IK 发布，bridge 订阅 |
| `/right_teleop_trigger` | 通常保留；这是 `Trigger` **服务**，不是话题，不是机械臂上电/使能服务 | IK 提供，bridge 调用 |

源码依据：`uma_kinematics/agx_arm_ctrl/agx_arm_ctrl/teleop_single_arm_node.py:898-903,962-981`；`uma_bringup/uma_bringup/bridge_launch.py:62-74`。本文所有源码路径均相对于 `UMA_Control/`。

### 1.1 左右路由对照

| 接口 | `side=left` | `side=right` |
| --- | --- | --- |
| 机械臂反馈 | `/feedback/joint_states_left` | `/feedback/joint_states_right` |
| 最终机械臂命令 | `/control/move_js_left` | `/control/move_js_right` |
| IK 原始输出 | `/control/joint_states_left_raw` | `/control/joint_states_right_raw` |
| 手柄位姿 | `/left_handle_pose` | `/right_handle_pose` |
| 遥操状态 | `/left_teleop_enable` | `/right_teleop_enable` |
| 遥操切换服务 | `/left_teleop_trigger` | `/right_teleop_trigger` |

`side` 当前只支持 `left/right`，不能把它设成厂商品牌名。使用其他命名空间时应显式 remap；这些接口多为绝对名称，不能只依赖 `namespace` 自动加前缀。

源码依据：`uma_kinematics/agx_arm_ctrl/agx_arm_ctrl/teleop_single_arm_node.py:783-808,898-903,962-981`。

## 2. 理解 UMA 遥操作链路与各层职责

```text
UMA 示教器 CAN
       │
       ▼
UMA_SDK + UMA bridge ─── /right_handle_pose ──► teleop_single_arm / IK
       ▲                                                │
       │                       /control/joint_states_right_raw
       └────────────────────────────────────────────────┘
       │
       │ 遥操门控后的目标；另外也可能包含 bridge 回零命令
       ▼
/control/move_js_right
       │
       ▼
第三方机械臂适配节点 / 第三方驱动
       │ 厂商 SDK、网络、总线或控制器接口
       ▼
第三方机械臂
       │ 真实测量反馈
       ▼
/feedback/joint_states_right ──► IK 与 bridge
```

| 层 | 保留/替换 | 职责 |
| --- | --- | --- |
| UMA_SDK | 通常保留 | UMA 示教器的 CAN 协议、状态与命令；不负责第三方机械臂厂商协议 |
| UMA bridge | 保留示教器功能，审核机型耦合逻辑 | 手柄位姿、按键、遥操状态门控、命令转发及现有回零逻辑 |
| `teleop_single_arm` | 可复用，但须匹配模型能力 | 当前反馈 → 末端起点；手柄增量 → 末端目标；IK → 关节解 |
| 机械臂 URDF/profile | 必须替换 | 第三方机械臂关节、基座、TCP、关节限位、安装方向和相关约束 |
| 厂商驱动/适配层 | 必须提供 | 厂商通信、关节映射、单位换算、持续反馈、控制模式、超时停止、故障与电机使能 |
| 末端执行器 | 按需单独适配 | 扳机到第三方夹爪/灵巧手命令的转换，不应占用机械臂关节槽 |

在 `horizontal` 模式下，开启时捕获机械臂当前 FK 位姿与手柄当前位姿，并固定水平方向映射 `M`。后续目标位置是：

```text
p_target = p_arm_start + M × (p_handle_current - p_handle_start)
```

旋转同样使用手柄相对于开启瞬间的增量，不是把手柄绝对姿态直接当作机械臂姿态。每次开启从真实当前位置继续，**不会因为开启遥操就自动回到 profile 的 home**。

源码依据：`uma_kinematics/agx_arm_ctrl/agx_arm_ctrl/teleop_state.py:53-76`；`uma_kinematics/agx_arm_ctrl/agx_arm_ctrl/teleop_single_arm_node.py:1177-1209,1233-1282`。分层依据：`arm_adaptance_guide.md:18-35`。

## 3. 选择接入方案

### 方案 A：保留 UMA 话题，新增厂商适配节点（推荐）

适合厂商消息类型、关节名、单位或控制模式与 UMA 不同的情况。

以下 `/vendor_arm/...` 都是**示例名**，不是本仓库已经提供的接口：

```text
/vendor_arm/joint_states
    │ 第三方驱动反馈
    ▼
适配层：选出当前机械臂、改名、换单位、统一方向/零位、稳定排序
    ▼
/feedback/joint_states_right ──► IK 与 bridge

/control/move_js_right
    ▼
适配层：校验目标、映射厂商关节、换单位、转换消息/控制模式
    ▼
/vendor_arm/joint_position_cmd 或厂商 SDK/控制接口
```

这时 UMA 内部话题不用改名；**真正被替换的是话题的生产者/消费者以及消息语义**。厂商原有 `/joint_states` 可以继续供其他工具使用，不需要全局改掉。

若厂商反馈包含全机器人、移动底盘或夹爪关节，适配层应发布只含本臂的规范反馈，不要直接把聚合反馈送入 bridge 的回零路径。

### 方案 B：直接 remap 到厂商接口

仅当厂商接口满足下列全部条件时才适合：

- 反馈和命令接口均接受本文约定的 `sensor_msgs/msg/JointState`。
- 关节名、rad 单位、方向、零位与 URDF 已一致。
- 反馈数组顺序稳定，与 `arm_home_state` 一致。
- 控制器能正确执行流式关节位置目标，并有独立的限速与停止处理。

示例映射：

| UMA 接口 | 第三方示例接口 | 需要同时修改的位置 |
| --- | --- | --- |
| `/feedback/joint_states_right` | `/vendor_arm/joint_states` | IK 的 feedback remap；bridge 的 `/joint_states_single` remap |
| `/control/move_js_right` | `/vendor_arm/joint_position_cmd` | bridge 的 `remote_joint_topic_out`；确认第三方驱动订阅该接口 |

**remap 只能改名称，不能把 `JointState` 转成 `JointTrajectory`、厂商自定义消息或服务调用，也不能换算角度单位。**消息不兼容应回到方案 A。

### 不建议：驱动直接接 IK raw 输出

不要仅把厂商驱动订阅端接到 `/control/joint_states_right_raw` 就认为适配完成。这样跳过了现有 bridge 的最终转发与末端处理路径，也没有接入 bridge 的回零命令。若确实要绕过 bridge，必须另外实现并验证门控、超时、回零和末端逻辑；那属于另一套控制架构，本文不把它当作默认方案。

源码依据：`uma_ros/src/uma_ros_messenger.cpp:557-575`；`uma_ros/scripts/uma_ros_py_node.py:728-752`。

## 4. 具体改哪里：话题参数与 remapping 清单

### 4.1 IK 节点

IK 可执行入口是 `agx_arm_ctrl` 包的 `teleop_single_arm`，源码为 `teleop_single_arm_node.py`。

| 数据接口 | 源码中的名称 | 默认建议 | 改名办法 |
| --- | --- | --- | --- |
| 反馈输入 | `/feedback/joint_states_{side}` | 接规范真实反馈 | `Node(remappings=[(...)])` |
| raw 输出 | `/control/joint_states_{side}_raw` | 保留 | 若改名，IK 输出与 bridge `remote_joint_topic_in` 同时改 |
| 手柄输入 | `/{side}_handle_pose` | 保留 | 若改名，IK 订阅与 bridge 手柄位姿输出同时改 |
| 遥操状态输出 | `/{side}_teleop_enable` | 保留 | 若改名，IK 输出与 bridge `/teleop_enable` remap 同时改 |
| 切换服务 | `/{side}_teleop_trigger` | 保留 | 若改名，IK 服务 remap 与 bridge `remote_service_name` 同时改 |

当前 IK 没有独立的 `feedback_topic` 或 `command_topic` 参数。不要只在 YAML 中新增同名字段并期待生效；使用 launch remapping，或者明确修改节点接口。

源码依据：`uma_kinematics/agx_arm_ctrl/setup.py:31-35`；`uma_kinematics/agx_arm_ctrl/agx_arm_ctrl/teleop_single_arm_node.py:783-845,898-903,962-981`。

### 4.2 bridge：区分真正的节点参数与 launch remap 参数

C++ 与 Python bridge 当前都使用下面这组端点。这里仅确认这些接口，不推断两个后端的所有线程和参数行为完全等价。

| 接口 | bridge 创建端点时使用的名称 | 如何指定最终名称 |
| --- | --- | --- |
| IK raw 输入 | 参数 `remote_joint_topic_in` | 设置成 `/control/joint_states_right_raw` 或自定义 raw 话题 |
| 最终机械臂输出 | 参数 `remote_joint_topic_out` | 设置成 `/control/move_js_right` 或厂商规范命令话题 |
| 机械臂反馈输入 | 硬编码 `/joint_states_single` | remap 到 `/feedback/joint_states_right` 或厂商规范反馈话题 |
| 手柄位姿输出 | 硬编码 `/uma_pose` | remap 到 `/right_handle_pose` |
| 遥操状态输入 | 硬编码 `/teleop_enable` | remap 到 `/right_teleop_enable` |
| IK 切换服务客户端 | 参数 `remote_service_name` | 设置成 `/right_teleop_trigger`；建议绝对名避免命名空间错配 |
| AGX 兼容夹爪输出 | 参数 `nero_gripper_topic_name` | 第三方末端另行处理，见第 7 节 |

C++ 的 `run_single_node.launch.py` 提供 `uma_pose_topic`、`remote_joint_single_topic`、`teleop_enable_topic` **launch 参数**，并在内部生成 remappings。它们不是这两个 bridge 源码直接读取的同名节点参数。直接创建 Python bridge `Node` 时，也要明确写出三条 remapping，不能只传这些名字的参数。

源码依据：`uma_ros/src/uma_ros_messenger.cpp:59-118,128-155`；`uma_ros/scripts/uma_ros_py_node.py:315-361,415-457`；`uma_bringup/launch/run_single_node.launch.py:122-177`；`uma_bringup/uma_bringup/bridge_launch.py:161-211`。

### 4.3 不要复制整套 Piper/Nero 驱动话题

最小 UMA IK 链路不要求第三方驱动实现以下 AGX 示例接口：

- `/feedback/tcp_pose_{side}`、`/feedback/arm_status_{side}` 等附加反馈。
- `/control/move_j_{side}`、`/control/move_p_{side}`、`/control/move_l_{side}`、`/control/move_c_{side}`、`/control/move_mit_{side}` 等其他运动模式。
- `/enable_agx_arm_{side}`、`/move_home_{side}` 等 AGX 驱动服务。

这些接口可替换为厂商自己的状态、使能、停止和回零接口；但**功能不能因此省略**。尤其 `/{side}_teleop_trigger` 只切换遥操，不替代厂商电机使能、故障复位或急停。

还要区分 `/control/joint_states_right` 与 `/control/joint_states_right_raw`：前者在 AGX 示例中可能承载独立夹爪命令，不是本文规定的 IK raw 话题。

源码依据：`uma_kinematics/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py:209-264`；`uma_kinematics/agx_arm_ctrl/launch/start_single_agx_arm.launch.py:58-89`；`uma_bringup/uma_bringup/bridge_launch.py:65-74`。

## 5. JointState、关节名字、单位和排序契约

### 5.1 推荐的最小布局：只含机械臂关节

以六轴 URDF 关节 `joint1` 至 `joint6` 为例：

```yaml
name: [joint1, joint2, joint3, joint4, joint5, joint6]
position: [q1, q2, q3, q4, q5, q6]  # 所有值单位 rad
```

规范反馈应满足：

- `name` 与 `position` 长度一致、关节名唯一、位置有限，不能包含 NaN/Inf。
- 使用机械臂真实测量值，不能把最后一次命令重新发布成“真实反馈”。
- 持续发布，而不是只在收到控制命令后发布一次。
- 与 URDF 的关节正方向、零位和关节名称一致。
- 为兼容 bridge 回零，输出数组按 URDF 机械臂关节顺序稳定排列。

IK 从反馈中**按名称**提取关节；bridge 回零则按**数组索引**插值。因此，“IK 读得对”不代表“回零顺序也正确”。IK 生成的位置命令不附带速度和力矩字段，适配层不能依赖这些字段控制厂商电机。

源码依据：`uma_kinematics/agx_arm_ctrl/agx_arm_ctrl/robot_profile.py:153-185`；`uma_kinematics/agx_arm_ctrl/agx_arm_ctrl/teleop_single_arm_node.py:1007-1023,1051-1069`；`uma_ros/scripts/uma_ros_py_node.py:578-665`。

### 5.2 当前 profile 不支持通过 YAML 任意改关节名

虽然 `JointMapping` 有 `urdf_name/feedback_name/command_name` 字段，但当前 `validate()` 从 URDF 自动派生机械臂关节，并将三个名字初始化为同一个 URDF 名称；`load_profile()` 没有加载自定义 YAML `joints` 映射表。

所以不要写一个 `robot.joints` 区块就认为驱动别名已经映射成功。建议：

1. 在厂商适配层把反馈名转换为 URDF 名。
2. 将 IK/bridge 命令中的 URDF 名映射回厂商关节名。
3. 方向、零位、单位转换也放在该边界，避免在多个节点重复处理。

例如厂商使用度数，经过标定后可采用以下转换关系：

```text
q_urdf_rad = sign × q_vendor_deg × π / 180 + offset_rad
q_vendor_deg = (q_urdf_rad - offset_rad) × 180 / (sign × π)
```

`sign` 与 `offset_rad` 必须由实际标定确定，不是按机械臂品牌猜测。转换应按关节名查找后稳定排序，并校验缺失/重复关节及值范围。

源码依据：`uma_kinematics/agx_arm_ctrl/agx_arm_ctrl/robot_profile.py:1-4,54-58,327-339,367-371,563-595`。

### 5.3 厂商仅支持轨迹或服务控制时

若厂商只接收轨迹消息、动作目标或单次运动服务，适配层需要另外实现目标更新/覆盖策略、速度与加速度约束、取消/停止和故障处理。不要把每一帧 IK 目标都简单提交成一条排队执行的长轨迹，也不要假定控制器能承受示例的发布频率。

本文的 `JointState` 是“目标关节位置”契约，不是厂商通信协议。没有合适流式控制接口的机械臂，不能仅靠改话题名完成实时遥操作。

## 6. 替换机械臂 URDF/profile

### 6.1 URDF 要求与当前通用性边界

- 使用目标机械臂真实的连杆尺寸、关节轴、关节零位与限位。
- 优先采用固定基座、单臂串联、单自由度且有限位的转动关节模型。
- `base_frame` 必须是模型中的真实 frame，并处于固定基座链上。
- `ee_frame` 对应要操控的真实 TCP；没有合适 frame 时可用 `ee_offset`。
- 首次适配推荐只加载机械臂链，末端活动关节独立处理。
- 当前关节派生主要识别 Pinocchio revolute 模型，并按单自由度计算 `nq`。含移动底座、伸缩关节、多自由度关节、continuous 关节的 URDF 必须额外复核，不能当作已验证的通用模型。
- 不足六个可控自由度的机构未必能完整跟踪六维末端目标，需要评估任务约束或修改 IK。

`horizontal` 路径会把 FK 结果转换到 `profile.base_frame`；侧装 `left/right` 路径沿用旧映射，当前实现注释要求模型根坐标与 `base_frame` 重合。第三方模型不能盲目套用侧装示例。

源码依据：`uma_kinematics/agx_arm_ctrl/agx_arm_ctrl/robot_profile.py:327-403`；`uma_kinematics/agx_arm_ctrl/agx_arm_ctrl/teleop_single_arm_node.py:157-225,448-461,984-1000`。

### 6.2 六轴、无夹爪的 profile 结构示例

建议在自己的 `my_arm_bringup/config/vendor_arm_profile.yaml` 中新增配置，而不是修改 Piper/Nero 的现有 profile。

**以下模型路径、TCP 名、home 值与每帧限值仅展示字段结构，必须替换/标定，不是可直接用于真机的参数。**

```yaml
robot:
  name: vendor_arm
  urdf_path: package://my_arm_description/urdf/vendor_arm.urdf
  base_frame: base_link
  installation_pos: horizontal
  ee_frame: tool0
  locked_joints: []
  effector_type: none
  # 当前 schema 保留的 pyAgxArm 字段；不代表第三方固件版本
  firmware_version: default
  arm_home_state:
    default: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

ik:
  enable_hard_delta_q_constraint: true
  max_delta_q_rad: 0.01  # 仅示例；是每步约束，不是电机速度上限
  enable_psi_constraint: false
  enable_joint_reference_penalty: false
  enable_elbow_side_penalty: false
  singularity_weight: 0.02

handle:
  zoom_factor: 1.0

# 未配置安全空间不代表不存在碰撞风险；应按真实场地另行设计和验证。
safety: {}
```

注意：

- 当前 loader 不会读取 `robot.speed_percent` 来设置第三方驱动速度，厂商限速在厂商驱动/适配层配置。
- `arm_home_state` 必须至少满足本臂 IK 起点参考的维度；如果启用 bridge 回零，其总长度还必须等于规范反馈的通道数，顺序一一对应。六轴无末端通道示例使用六项，不照抄 Piper 的九项或 Nero 的十项。
- 七轴无末端通道时使用七项，并处理第 7 节的第七槽覆盖问题。
- `ee_frame` 与 `ee_offset` 二选一；当前实现同时配置时优先使用 `ee_frame`。
- 肘部惩罚需要真实肩/肘/腕 frame；未确认前不要复制 Nero 的 frame 名和惩罚参数。
- `firmware_version` 和 `effector_type` 的当前可选值仍含 AGX 语义。不要填任意厂商固件字符串或自定义末端名并期待现有 loader 接受。

源码依据：`uma_kinematics/agx_arm_ctrl/agx_arm_ctrl/robot_profile.py:15-51,269-289,545-619`；`uma_kinematics/agx_arm_ctrl/agx_arm_ctrl/teleop_single_arm_node.py:168-191,331-339,945-955`。

## 7. 接真机前必须处理的 bridge 兼容性问题

### 7.1 七轴机械臂：第七个关节可能被扳机宽度覆盖

当前 C++、Python bridge 均存在如下行为：

```text
is_nero == false 且 raw JointState.position 长度 >= 7
    → position[6] 被替换为手柄计算的夹爪宽度
```

**这会破坏第三方七轴机械臂的第七个真实关节目标。`effector_type=none` 并不会关闭这段覆盖。**

处理办法：

- 推荐修改/扩展实际采用的 bridge，使机械臂自由度与夹爪通道显式配置，机械臂 raw 关节位置原样转发，夹爪走独立通道；同时测试七轴数据。
- 当前源码中，`is_nero=true` 可绕开这段覆盖，能作为经审核的临时兼容设置，但它是历史开关，不代表第三方机械臂变成 Nero，也不代表其他所有逻辑已兼容。
- 本文后面的六轴模板使用 `is_nero=false`，前提是 IK raw **严格只有六个机械臂关节**。七轴不得不加修改地照抄模板。

源码依据：`uma_ros/src/uma_ros_messenger.cpp:557-575`；`uma_ros/scripts/uma_ros_py_node.py:728-741`。

### 7.2 第三方夹爪：单独适配

初次机械臂接入建议 `effector_type=none`，只验证臂关节链。若需要夹爪：

- 从 bridge 的私有 `~/handle_state` 获取 `trigger_press_percentage`，由独立适配层转换为厂商夹爪开口、位置或力控制命令。
- 按实际夹爪限制定义单位、方向、死区、限幅、状态门控与通信失效处理。
- 不要把夹爪值写进七轴机械臂的 `position[6]`。
- `agx_gripper` 兼容路径会在 `nero_gripper_topic_name` 输出名为 `gripper` 的 `JointState`，还包含示例 effort 值；这不是所有第三方夹爪都能直接使用的协议。
- 私有话题随节点名变化，例如本文 C++ 模板节点名为 `uma_vendor_r`、无 namespace 时，`~/handle_state` 对应 `/uma_vendor_r/handle_state`。

源码依据：`uma_msgs/msg/HandleState.msg:1-13`；`uma_ros/src/uma_ros_messenger.cpp:64-86,1110-1126`；`uma_ros/scripts/uma_ros_py_node.py:326-341`。

### 7.3 回零：关闭遥操不等于完全没有机械臂命令

bridge 的既有回零逻辑会在遥操关闭时，按反馈数组与 `arm_home_state` 做约 0.6 秒、50 Hz 的关节插值，直接发布到最终命令话题，**不经过 IK 求解及其工作空间约束**。

首次第三方真机接入应先禁用/隔离该运动路径，或改接厂商经过验证的回零流程；仅约定“不要长按”不等于已经禁用。当前本文模板没有提供已实现的回零禁用开关。

还需注意：

- 不要把 UMA 的 `~/reset_pose`、`~/reset_control` 当成厂商回零接口。
- 不要把 bridge 回零路径与 AGX 驱动 `/move_home_{side}` 服务混为一谈。
- 同一个最终 `JointState` 话题当前不携带“遥操/回零”来源标签。适配层若一律在 `teleop_enable=false` 时拒绝目标，会同时拒绝这套回零；应明确选择禁用旧回零，或设计独立且经过授权的回零通道。
- “停止发布”并不保证控制器取消已经收到的运动，驱动必须实现实际停止策略。

源码依据：`uma_ros/src/uma_ros_messenger.cpp:352-405,434-478,506-536`；`uma_ros/scripts/uma_ros_py_node.py:578-665`；`uma_kinematics/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py:263-264`。

### 7.4 必须关闭模拟反馈

低层 C++ launch 的 `publish_sim_joint` 默认为 `true`；模拟发布器与真实反馈订阅使用同一个基础名 `/joint_states_single`。若 remap 到真实机械臂反馈话题，模拟消息可能混入反馈链路。

因此第三方真实反馈接入必须显式设置 `publish_sim_joint=false`，并检查该话题的发布者，避免把 UMA 示教器编码器值当作机械臂反馈。Piper/Nero wrapper 已显式关闭，但绕过 wrapper 时不能依赖这个默认。

源码依据：`uma_bringup/launch/run_single_node.launch.py:82-84,173-176`；`uma_ros/src/uma_ros_messenger.cpp:64-72,92-97`；`uma_bringup/uma_bringup/bridge_launch.py:139-140,186`。

## 8. 第三方 launch 集成示例

### 8.1 替换机械臂驱动，而不是只改 `arm_type`

不要将第三方品牌名直接传给现有 Piper/Nero wrapper。当前共享 wrapper 工厂仅接受 `nero/piper`，并根据该类型选择历史参数和末端逻辑。

新增自己的 bringup，组合：

1. UMA bridge。
2. `teleop_single_arm` IK。
3. 厂商驱动及方案 A 所需适配节点（使用厂商自己的配置）。

不要启动现有 `agx_arm_ctrl_single` 作为第三方机械臂驱动；它是现有 AGX 集成。示教器 `port_name` 仍指 UMA CAN 接口，不应填厂商控制器 IP 或把第三方机械臂接口配置为 UMA 示教器端口。

源码依据：`uma_bringup/uma_bringup/bridge_launch.py:49-79,215-221`；`uma_kinematics/agx_arm_ctrl/launch/start_single_agx_arm.launch.py:35-55`。

### 8.2 六轴、无夹爪的 C++ bridge + IK 模板

下面是**需要新增到自有 bringup 包的示例**，不是仓库里已经存在的适配 launch。它演示方案 B 的两个机械臂话题替换点；使用方案 A 时将 feedback 设置为 `/feedback/joint_states_right`，将最终输出接适配层。

**默认命令输出到观察话题 `/uma_adaptation/observed_joint_commands`，不接厂商运动接口。模板不启动厂商驱动，不自动启用机械臂；本文没有执行它。输出隔离也不能替代整机安全措施，bridge 本身仍连接示教器。**

```python
# 示例文件：my_arm_bringup/launch/uma_vendor_arm.launch.py
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from agx_arm_ctrl.robot_profile import load_profile, format_arm_home_state
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    side = "right"
    profile_path = str(
        Path(get_package_share_directory("my_arm_bringup"))
        / "config" / "vendor_arm_profile.yaml"
    )
    # 只用于提取参数；IK 节点还会对真实 URDF 做完整校验。
    profile = load_profile(profile_path, validate_robot=False)
    home = format_arm_home_state(profile, side)
    feedback = LaunchConfiguration("arm_feedback_topic")
    command = LaunchConfiguration("arm_command_topic")
    uma_port = LaunchConfiguration("uma_port")

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("uma_bringup"))
                / "launch" / "run_single_node.launch.py")
        ),
        launch_arguments={
            "node_name": "uma_vendor_r",
            "port_name": uma_port,
            "uma_pose_topic": "/right_handle_pose",
            "remote_joint_topic_in": "/control/joint_states_right_raw",
            "remote_joint_topic_out": command,       # 替换点 1：最终命令
            "remote_joint_single_topic": feedback,  # 替换点 2a：bridge 反馈
            "teleop_enable_topic": "/right_teleop_enable",
            "remote_service_name": "/right_teleop_trigger",
            "publish_sim_joint": "false",
            "arm_home_state": home,
            "effector_type": "none",
            "is_nero": "false",  # 仅限 raw 严格为六轴；七轴见第 7.1 节
            "angle_reverse": "true",  # 参考右手柄配置，须按实际安装核对
            "use_piper_mode": "false",
            "world_to_base_y": "0.0",
            "gripper_length": "0.0",
            "use_pose_dt_zoom": "true",
            "zoom_factor": str((profile.handle or {}).get("zoom_factor", 1.0)),
            "pub_rate": "200.0",  # 示教器发布示例值，不是厂商控制器能力承诺
            # 以下是当前 wrapper 的 UMA 示教器几何参考，不是机械臂连杆。
            # 必须核对实际 UMA 硬件版本、零位与 IMU 标定。
            "handle_seg0": "[0.0, 0.0, 0.0534, 0.0, 0.0, 0.0]",
            "handle_seg1": "[0.0, 0.0, 0.0, 1.5707963, 0.0, 3.1415926]",
            "handle_seg2": "[0.220, 0.0, 0.0, 0.0, 0.0, -1.5707963]",
            "handle_seg3": "[-0.04804, -0.19115, 0.05487, 1.5707963, 0.0, 0.0]",
        }.items(),
    )

    ik = Node(
        package="agx_arm_ctrl",
        executable="teleop_single_arm",
        name="teleop_vendor_right",
        prefix="python3",
        parameters=[{
            "side": side,
            "profile_path": profile_path,
            "feedback_timeout_sec": 1.0,
        }],
        remappings=[
            ("/feedback/joint_states_right", feedback),  # 替换点 2b：IK 反馈
        ],
        additional_env={"PYTHONNOUSERSITE": "1"},
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("uma_port", default_value="right_uma"),
        DeclareLaunchArgument("arm_feedback_topic", default_value="/vendor_arm/joint_states"),
        DeclareLaunchArgument("arm_command_topic", default_value="/uma_adaptation/observed_joint_commands"),
        bridge,
        ik,
        # 厂商驱动和适配层另行集成，不使用 AGX 示例驱动。
    ])
```

**话题替换最关键的是 `remote_joint_topic_out`、`remote_joint_single_topic` 和 IK 的 feedback remap 三处。后两处必须指向同一个规范真实反馈话题。**

此模板只做语法和接口核对，不代表在某个厂商模型上已运行成功。`my_arm_bringup/my_arm_description`、URDF、profile、控制器能力、手柄标定和回零隔离都需要实际实现。

源码依据：`uma_bringup/launch/run_single_node.launch.py:122-177`；`uma_bringup/uma_bringup/bridge_launch.py:16-21,120-154`；`uma_kinematics/agx_arm_ctrl/launch/teleop_single_arm.launch.py:32-47`。

### 8.3 如果改用 Python bridge

Python bridge 可执行文件是 `uma_ros` 包的 `uma_ros_py_node.py`，不是 C++ `run_single_node.launch.py` 的另一种 executable 参数。直接创建 `Node` 时：

```python
# 以下是 Python Node 配置片段，不是完整 launch。
parameters = [{
    "remote_joint_topic_in": "/control/joint_states_right_raw",
    "remote_joint_topic_out": "/uma_adaptation/observed_joint_commands",
    "remote_service_name": "/right_teleop_trigger",
    "publish_sim_joint": False,
    "effector_type": "none",
    # 还必须补齐真实 port_name、home、手柄几何、IMU 等所需参数。
}]
remappings = [
    ("/uma_pose", "/right_handle_pose"),
    ("/joint_states_single", "/vendor_arm/joint_states"),
    ("/teleop_enable", "/right_teleop_enable"),
]
```

同时保留 IK 的 feedback remap。Python 后端还需要其实际运行环境可导入 `uma_sdk`；改变 bridge 后端不会自动替换 IK 或厂商驱动，也不会自动解决第七槽/回零兼容问题。

源码依据：`uma_ros/scripts/uma_ros_py_node.py:315-361,728-741`；`uma_bringup/uma_bringup/bridge_launch.py:161-211`。

## 9. 联调与验收：先不接运动输出

以下是建议验收步骤，不是本文已经执行的机器人操作。

### 阶段 A：模型和接口离线核对

- [ ] 核对真实机械臂自由度、URDF 关节名/方向/零位、TCP 与基座。
- [ ] 对已知真实关节姿态比较 FK 与厂商 TCP，排查模型或标定偏差。
- [ ] 检查 profile 可被完整校验，而不只是 `validate_robot=False` 加载成功。
- [ ] 验证单位转换往返一致、关节重排稳定、缺失/重复/非有限数据被拒收。
- [ ] 七轴用离线测试确认 `position[6]` 不被夹爪覆盖。

### 阶段 B：真实反馈、命令仅观察

在系统由操作者明确启动且命令输出已隔离后，可以使用以下**只读诊断命令**检查图和消息；不要通过 `ros2 topic pub` 或服务调用试探真实机器人。

```bash
# 方案 A 的规范反馈；方案 B 换成实际规范反馈话题
ros2 topic info /feedback/joint_states_right --verbose
ros2 topic echo /feedback/joint_states_right --once
ros2 topic hz /feedback/joint_states_right

ros2 topic info /right_handle_pose --verbose
ros2 topic info /control/joint_states_right_raw --verbose
ros2 topic info /uma_adaptation/observed_joint_commands --verbose
ros2 service list -t
```

- [ ] 反馈发布者确实是第三方驱动/适配层，没有 UMA 模拟关节混入。
- [ ] feedback 的订阅者包括 IK 与 bridge；raw 链路连接 IK 与 bridge。
- [ ] 消息类型、QoS 实际匹配；第三方仅 best-effort 反馈时须核对当前订阅 QoS，不能只看名字相同。
- [ ] 未收到新鲜反馈时不能开启；反馈中断后遥操会关闭。
- [ ] `horizontal` 启动还需要有效新鲜手柄位姿；手柄朝向接近竖直时应拒绝开启。
- [ ] 从机械臂当前姿态建立起点；小幅手柄移动生成方向正确、连续的关节目标。
- [ ] 关闭遥操后停止实时 raw 转发；另行验证已隔离的回零路径，不把它误认成门控失效。

默认 IK 反馈窗口为 1 秒，状态周期为 50 ms；它按本机接收时间判断新鲜度，不会认证这份数据一定来自真实编码器。应根据厂商反馈周期和系统需求设计更完整的失效处理，而不是仅依赖该默认值。

源码依据：`uma_kinematics/agx_arm_ctrl/agx_arm_ctrl/teleop_single_arm_node.py:783-793,979-982,1007-1056,1212-1282`。

### 阶段 C：完成独立安全验证后，操作者授权的低速真机测试

- [ ] 厂商驱动有经过验证的电机使能、停止/取消、故障和通信失效处理。
- [ ] 控制器速度/加速度/关节限位独立设置；每帧 IK 限值不是硬件速度限制。
- [ ] 真实工作空间、TCP、障碍物、负载及重力补偿经过验证。
- [ ] bridge 旧回零已禁用/隔离或改接已验证流程。
- [ ] 夹爪独立适配，并验证不会污染臂关节目标。
- [ ] 急停可用、现场隔离，按关节/方向逐项小范围验收。
- [ ] 手柄零位、IMU 参考与基座方向已核对，遥操中保持示教器底座稳定。

IK 的关节限位、虚拟墙和工作空间盒是软件约束。当前自碰撞标志只打印警告，并未阻止已求解目标发布；bridge 回零也绕过 IK。**不能把这些机制当作硬件安全认证或急停替代品。**

源码依据：`uma_kinematics/agx_arm_ctrl/agx_arm_ctrl/teleop_single_arm_node.py:527-605,1084-1097`；`README.md:483-491`。

## 10. 常见故障定位表

| 现象 | 优先检查 |
| --- | --- |
| 手柄有数据，但无法开启遥操 | IK 的 feedback remap 是否正确；真实反馈是否持续；规范关节名是否齐全；正装手柄姿态是否有效 |
| IK raw 有目标，最终话题无目标 | bridge 的 `remote_joint_topic_in`、`/teleop_enable` remap、Trigger 服务是否匹配 |
| 最终话题有目标，机械臂不动 | 厂商接口类型/控制模式是否匹配；适配层是否启用输出；电机是否使能；驱动是否故障或拒绝目标 |
| 七轴第七关节突然异常 | `is_nero=false` 导致 `position[6]` 被覆盖；末端通道是否错误混入 arm raw |
| FK 对不上真实 TCP | URDF 尺寸、方向/零位标定、单位、base/TCP frame 是否错误 |
| 遥操正常但回零错关节 | 反馈数组顺序与 home 不一致；IK 按名字正确并不代表 bridge 按索引正确 |
| 反馈断了但 controller 仍移动 | 控制器保留了旧命令；需要驱动侧明确停止/取消，不能只停止 ROS 发布 |
| 添加 YAML 关节别名后仍报缺关节 | 当前 loader 不加载自定义 `joints` 映射；在适配层改名 |
| 改 namespace 后仍连到原话题 | 原接口以 `/` 开头，需显式 remap，并核对 Trigger 客户端绝对名 |
| 接入真实反馈后出现额外关节消息 | 检查 `publish_sim_joint=false`、聚合反馈来源和话题发布者 |

## 11. 最终交付清单

建议第三方集成保存在自有 ROS 包中，并交付：

- 第三方机械臂 URDF/TCP 定义与 `vendor_arm_profile.yaml`。
- 厂商驱动，以及需要时的消息、关节和单位适配节点。
- UMA bridge + IK + 第三方驱动组合 launch，明确标注实际话题名。
- 第七槽/夹爪与旧回零处理方案及测试。
- 驱动限速、停止、超时和故障处理说明。
- 关节映射/标定表与离线、只观察、低速真机验收记录。

**最小话题改动总结：让第三方反馈同时进入 IK 与 bridge，让 bridge 的最终关节目标进入第三方控制接口；保留手柄位姿、IK raw、遥操状态与 Trigger 服务的内部配对。换驱动与模型后，还必须审核 bridge 的第七槽、末端和回零逻辑，不能仅替换话题名称。**
