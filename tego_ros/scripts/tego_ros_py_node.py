#!/usr/bin/env python3
"""
Python bridge for the Tego handle SDK
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool
from std_srvs.srv import Empty, SetBool, Trigger
from tf2_ros import TransformBroadcaster

from tego_msgs.msg import HandleState, MtSensorStatus, SensorData
from tego_msgs.srv import ResetControl
from tego_sdk import (
    HandleLightControlCmd,
    HandleMotorControlCmd,
    LightColorId,
    MotorMode,
    MtResetCmd,
    ResetMode,
    TegoBase,
)


def _clamp_min(value: float, minimum: float) -> float:
    return value if value > minimum else minimum


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def _quat_inv(q):
    norm = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]
    if norm <= 1.0e-12:
        return (1.0, 0.0, 0.0, 0.0)
    qc = _quat_conj(q)
    return (qc[0] / norm, qc[1] / norm, qc[2] / norm, qc[3] / norm)


def _quat_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr,
    )


def _quat_to_rpy(q):
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _quat_to_rot(q):
    w, x, y, z = q
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ]


def _rot_to_quat(rot):
    tr = rot[0][0] + rot[1][1] + rot[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return (
            0.25 * s,
            (rot[2][1] - rot[1][2]) / s,
            (rot[0][2] - rot[2][0]) / s,
            (rot[1][0] - rot[0][1]) / s,
        )
    i = 0
    if rot[1][1] > rot[0][0]:
        i = 1
    if rot[2][2] > rot[i][i]:
        i = 2
    j = (i + 1) % 3
    k = (j + 1) % 3
    s = math.sqrt(rot[i][i] - rot[j][j] - rot[k][k] + 1.0) * 2.0
    q = [0.0, 0.0, 0.0, 0.0]
    q[i + 1] = 0.25 * s
    q[0] = (rot[k][j] - rot[j][k]) / s
    q[j + 1] = (rot[j][i] + rot[i][j]) / s
    q[k + 1] = (rot[k][i] + rot[i][k]) / s
    return (q[0], q[1], q[2], q[3])


def _eye4():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _mat4_mul(a, b):
    return [
        [
            a[i][0] * b[0][j]
            + a[i][1] * b[1][j]
            + a[i][2] * b[2][j]
            + a[i][3] * b[3][j]
            for j in range(4)
        ]
        for i in range(4)
    ]


def _mat4_inv_rigid(t):
    r = [[t[i][j] for j in range(3)] for i in range(3)]
    rt = [[r[j][i] for j in range(3)] for i in range(3)]
    inv = _eye4()
    for i in range(3):
        for j in range(3):
            inv[i][j] = rt[i][j]
    for i in range(3):
        inv[i][3] = -sum(rt[i][j] * t[j][3] for j in range(3))
    return inv


def _to_transform_xyz_rpy(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, scale=1.0):
    """Same convention as C++ ToTransform: Rz(yaw)*Ry(pitch)*Rx(roll), translation scaled."""
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x * scale],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y * scale],
        [-sp, cp * sr, cp * cr, z * scale],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _to_transform_xyz_quat(x=0.0, y=0.0, z=0.0, q=(1.0, 0.0, 0.0, 0.0)):
    rot = _quat_to_rot(q)
    return [
        [rot[0][0], rot[0][1], rot[0][2], x],
        [rot[1][0], rot[1][1], rot[1][2], y],
        [rot[2][0], rot[2][1], rot[2][2], z],
        [0.0, 0.0, 0.0, 1.0],
    ]


class TegoBridgeNode(Node):
    def __init__(self, tego: TegoBase, node_name: str = "tego_bridge_node") -> None:
        super().__init__(node_name)
        self.tego = tego
        self.tf_broadcaster = TransformBroadcaster(self)
        self._stop_threads = threading.Event()
        self._threads: list[threading.Thread] = []

        # Same defaults as the C++ messenger; overridden by launch parameters.
        self.check_get = True
        self.wait_feedback = True
        self.publish_sim_joint = True
        self.gripper_width_min = -0.01
        self.gripper_width_max = 0.07
        self.remote_joint_topic_in = "joint_in"
        self.remote_joint_topic_out = "joint_out"
        self.remote_service_name = "teleop_piper_status"
        self.node_name_param = "tego_node"
        self.arm_home_state = "0, 0, 0, 0, 0, 0, 0.1"
        self.imu_zero_rpy = "0, 0, 0"
        self.nero_gripper_topic_name = "gripper_states"

        self.angle_value = 0.35
        self.angle_reverse = False
        self.add_angle = True
        self.base_reverse = False
        self.imu_reverse = True
        self.show_imu_data = True
        self.use_piper_mode = True
        self.is_nero = False
        self.effector_type = "none"
        self.zoom_factor = 1.0
        self.handle_seg0 = [0.0, 0.0, 0.123, 0.0, 0.0, 0.0]
        self.handle_seg1 = [0.0, 0.0, 0.0, 1.5707963, -0.1359, 3.1415926]
        self.handle_seg2 = [0.28503, 0.0, 0.0, 0.0, 0.0, -1.7939]
        self.handle_seg3 = [-0.022, -0.25075, 0.0, 1.5707963, 0.0, 0.0]
        self.base_to_imu_roll = 0.0
        self.base_to_imu_pitch = 0.0
        self.base_to_imu_yaw = 0.0
        self.world_to_base_y = 0.0
        self.arm_home_state_vec: list[float] = []
        self.imu_zero_rpy_vec = [0.0] * 3

        self.joint_publisher = None
        self.sim_joint_publisher = None
        self.remote_joint_publisher = None
        self.sensor_status_publisher = None
        self.handle_state_publisher = None
        self.handle_pose_publisher = None
        self.imu_publisher = None
        self.nero_gripper_publisher = None

        self.teleop_piper_cli = None

        self.mt_sensor_send = [0] * 7
        self.handle_status_send = 0
        self.handle_calibration_send = 0
        self.key1_send = 0
        self.key2_send = 0
        self.key_event_send = 0
        self.imu_linear_acceleration_send = 0
        self.imu_angular_velocity_send = 0
        self.imu_orientation_send = 0

        self.update_imu = True
        self.update_joint = True
        self.update_base_pose = False
        self.imu2base_rpy_q = (1.0, 0.0, 0.0, 0.0)
        self.imu_base_quatd = (1.0, 0.0, 0.0, 0.0)
        self.imu_diff_quatd = (1.0, 0.0, 0.0, 0.0)
        self.imu_zero_quatd = (1.0, 0.0, 0.0, 0.0)
        self.get_imu = False
        self.angle0_base = 0.0
        self.angle1_base = 0.0
        self.angle2_base = 0.0
        self.use_trigger_pose = False
        self.gripper_length = 0.15

        self.angle_list = [0.0] * 7
        self.trigger_press_percentage = 0

        self.is_trigger_action = False
        self.trigger_action_tmp = False
        self._resetting_trigger_action = False  # reset thread syncs local state only
        self.is_init_action = False
        self.init_action_tmp = False
        self._init_action_running = False  # atomic-style claim flag
        self.teleop_enable = False
        self.last_get_teleop_enable_time = self._now_seconds()
        self.last_call_teleop_time = self.last_get_teleop_enable_time
        self.last_arm_joint_time = 0.0
        self.last_remote_joint = JointState()
        self.last_arm_joint = JointState()

        self.use_pose_dt_zoom = True
        self.remote_base_pose_matrix = _eye4()
        self.cur_pose_matrix = _eye4()

        self.low_press_level = 10
        self.high_press_level = 70
        self.double_press_timeout = 0.6
        self.long_press_timeout = 2.0
        # RLock: set_parameters() re-enters dynamic_parameters_callback from the
        # finally block, which already holds this lock.
        self.config_lock = threading.RLock()
        self.update_lock = threading.Lock()
        self.is_reconfig_run = False
        self._remote_publish_lock = threading.Lock()
        self._arm_joint_lock = threading.Lock()
        self._teleop_state_lock = threading.Lock()

        self.remote_pose_first_init = True
        self.last_gripper_length = -1.0

        self._parameter_setters: dict[str, Callable[[object], None]] = {
            "add_angle": lambda v: setattr(self, "add_angle", bool(v)),
            "base_reverse": lambda v: setattr(self, "base_reverse", bool(v)),
            "imu_reverse": lambda v: setattr(self, "imu_reverse", bool(v)),
            "show_imu_data": lambda v: setattr(self, "show_imu_data", bool(v)),
            "use_piper_mode": lambda v: setattr(self, "use_piper_mode", bool(v)),
            "use_pose_dt_zoom": lambda v: setattr(self, "use_pose_dt_zoom", bool(v)),
            "zoom_factor": lambda v: setattr(self, "zoom_factor", float(v)),
            "use_trigger_pose": lambda v: setattr(self, "use_trigger_pose", bool(v)),
            "gripper_length": lambda v: setattr(self, "gripper_length", float(v)),
            "handle_seg0": lambda v: setattr(self, "handle_seg0", list(v)),
            "handle_seg1": lambda v: setattr(self, "handle_seg1", list(v)),
            "handle_seg2": lambda v: setattr(self, "handle_seg2", list(v)),
            "handle_seg3": lambda v: setattr(self, "handle_seg3", list(v)),
            "trigger_action": lambda v: setattr(self, "trigger_action_tmp", bool(v)),
            "init_action": lambda v: setattr(self, "init_action_tmp", bool(v)),
        }

        self._state_timer = None

    def setup_subscription(self) -> None:
        self.declare_parameters_all()

        self.joint_publisher = self.create_publisher(JointState, "~/joint", 200)
        if self.publish_sim_joint:
            self.sim_joint_publisher = self.create_publisher(
                JointState, "/joint_states_single", 200
            )
        self.sensor_status_publisher = self.create_publisher(
            MtSensorStatus, "~/mt_sensor_status", 200
        )
        self.handle_state_publisher = self.create_publisher(
            HandleState, "~/handle_state", 200
        )
        self.handle_pose_publisher = self.create_publisher(
            PoseStamped, "/pika_pose", 200
        )
        self.imu_publisher = self.create_publisher(Imu, "~/imu", 200)
        self.remote_joint_publisher = self.create_publisher(
            JointState, self.remote_joint_topic_out, 200
        )
        if self.effector_type == "agx_gripper":
            self.nero_gripper_publisher = self.create_publisher(
                JointState, self.nero_gripper_topic_name, 200
            )

        self.create_subscription(
            JointState, self.remote_joint_topic_in, self.remote_joint_callback, 200
        )
        self.create_subscription(
            JointState, "/joint_states_single", self.arm_joint_callback, 200
        )
        self.create_subscription(
            Bool, "/teleop_enable", self.teleop_enable_callback, 200
        )

        self.create_service(Empty, "~/reset_pose", self.reset_pose_server)
        self.create_service(ResetControl, "~/reset_control", self.reset_control_server)
        self.create_service(
            SetBool, "~/enable_handle_motor", self.enable_handle_motor_server
        )
        self.create_service(SetBool, "~/enable_reverse", self.enable_reverse_server)

        self.teleop_piper_cli = self.create_client(Trigger, self.remote_service_name)
        self.add_on_set_parameters_callback(self.dynamic_parameters_callback)

        self._spawn_thread(self.reset_teleop_enable_thread)
        self._spawn_thread(self.reset_trigger_action_thread)

    def start_publish_loop(self, hz: float = 100.0) -> None:
        self._state_timer = self.create_timer(1.0 / hz, self.publish_state_to_ros)

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def _spawn_thread(self, target: Callable[[], None]) -> None:
        thread = threading.Thread(target=target, daemon=True)
        thread.start()
        self._threads.append(thread)

    def _sleep(self, seconds: float) -> None:
        time.sleep(max(0.0, seconds))

    def get_ros_parameter(self, name: str, default_value):
        if not self.has_parameter(name):
            self.declare_parameter(name, default_value)
        return self.get_parameter(name).value

    def declare_parameters_all(self) -> None:
        self.check_get = bool(self.get_ros_parameter("check_get", self.check_get))
        self.wait_feedback = bool(
            self.get_ros_parameter("wait_feedback", self.wait_feedback)
        )
        self.publish_sim_joint = bool(
            self.get_ros_parameter("publish_sim_joint", self.publish_sim_joint)
        )
        self.gripper_width_min = float(
            self.get_ros_parameter("gripper_width_min", self.gripper_width_min)
        )
        self.gripper_width_max = float(
            self.get_ros_parameter("gripper_width_max", self.gripper_width_max)
        )
        self.world_to_base_y = float(
            self.get_ros_parameter("world_to_base_y", self.world_to_base_y)
        )
        self.low_press_level = int(
            self.get_ros_parameter("low_press_level", self.low_press_level)
        )
        self.high_press_level = int(
            self.get_ros_parameter("high_press_level", self.high_press_level)
        )
        self.double_press_timeout = float(
            self.get_ros_parameter("double_press_timeout", self.double_press_timeout)
        )
        self.long_press_timeout = float(
            self.get_ros_parameter("long_press_timeout", self.long_press_timeout)
        )
        self.remote_joint_topic_in = str(
            self.get_ros_parameter("remote_joint_topic_in", self.remote_joint_topic_in)
        )
        self.remote_joint_topic_out = str(
            self.get_ros_parameter(
                "remote_joint_topic_out", self.remote_joint_topic_out
            )
        )
        self.remote_service_name = str(
            self.get_ros_parameter("remote_service_name", self.remote_service_name)
        )
        self.node_name_param = str(
            self.get_ros_parameter("node_name", self.node_name_param)
        )
        self.angle_reverse = bool(
            self.get_ros_parameter("angle_reverse", self.angle_reverse)
        )
        self.angle_value = float(
            self.get_ros_parameter("angle_value", self.angle_value)
        )
        self.base_to_imu_roll = float(
            self.get_ros_parameter("base_to_imu_roll", self.base_to_imu_roll)
        )
        self.base_to_imu_pitch = float(
            self.get_ros_parameter("base_to_imu_pitch", self.base_to_imu_pitch)
        )
        self.base_to_imu_yaw = float(
            self.get_ros_parameter("base_to_imu_yaw", self.base_to_imu_yaw)
        )
        self.arm_home_state = str(
            self.get_ros_parameter("arm_home_state", self.arm_home_state)
        )
        self.imu_zero_rpy = str(
            self.get_ros_parameter("imu_zero_rpy", self.imu_zero_rpy)
        )
        self.is_nero = bool(self.get_ros_parameter("is_nero", self.is_nero))
        self.effector_type = str(
            self.get_ros_parameter("effector_type", self.effector_type)
        )
        self.nero_gripper_topic_name = str(
            self.get_ros_parameter(
                "nero_gripper_topic_name", self.nero_gripper_topic_name
            )
        )

        self.add_angle = bool(self.get_ros_parameter("add_angle", self.add_angle))
        self.base_reverse = bool(
            self.get_ros_parameter("base_reverse", self.base_reverse)
        )
        self.imu_reverse = bool(self.get_ros_parameter("imu_reverse", self.imu_reverse))
        self.show_imu_data = bool(
            self.get_ros_parameter("show_imu_data", self.show_imu_data)
        )
        self.use_piper_mode = bool(
            self.get_ros_parameter("use_piper_mode", self.use_piper_mode)
        )
        self.use_pose_dt_zoom = bool(
            self.get_ros_parameter("use_pose_dt_zoom", self.use_pose_dt_zoom)
        )
        self.zoom_factor = float(
            self.get_ros_parameter("zoom_factor", self.zoom_factor)
        )
        self.use_trigger_pose = bool(
            self.get_ros_parameter("use_trigger_pose", self.use_trigger_pose)
        )
        self.gripper_length = float(
            self.get_ros_parameter("gripper_length", self.gripper_length)
        )
        self.handle_seg0 = list(self.get_ros_parameter("handle_seg0", self.handle_seg0))
        self.handle_seg1 = list(self.get_ros_parameter("handle_seg1", self.handle_seg1))
        self.handle_seg2 = list(self.get_ros_parameter("handle_seg2", self.handle_seg2))
        self.handle_seg3 = list(self.get_ros_parameter("handle_seg3", self.handle_seg3))
        self.is_trigger_action = bool(
            self.get_ros_parameter("trigger_action", self.is_trigger_action)
        )
        self.init_action_tmp = bool(
            self.get_ros_parameter("init_action", self.is_init_action)
        )
        self.is_init_action = self.init_action_tmp

        self.get_logger().info("Version: [python] 0.1.0")
        self.base_to_imu_roll = self.base_to_imu_roll / 180.0 * math.pi
        self.base_to_imu_pitch = self.base_to_imu_pitch / 180.0 * math.pi
        self.base_to_imu_yaw = self.base_to_imu_yaw / 180.0 * math.pi
        self.split_ros_params(self.arm_home_state, self.arm_home_state_vec)
        self.split_ros_params(self.imu_zero_rpy, self.imu_zero_rpy_vec)

        self.imu2base_rpy_q = _quat_inv(
            _quat_from_rpy(
                self.base_to_imu_roll, self.base_to_imu_pitch, self.base_to_imu_yaw
            )
        )
        self.imu_diff_quatd = _quat_from_rpy(0.0, 0.0, 0.0)
        self.imu_zero_quatd = _quat_from_rpy(
            self.imu_zero_rpy_vec[0], self.imu_zero_rpy_vec[1], self.imu_zero_rpy_vec[2]
        )
        self.remote_base_pose_matrix = _eye4()

        now = self._now_seconds()
        self.last_get_teleop_enable_time = now
        self.last_call_teleop_time = now

    def dynamic_parameters_callback(self, parameters) -> SetParametersResult:
        result = SetParametersResult(successful=True, reason="success")
        self.is_reconfig_run = True

        for param in parameters:
            handler = self._parameter_setters.get(param.name)
            if handler is not None:
                handler(param.value)

        if self.is_trigger_action != self.trigger_action_tmp:
            self.is_trigger_action = self.trigger_action_tmp
            # A reset-thread sync must not re-push the trigger to the server:
            # the server TOGGLES on every call, so re-pushing from a state sync
            # would toggle it again and feed an oscillation when the server is
            # duplicated or temporarily refusing.
            if (not self._resetting_trigger_action) and (
                self.is_trigger_action != self.teleop_enable
            ):
                if self.teleop_piper_cli is not None:
                    self.last_call_teleop_time = self._now_seconds()
                    self.get_logger().info("call teleop server.")
                    req = Trigger.Request()
                    self.teleop_piper_cli.call_async(req)

            def trigger_motor_feedback() -> None:
                if not self.is_trigger_action:
                    self.set_handle_motor(True)
                    self._sleep(0.15)
                    self.set_handle_motor(False)
                    self._sleep(0.05)
                    self.set_handle_motor(True)
                    self._sleep(0.15)
                    self.set_handle_motor(False)
                else:
                    self.set_handle_motor(True)
                    self._sleep(0.25)
                    self.set_handle_motor(False)

            self._spawn_thread(trigger_motor_feedback)

        if self.init_action_tmp:
            if self.teleop_enable:
                # Same as C++: reject the request instead of silently succeeding.
                self.get_logger().warn(
                    "cannot start home action while teleop is enabled"
                )
                self.init_action_tmp = False
                result.successful = False
                result.reason = "cannot start home action while teleop is enabled"
            elif not self.is_init_action:
                # Claim the running state under the lock so concurrent parameter
                # callbacks (executor + worker threads) cannot double-start homes.
                with self.config_lock:
                    if not self._init_action_running:
                        self._init_action_running = True
                        self.is_init_action = True
                        self._spawn_thread(self._init_action_thread)

        self.is_reconfig_run = False
        return result

    def _init_action_thread(self) -> None:
        self.get_logger().info("start init action thread")
        try:
            # Snapshot the arm feedback under the same lock the callback uses,
            # so joint angles and timestamp come from one consistent state.
            with self._arm_joint_lock:
                arm_joint = JointState()
                arm_joint.name = list(self.last_arm_joint.name)
                arm_joint.position = list(self.last_arm_joint.position)
                arm_joint.velocity = list(self.last_arm_joint.velocity)
                arm_joint.effort = list(self.last_arm_joint.effort)
                arm_joint_time = self.last_arm_joint_time
            get_arm_joint = not self.check_timeout(arm_joint_time, 3.0)

            if not get_arm_joint:
                self.get_logger().error(
                    "no fresh arm joint feedback; aborting home action"
                )
                return
            if (
                not arm_joint.name
                or len(arm_joint.name) != len(arm_joint.position)
                or len(set(arm_joint.name)) != len(arm_joint.name)
            ):
                self.get_logger().error(
                    "invalid feedback JointState name/position layout; aborting home action"
                )
                return
            for position in arm_joint.position:
                if not math.isfinite(float(position)):
                    self.get_logger().error(
                        "feedback JointState contains non-finite position; aborting home action"
                    )
                    return
            if len(self.arm_home_state_vec) != len(arm_joint.position):
                self.get_logger().error(
                    f"arm_home_state length {len(self.arm_home_state_vec)} does not match "
                    f"feedback joint length {len(arm_joint.position)}; aborting home action"
                )
                return
            if self.teleop_enable:
                self.get_logger().warn("teleop enabled; canceling home action")
                return

            self.set_handle_motor(True)
            init_time = 0.6
            init_rate = 50.0
            init_steps = max(1, int(init_rate * init_time))
            angle_dt = []
            for i, pos in enumerate(arm_joint.position):
                base = (
                    self.arm_home_state_vec[i]
                    if i < len(self.arm_home_state_vec)
                    else 0.0
                )
                angle_dt.append((float(pos) - base) / init_steps)
            period = 1.0 / init_rate

            for _ in range(init_steps):
                if self.teleop_enable:
                    self.get_logger().warn("teleop enabled; canceling home action")
                    self.set_handle_motor(False)
                    return
                for j in range(len(arm_joint.position)):
                    arm_joint.position[j] -= angle_dt[j]
                if self.remote_joint_publisher is not None:
                    with self._remote_publish_lock:
                        if self.teleop_enable:
                            self.get_logger().warn(
                                "teleop enabled; canceling home action"
                            )
                            self.set_handle_motor(False)
                            return
                        self.remote_joint_publisher.publish(arm_joint)
                self._sleep(period)

            if self.teleop_enable:
                self.get_logger().warn("teleop enabled; canceling home action")
                self.set_handle_motor(False)
                return
            arm_joint.position = list(self.arm_home_state_vec)
            if self.remote_joint_publisher is not None:
                with self._remote_publish_lock:
                    if self.teleop_enable:
                        self.get_logger().warn("teleop enabled; canceling home action")
                        self.set_handle_motor(False)
                        return
                    self.remote_joint_publisher.publish(arm_joint)
            self.set_handle_motor(False)
        finally:
            with self.config_lock:
                self._init_action_running = False
                self.is_init_action = False
                self.set_parameters(
                    [rclpy.parameter.Parameter("init_action", value=False)]
                )
                self.get_logger().info("stop init action thread")

    def enable_handle_motor_server(self, request, response) -> SetBool.Response:
        if not self.tego.is_connected():
            response.success = False
            response.message = "TEGO CAN is not connected"
            return response
        self.set_handle_motor(bool(request.data))
        response.success = True
        response.message = "success"
        return response

    def reset_pose_server(self, request, response) -> Empty.Response:
        _ = request
        self.update_imu = True
        self.get_imu = False
        self.update_joint = True
        return response

    def reset_control_server(self, request, response) -> ResetControl.Response:
        if not self.tego.is_connected():
            response.success = False
            response.message = "TEGO CAN is not connected"
            return response
        reset_mode = (
            ResetMode.FACTORY
            if request.reset_mode == ResetControl.Request.RESET_FACTORY
            else ResetMode.ZERO
        )
        self.tego.set_reset_command(MtResetCmd(reset_mode=reset_mode))
        response.success = True
        response.message = "reset command sent"
        return response

    def enable_reverse_server(self, request, response) -> SetBool.Response:
        self.get_logger().info(
            f"get reverse request: {'true' if request.data else 'false'}"
        )
        if self.base_reverse != bool(request.data) and self.get_imu:
            self.get_logger().info("reverse imu.")
            last_quatd = self.imu2base_rpy_q
            yaw = self.base_to_imu_yaw + (math.pi if request.data else 0.0)
            self.imu2base_rpy_q = _quat_inv(
                _quat_from_rpy(self.base_to_imu_roll, self.base_to_imu_pitch, yaw)
            )
            self.imu_base_quatd = _quat_mul(
                _quat_mul(self.imu_base_quatd, _quat_inv(last_quatd)),
                self.imu2base_rpy_q,
            )
        self.base_reverse = bool(request.data)
        response.success = True
        response.message = "success."
        return response

    def remote_joint_callback(self, msg: JointState) -> None:
        self.last_remote_joint = msg
        self.last_remote_joint.header.stamp = self.get_clock().now().to_msg()
        if (not self.is_nero) and len(self.last_remote_joint.position) >= 7:
            positions = list(self.last_remote_joint.position)
            positions[6] = self.get_gripper_width()
            self.last_remote_joint.position = positions
        if self.remote_joint_publisher is None:
            return
        # Same as C++: publish under the shared lock and re-check teleop_enable
        # so home interpolation commands cannot interleave with live teleop.
        with self._remote_publish_lock:
            if self.teleop_enable:
                self.remote_joint_publisher.publish(self.last_remote_joint)

    def arm_joint_callback(self, msg: JointState) -> None:
        with self._arm_joint_lock:
            self.last_arm_joint = msg
            self.last_arm_joint.header.stamp = self.get_clock().now().to_msg()
            self.last_arm_joint_time = self._now_seconds()

    def teleop_enable_callback(self, msg: Bool) -> None:
        with self._teleop_state_lock:
            self.last_get_teleop_enable_time = self._now_seconds()
            self.teleop_enable = bool(msg.data)

    def imu_callback(self, msg: Imu) -> None:
        if self.show_imu_data:
            roll, pitch, yaw = _quat_to_rpy(self.get_imu_quaternion(msg))
            self.get_logger().info(
                f"({roll * 180.0 / math.pi:.2f}, "
                f"{pitch * 180.0 / math.pi:.2f}, "
                f"{yaw * 180.0 / math.pi:.2f})"
            )
        self.imu_converter(msg)
        self.get_imu = True
        if self.update_imu:
            self.update_imu = False
            self.imu_base_quatd = self.get_imu_quaternion(msg)
            self.update_base_pose = True

        cur_imu_quatd = self.get_imu_quaternion(msg)
        self.imu_diff_quatd = _quat_mul(_quat_inv(self.imu_base_quatd), cur_imu_quatd)
        self.publish_tf(
            _to_transform_xyz_quat(0.0, 0.0, 0.0, self.imu_diff_quatd),
            "imu_base",
            "imu_diff",
        )

    def imu_converter(self, input_msg: Imu) -> None:
        q_from = self.get_imu_quaternion(input_msg)
        q_final = _quat_mul(q_from, self.imu2base_rpy_q)
        input_msg.orientation.w = q_final[0]
        input_msg.orientation.x = q_final[1]
        input_msg.orientation.y = q_final[2]
        input_msg.orientation.z = q_final[3]

    def get_imu_quaternion(self, imu: Imu):
        return (
            imu.orientation.w,
            imu.orientation.x,
            imu.orientation.y,
            imu.orientation.z,
        )

    def publish_tf(self, transform, frame_id: str, child_frame_id: str) -> None:
        rot = [[transform[i][j] for j in range(3)] for i in range(3)]
        q = _rot_to_quat(rot)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = frame_id
        tf_msg.child_frame_id = child_frame_id
        tf_msg.transform.translation.x = transform[0][3]
        tf_msg.transform.translation.y = transform[1][3]
        tf_msg.transform.translation.z = transform[2][3]
        tf_msg.transform.rotation.w = q[0]
        tf_msg.transform.rotation.x = q[1]
        tf_msg.transform.rotation.y = q[2]
        tf_msg.transform.rotation.z = q[3]
        self.tf_broadcaster.sendTransform(tf_msg)

    def update_and_publish_pose(self) -> None:
        world_to_joint_base_yaw = math.pi if self.base_reverse else 0.0
        angle_reverse_k = -1 if self.angle_reverse else 1
        angle0 = -(self.angle_list[1] - self.angle0_base)
        angle1 = angle_reverse_k * (self.angle_list[2] - self.angle1_base)
        angle2 = -angle_reverse_k * (self.angle_list[3] - self.angle2_base) + (
            self.angle_value if self.add_angle else 0.0
        )
        gripper_pos = self.get_gripper_width() / 2.0

        cur_imu_diff_quatd = _quat_mul(self.imu_zero_quatd, self.imu_diff_quatd)
        if self.imu_reverse:
            diff_roll, diff_pitch, diff_yaw = _quat_to_rpy(self.imu_diff_quatd)
            cur_imu_diff_quatd = _quat_from_rpy(-diff_roll, -diff_pitch, diff_yaw)

        world_to_joint_base_matrix = _to_transform_xyz_rpy(
            0.0, self.world_to_base_y, 0.0, 0.0, 0.0, world_to_joint_base_yaw
        )
        gripper_length_matrix = _to_transform_xyz_rpy(self.gripper_length)
        gripper_left_matrix = _to_transform_xyz_rpy(self.gripper_length, gripper_pos)
        gripper_right_matrix = _to_transform_xyz_rpy(
            self.gripper_length, -gripper_pos, 0.0, math.pi
        )

        # Same as C++: handle DH from handle_seg0..3 parameters.
        # seg0 yaw + angle0; seg1 pitch - angle1; seg2 yaw + angle2; seg3 z * angle_reverse_k.
        seg_zoom = (
            1.0
            if self.use_piper_mode
            else (1.0 if self.use_pose_dt_zoom else self.zoom_factor)
        )
        s0, s1, s2, s3 = (
            self.handle_seg0,
            self.handle_seg1,
            self.handle_seg2,
            self.handle_seg3,
        )
        joint_base_to_joint0_matrix = _to_transform_xyz_rpy(
            s0[0], s0[1], s0[2], s0[3], s0[4], angle0 + s0[5]
        )
        joint0_to_joint1_matrix = _to_transform_xyz_rpy(
            s1[0], s1[1], s1[2], s1[3], s1[4] - angle1, s1[5]
        )
        joint1_to_joint2_matrix = _to_transform_xyz_rpy(
            s2[0], s2[1], s2[2], s2[3], s2[4], angle2 + s2[5], seg_zoom
        )
        joint2_to_joint3_matrix = _to_transform_xyz_rpy(
            s3[0], s3[1], s3[2] * angle_reverse_k, s3[3], s3[4], s3[5], seg_zoom
        )

        # Same behavior as C++: joint3/4/5 matrices are identity (v3 override).
        joint3_to_joint4_matrix = _eye4()
        joint4_to_joint5_matrix = _eye4()

        pose_matrix = _mat4_mul(
            _mat4_mul(
                _mat4_mul(
                    _mat4_mul(
                        _mat4_mul(
                            world_to_joint_base_matrix, joint_base_to_joint0_matrix
                        ),
                        joint0_to_joint1_matrix,
                    ),
                    joint1_to_joint2_matrix,
                ),
                joint2_to_joint3_matrix,
            ),
            joint3_to_joint4_matrix,
        )
        imu_diff_matrix = _to_transform_xyz_quat(
            pose_matrix[0][3], pose_matrix[1][3], pose_matrix[2][3], cur_imu_diff_quatd
        )
        imu_diff_matrix = _mat4_mul(_mat4_inv_rigid(pose_matrix), imu_diff_matrix)
        pose_matrix = _mat4_mul(
            _mat4_mul(_mat4_mul(pose_matrix, imu_diff_matrix), joint4_to_joint5_matrix),
            gripper_length_matrix,
        )

        prefix = self.node_name_param
        self.publish_tf(world_to_joint_base_matrix, "world", f"{prefix}_joint_base")
        self.publish_tf(
            joint_base_to_joint0_matrix, f"{prefix}_joint_base", f"{prefix}_joint0"
        )
        self.publish_tf(joint0_to_joint1_matrix, f"{prefix}_joint0", f"{prefix}_joint1")
        self.publish_tf(joint1_to_joint2_matrix, f"{prefix}_joint1", f"{prefix}_joint2")
        self.publish_tf(joint2_to_joint3_matrix, f"{prefix}_joint2", f"{prefix}_joint3")
        self.publish_tf(joint3_to_joint4_matrix, f"{prefix}_joint3", f"{prefix}_joint4")
        self.publish_tf(imu_diff_matrix, f"{prefix}_joint4", f"{prefix}_joint_end")
        self.publish_tf(
            joint4_to_joint5_matrix, f"{prefix}_joint_end", f"{prefix}_joint_trigger"
        )
        if self.gripper_length > 0.0:
            self.publish_tf(
                gripper_length_matrix,
                f"{prefix}_joint_trigger",
                f"{prefix}_joint_gripper",
            )
            self.publish_tf(
                gripper_left_matrix,
                f"{prefix}_joint_trigger",
                f"{prefix}_joint_gripper_left",
            )
            self.publish_tf(
                gripper_right_matrix,
                f"{prefix}_joint_trigger",
                f"{prefix}_joint_gripper_right",
            )

        self.cur_pose_matrix = pose_matrix
        self.publish_remote_pose()

    def publish_remote_pose(self) -> None:
        if self.update_base_pose:
            self.remote_pose_first_init = False
            self.update_base_pose = False
            self.remote_base_pose_matrix = self.cur_pose_matrix
        if self.remote_pose_first_init:
            return

        cur_zoom_factor = (
            self.zoom_factor
            if ((not self.use_piper_mode) and self.use_pose_dt_zoom)
            else 1.0
        )
        pose_matrix = [row[:] for row in self.remote_base_pose_matrix]
        dt_matrix = _mat4_mul(
            _mat4_inv_rigid(self.remote_base_pose_matrix), self.cur_pose_matrix
        )
        for i in range(3):
            dt_matrix[i][3] *= cur_zoom_factor
        pose_matrix = _mat4_mul(pose_matrix, dt_matrix)
        q = _rot_to_quat([[pose_matrix[i][j] for j in range(3)] for i in range(3)])

        self.publish_tf(pose_matrix, "world", f"{self.node_name_param}_gripper_pose")
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = pose_matrix[0][3]
        pose_msg.pose.position.y = pose_matrix[1][3]
        pose_msg.pose.position.z = pose_matrix[2][3]
        pose_msg.pose.orientation.w = q[0]
        pose_msg.pose.orientation.x = q[1]
        pose_msg.pose.orientation.y = q[2]
        pose_msg.pose.orientation.z = q[3]
        if self.handle_pose_publisher is not None:
            self.handle_pose_publisher.publish(pose_msg)

    def analyze_key_event_action(self, key_id: int, click_type: int) -> None:
        """Side key gesture, aligned with the C++ AnalyzeKeyEventAction.

        KEY_LEFT = 0x01, KEY_RIGHT = 0x02; CLICK_DOUBLE = 0x02, CLICK_LONG = 0x03.
        Left handle toggles teleop with the RIGHT (inner) key; right handle with
        the LEFT (inner) key. Double click toggles teleop; long press returns home.
        """
        KEY_LEFT = 0x01
        KEY_RIGHT = 0x02
        CLICK_DOUBLE = 0x02
        CLICK_LONG = 0x03

        is_left_handle = (
            "left" in self.node_name_param or "_l" in self.node_name_param
        )
        is_right_handle = (
            "right" in self.node_name_param or "_r" in self.node_name_param
        )
        is_inside_key = (
            (not is_left_handle and not is_right_handle)
            or (is_left_handle and key_id == KEY_RIGHT)
            or (is_right_handle and key_id == KEY_LEFT)
        )
        if not is_inside_key:
            return

        if click_type == CLICK_DOUBLE:
            if self.teleop_enable:
                self._sleep(self.double_press_timeout)

            def toggle_teleop() -> None:
                next_action = not self.is_trigger_action
                self.get_logger().info(f"[side key trigger] -> {'true' if next_action else 'false'}")
                self.set_parameters([rclpy.parameter.Parameter("trigger_action", value=next_action)])

            self._spawn_thread(toggle_teleop)
            return

        if click_type == CLICK_LONG:
            if not self.teleop_enable:
                self.get_logger().info("[side key init]")

                def home_action() -> None:
                    self._sleep(self.double_press_timeout)
                    if not self.teleop_enable:
                        self.set_parameters([rclpy.parameter.Parameter("init_action", value=True)])
                    else:
                        self.get_logger().info("[side key init] canceled because teleop is enabled")

                self._spawn_thread(home_action)
            else:
                print("pass")
            return

    def check_timeout(self, timestamp: float, timeout: float = 1.0) -> bool:
        return abs(self._now_seconds() - timestamp) >= timeout

    def split_ros_params(self, data: str, vec: list[float]) -> None:
        """Strict parsing, same failure semantics as the C++ SplitRosParams."""
        vec.clear()
        if not data:
            raise ValueError("parameter must not be empty")
        for token in data.split(","):
            trimmed = token.strip()
            if not trimmed:
                raise ValueError("parameter contains an empty value")
            try:
                value = float(trimmed)
            except ValueError:
                raise ValueError(
                    f"parameter contains a non-numeric value: {trimmed}"
                )
            if not math.isfinite(value):
                raise ValueError(
                    "parameter must contain only finite values: " + trimmed
                )
            vec.append(value)
        if not vec or data.endswith(","):
            raise ValueError("parameter must contain at least one value")

    def set_handle_motor(self, run: bool = False) -> None:
        mode = MotorMode.START if run else MotorMode.STOP
        self.tego.set_handle_motor_command(HandleMotorControlCmd(motor_mode=mode))

    def set_handle_light(self, color: LightColorId) -> None:
        self.tego.set_handle_light_command(HandleLightControlCmd(color_id=color))

    def reset_teleop_enable_thread(self) -> None:
        timeout = 1.0
        while rclpy.ok() and (not self._stop_threads.is_set()):
            with self._teleop_state_lock:
                if self.teleop_enable and self.check_timeout(
                    self.last_get_teleop_enable_time, timeout
                ):
                    with self._remote_publish_lock:
                        # Re-check after acquiring the publish lock so a concurrent
                        # teleop-enable callback cannot be overwritten by a stale timeout.
                        if self.teleop_enable and self.check_timeout(
                            self.last_get_teleop_enable_time, timeout
                        ):
                            self.get_logger().info("can't get teleop_enable status")
                            self.teleop_enable = False
            self._sleep(0.05)

    def reset_trigger_action_thread(self) -> None:
        while rclpy.ok() and (not self._stop_threads.is_set()):
            if (
                (not self.is_reconfig_run)
                and self.check_timeout(self.last_call_teleop_time, 0.5)
                and (self.is_trigger_action != self.teleop_enable)
            ):
                # Capture once, like the C++ thread. This sync only updates the
                # local state; it must not call the teleop server (see callback).
                next_trigger_action = not self.is_trigger_action
                self.get_logger().info(
                    f"[reset trigger] -> "
                    f"{'true' if next_trigger_action else 'false'}"
                )
                self._resetting_trigger_action = True
                try:
                    self.set_parameters(
                        [
                            rclpy.parameter.Parameter(
                                "trigger_action", value=next_trigger_action
                            )
                        ]
                    )
                finally:
                    self._resetting_trigger_action = False
                # Gate our own flips so a persistent mismatch (e.g. a duplicated
                # teleop server) cannot spin at the loop rate.
                self.last_call_teleop_time = self._now_seconds()
            self._sleep(0.05)

    def get_gripper_width(self) -> float:
        width = (self.gripper_width_max - self.gripper_width_min) / 100.0 * (
            100.0 - self.trigger_press_percentage
        ) + self.gripper_width_min
        return _clamp_min(width, 0.0)

    def publish_state_to_ros(self) -> None:
        state = self.tego.get_tego_state()
        now_msg = self.get_clock().now().to_msg()

        if self.publish_sim_joint and self.sim_joint_publisher is not None:
            sim_msg = JointState()
            sim_msg.position = [0.0] * 7
            self.sim_joint_publisher.publish(sim_msg)

        if (not self.check_get) or (state.mt_sensor_get != self.mt_sensor_send):
            self.mt_sensor_send = list(state.mt_sensor_get)
            joint_msg = JointState()
            status_msg = MtSensorStatus()
            for i in range(7):
                name = f"joint{i}"
                joint_msg.name.append(name)
                joint_msg.position.append(state.mt_sensor_states[i].angle_rad)
                self.angle_list[i] = (
                    state.mt_sensor_states[i].angle_deg / 180.0 * math.pi
                )

                sensor_data = SensorData()
                sensor_data.angle_deg = state.mt_sensor_states[i].angle_deg
                sensor_data.angle_rad = state.mt_sensor_states[i].angle_rad
                sensor_data.voltage_x = state.mt_sensor_states[i].voltage_x
                sensor_data.voltage_y = state.mt_sensor_states[i].voltage_y
                status_msg.name.append(name)
                status_msg.data.append(sensor_data)
                status_msg.receive_count.append(int(state.mt_sensor_get[i]))
            if self.update_joint:
                self.update_joint = False
                self.angle0_base = self.angle_list[1]
                self.angle1_base = self.angle_list[2]
                self.angle2_base = self.angle_list[3]
            if self.joint_publisher is not None:
                self.joint_publisher.publish(joint_msg)
            if self.sensor_status_publisher is not None:
                self.sensor_status_publisher.publish(status_msg)

        if (not self.check_get) or (
            self.handle_status_send != state.handle_status_get
            or self.handle_calibration_send != state.handle_calibration_get
            or self.key1_send != state.key1_get
            or self.key2_send != state.key2_get
        ):
            self.handle_status_send = state.handle_status_get
            self.handle_calibration_send = state.handle_calibration_get
            self.key1_send = state.key1_get
            self.key2_send = state.key2_get
            handle_msg = HandleState()
            handle_msg.joystick_x = state.handle_state.x
            handle_msg.joystick_y = state.handle_state.y
            handle_msg.trigger_press_percentage = state.handle_state.trigger
            handle_msg.handle_receive_count = state.handle_status_get
            handle_msg.key1_click_type = state.key1_click_type
            handle_msg.key2_click_type = state.key2_click_type
            handle_msg.key1_receive_count = state.key1_get
            handle_msg.key2_receive_count = state.key2_get
            handle_msg.calibration_receive_count = state.handle_calibration_get
            handle_msg.calibration_x = state.handle_calibration_state.x
            handle_msg.calibration_y = state.handle_calibration_state.y
            handle_msg.calibration_trigger = state.handle_calibration_state.trigger
            handle_msg.calibration_all = state.handle_calibration_state.all
            self.trigger_press_percentage = state.handle_state.trigger
            if self.handle_state_publisher is not None:
                self.handle_state_publisher.publish(handle_msg)

            cur_length = self.get_gripper_width()
            if (
                self.effector_type == "agx_gripper"
                and self.nero_gripper_publisher is not None
                and abs(self.last_gripper_length - cur_length) > 0.003
            ):
                self.last_gripper_length = cur_length
                gripper_msg = JointState()
                gripper_msg.header.stamp = now_msg
                gripper_msg.name = ["gripper"]
                gripper_msg.position = [cur_length]
                gripper_msg.effort = [3.0]
                self.nero_gripper_publisher.publish(gripper_msg)

        # Side key event (double click / long press), same as C++.
        if self.key_event_send != state.key_event_get:
            self.key_event_send = state.key_event_get
            self.analyze_key_event_action(
                state.key_event_state.key_id, state.key_event_state.click_type
            )

        imu_init = (
            state.imu_linear_acceleration_get != 0
            or state.imu_angular_velocity_get != 0
            or state.imu_orientation_get != 0
        )
        imu_get = (
            self.imu_linear_acceleration_send != state.imu_linear_acceleration_get
            or self.imu_angular_velocity_send != state.imu_angular_velocity_get
            or self.imu_orientation_send != state.imu_orientation_get
        )
        if (not self.check_get) or (imu_init and imu_get):
            self.imu_linear_acceleration_send = state.imu_linear_acceleration_get
            self.imu_angular_velocity_send = state.imu_angular_velocity_get
            self.imu_orientation_send = state.imu_orientation_get

            imu_msg = Imu()
            imu_msg.header.stamp = now_msg
            imu_msg.header.frame_id = "base_link"
            imu_msg.linear_acceleration.x = state.imu_linear_acceleration_state.x
            imu_msg.linear_acceleration.y = state.imu_linear_acceleration_state.y
            imu_msg.linear_acceleration.z = state.imu_linear_acceleration_state.z
            imu_msg.angular_velocity.x = state.imu_angular_velocity_state.x
            imu_msg.angular_velocity.y = state.imu_angular_velocity_state.y
            imu_msg.angular_velocity.z = state.imu_angular_velocity_state.z
            imu_msg.orientation.w = state.imu_orientation_state.w
            imu_msg.orientation.x = state.imu_orientation_state.x
            imu_msg.orientation.y = state.imu_orientation_state.y
            imu_msg.orientation.z = state.imu_orientation_state.z

            if self.imu_publisher is not None:
                self.imu_publisher.publish(imu_msg)
            self.imu_callback(imu_msg)

        if not self.update_joint:
            self.update_and_publish_pose()

    def destroy_node(self) -> bool:
        self._stop_threads.set()
        for thread in self._threads:
            if thread.is_alive():
                thread.join(timeout=0.2)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    tego = TegoBase()
    node = TegoBridgeNode(tego, "tego_bridge_node")

    try:
        port_name = str(node.get_ros_parameter("port_name", "can0"))
        pub_rate = float(node.get_ros_parameter("pub_rate", 200.0))
        node.get_logger().info(f"can_port: {port_name}")
        try:
            tego.connect(port_name)
        except Exception as exc:
            # Same as C++: keep the ROS interface alive; the device is unavailable.
            node.get_logger().error(
                f"Failed to open CAN interface: {port_name}: {exc}"
            )
        node.setup_subscription()
        time.sleep(0.5)
        node.start_publish_loop(pub_rate if pub_rate > 0.0 else 200.0)
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        tego.disconnect()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
