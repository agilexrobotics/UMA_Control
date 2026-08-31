#!/usr/bin/env python3
import os
import math
import time
import numpy as np
import scipy.spatial.transform

import casadi
import pinocchio as pin
import meshcat.geometry as mg
from pinocchio import casadi as cpin
# from oculus_reader import OculusReader
from pinocchio.visualize import MeshcatVisualizer

#ros2 department
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
try:
    from .transformations import quaternion_from_euler, quaternion_from_matrix, euler_from_quaternion
except ImportError:  # Support execution as the installed ROS script.
    from transformations import quaternion_from_euler, quaternion_from_matrix, euler_from_quaternion
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

def xyzquat2Mat(pos, quat):
    # 如果输入是列表，直接赋值
    # pos 预期为 [x, y, z]
    # quat 预期为 [x, y, z, w]
    p = pos
    q = quat

    rotation_matrix = scipy.spatial.transform.Rotation.from_quat(q).as_matrix()

    matrix = np.eye(4)
    matrix[:3, :3] = rotation_matrix
    matrix[:3, 3] = p

    return matrix

def matrix2xyzrpy(matrix):
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    roll = np.arctan2(matrix[2, 1], matrix[2, 2])
    pitch = np.arcsin(-matrix[2, 0])
    yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
    return [x, y, z, roll, pitch, yaw]

def xyzrpy2Mat(x, y, z, roll, pitch, yaw):
    transformation_matrix = np.eye(4)
    A = np.cos(yaw)
    B = np.sin(yaw)
    C = np.cos(pitch)
    D = np.sin(pitch)
    E = np.cos(roll)
    F = np.sin(roll)
    DE = D * E
    DF = D * F
    transformation_matrix[0, 0] = A * C
    transformation_matrix[0, 1] = A * DF - B * E
    transformation_matrix[0, 2] = B * F + A * DE
    transformation_matrix[0, 3] = x
    transformation_matrix[1, 0] = B * C
    transformation_matrix[1, 1] = A * E + B * DF
    transformation_matrix[1, 2] = B * DE - A * F
    transformation_matrix[1, 3] = y
    transformation_matrix[2, 0] = -D
    transformation_matrix[2, 1] = C * F
    transformation_matrix[2, 2] = C * E
    transformation_matrix[2, 3] = z
    transformation_matrix[3, 0] = 0
    transformation_matrix[3, 1] = 0
    transformation_matrix[3, 2] = 0
    transformation_matrix[3, 3] = 1
    return transformation_matrix

def calc_pose_incre(start_pose_matrix, current_pose_matrix, zero_matrix):
    end_matrix = xyzrpy2Mat(current_pose_matrix[0], current_pose_matrix[1], current_pose_matrix[2],
                                            current_pose_matrix[3], current_pose_matrix[4], current_pose_matrix[5])
    result_matrix = np.dot(zero_matrix, np.dot(np.linalg.inv(start_pose_matrix), end_matrix))
    xyzrpy = matrix2xyzrpy(result_matrix)
    return xyzrpy

class Arm_IK:
    def __init__(
        self,
        logger=None,
        max_delta_q_rad=None,
        enable_hard_delta_q_constraint=True,
        enable_psi_constraint=True,
        psi_weight=30.0,
        psi_eps=0.05,
        enable_joint_reference_penalty=False,
        joint_reference_weight=0.0,
        joint_reference_mask=None,
        enable_elbow_side_penalty=False,
        elbow_side_penalty_weight=0.0,
        elbow_side_penalty_sharpness=15.0,
        enable_virtual_wall=True,
        virtual_wall_x=-0.42,
        virtual_wall_clearance=0.01,
    ):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        # old
        # package_path = get_package_share_directory('nero_description')
        # urdf_path = os.path.join(package_path, 'urdf', 'nero.urdf')
        # new   tego_description/urdf/nero/urdf/nero_description.urdf
        package_path = get_package_share_directory('tego_description')
        urdf_path = os.path.join(package_path, 'urdf', 'nero', 'urdf', 'nero_description.urdf')

        self.robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path,
            package_dirs=package_path
        )

        self.mixed_jointsToLockIDs = ["joint8",]

        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0] * self.robot.model.nq),
        )

        #self.reduced_robot = self.robot

        # self.first_matrix = xyzrpy2Mat(0, 0, 0, -1.57, 0, -1.57)
        # self.second_matrix = xyzrpy2Mat(0.0, 0.023, 0.064, 0, 0, 0)  #第7轴到手腕末端坐标的变换矩阵  0.034--->0.064  手腕末端到手掌中心点的偏移
        # -1.571 0 0
        # 0.000, 0.023, 0.000
        self.first_matrix = xyzrpy2Mat(0, 0, 0, -1.57, 0, 0)
        self.second_matrix = xyzrpy2Mat(0.0, 0.023, 0, 0, 0, 0)

        self.last_matrix = np.dot(self.first_matrix, self.second_matrix)
        q = quaternion_from_matrix(self.last_matrix)
        self.reduced_robot.model.addFrame(
            pin.Frame('ee',

                      self.reduced_robot.model.getJointId('joint7'),
                      pin.SE3(
                          # pin.Quaternion(1, 0, 0, 0),
                          pin.Quaternion(q[3], q[0], q[1], q[2]),
                          np.array([self.last_matrix[0, 3], self.last_matrix[1, 3], self.last_matrix[2, 3]]),  # -y
                      ),
                      pin.FrameType.OP_FRAME)
        )

        self.geom_model = pin.buildGeomFromUrdf(self.robot.model, urdf_path, pin.GeometryType.COLLISION)
        for i in range(5,6):
            for j in range(0, 4):
                self.geom_model.addCollisionPair(pin.CollisionPair(i, j))
        self.geometry_data = pin.GeometryData(self.geom_model)

        self.init_data = np.zeros(self.reduced_robot.model.nq) #  np.zeros(self.reduced_robot.model.nq)
        self.history_data = np.zeros(self.reduced_robot.model.nq)
        if max_delta_q_rad is None:
            self.max_delta_q_rad = np.array([0.15] * self.reduced_robot.model.nq, dtype=float)
        else:
            self.max_delta_q_rad = np.array(max_delta_q_rad, dtype=float).reshape(-1)
        if self.max_delta_q_rad.size == 1:
            self.max_delta_q_rad = np.repeat(self.max_delta_q_rad, self.reduced_robot.model.nq)
        if self.max_delta_q_rad.size != self.reduced_robot.model.nq:
            raise ValueError(
                f"max_delta_q_rad must be scalar or {self.reduced_robot.model.nq} elements, got {self.max_delta_q_rad.size}"
            )
        self.enable_hard_delta_q_constraint = bool(enable_hard_delta_q_constraint)
        self.enable_psi_constraint = bool(enable_psi_constraint)
        self._psi_weight = float(psi_weight)
        self._psi_eps = float(psi_eps)
        self._enable_joint_reference_penalty = bool(enable_joint_reference_penalty)
        self._joint_reference_weight = float(joint_reference_weight)
        if joint_reference_mask is None:
            self._joint_reference_mask = np.zeros(self.reduced_robot.model.nq, dtype=float)
        else:
            self._joint_reference_mask = np.array(joint_reference_mask, dtype=float).reshape(-1)
        if self._joint_reference_mask.size == 1:
            self._joint_reference_mask = np.repeat(
                self._joint_reference_mask, self.reduced_robot.model.nq
            )
        if self._joint_reference_mask.size != self.reduced_robot.model.nq:
            raise ValueError(
                "joint_reference_mask must be scalar or "
                f"{self.reduced_robot.model.nq} elements, got {self._joint_reference_mask.size}"
            )
        if np.any(self._joint_reference_mask < 0.0):
            raise ValueError("joint_reference_mask must be non-negative")
        if self._psi_eps <= 0.0:
            raise ValueError("psi_eps must be > 0")
        self._enable_elbow_side_penalty = bool(enable_elbow_side_penalty)
        self._elbow_side_penalty_weight = float(elbow_side_penalty_weight)
        self._elbow_side_penalty_sharpness = float(elbow_side_penalty_sharpness)
        if self._elbow_side_penalty_sharpness <= 0.0:
            raise ValueError("elbow_side_penalty_sharpness must be > 0")

        # Virtual wall (vertical plane) constraint:
        # keep key points in front of x = virtual_wall_x + virtual_wall_clearance.
        self._enable_virtual_wall = bool(enable_virtual_wall)
        self._virtual_wall_x = float(virtual_wall_x)
        self._virtual_wall_clearance = float(virtual_wall_clearance)
        # self._virtual_wall_log_period_s = 0.2
        # self._last_virtual_wall_log_time = 0.0
        if self._virtual_wall_clearance < 0.0:
            raise ValueError("virtual_wall_clearance must be >= 0")
        self._shoulder_jid = self.reduced_robot.model.getJointId("joint1")
        self._elbow_jid = self.reduced_robot.model.getJointId("joint4")
        self._wrist_jid = self.reduced_robot.model.getJointId("joint5")
        self._logger = logger

        # # Initialize the Meshcat visualizer  for visualization
        self.vis = MeshcatVisualizer(self.reduced_robot.model, self.reduced_robot.collision_model, self.reduced_robot.visual_model)
        self.vis.initViewer(open=False)
        self.vis.loadViewerModel("pinocchio")
        self.vis.displayFrames(True, frame_ids=[113, 114], axis_length=0.15, axis_width=5)
        self.vis.display(pin.neutral(self.reduced_robot.model))

        # Enable the display of end effector target frames with short axis lengths and greater width.
        frame_viz_names = ['ee_target']
        FRAME_AXIS_POSITIONS = (
            np.array([[0, 0, 0], [1, 0, 0],
                      [0, 0, 0], [0, 1, 0],
                      [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
        )
        FRAME_AXIS_COLORS = (
            np.array([[1, 0, 0], [1, 0.6, 0],
                      [0, 1, 0], [0.6, 1, 0],
                      [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
        )
        axis_length = 0.1
        axis_width = 10
        for frame_viz_name in frame_viz_names:
            self.vis.viewer[frame_viz_name].set_object(
                mg.LineSegments(
                    mg.PointsGeometry(
                        position=axis_length * FRAME_AXIS_POSITIONS,
                        color=FRAME_AXIS_COLORS,
                    ),
                    mg.LineBasicMaterial(
                        linewidth=axis_width,
                        vertexColors=True,
                    ),
                )
            )

        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.cTf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        _c_shoulder = self.cdata.oMi[self._shoulder_jid].translation
        _c_elbow = self.cdata.oMi[self._elbow_jid].translation
        _c_wrist = self.cdata.oMi[self._wrist_jid].translation
        _c_sw = _c_wrist - _c_shoulder
        _c_sw_norm_sq = casadi.dot(_c_sw, _c_sw) + 1e-8
        _c_se = _c_elbow - _c_shoulder
        _c_elbow_perp = _c_se - (casadi.dot(_c_se, _c_sw) / _c_sw_norm_sq) * _c_sw
        _c_ep_norm = casadi.sqrt(casadi.dot(_c_elbow_perp, _c_elbow_perp) + self._psi_eps ** 2)
        _c_elbow_perp_unit = _c_elbow_perp / _c_ep_norm
        self._elbow_perp_fn = casadi.Function(
            "elbow_perp", [self.cq], [_c_elbow_perp]
        )
        self._elbow_perp_unit_fn = casadi.Function(
            "elbow_perp_unit", [self.cq], [_c_elbow_perp_unit]
        )

        # # Get the hand joint ID and define the error function
        self.gripper_id = self.reduced_robot.model.getFrameId("ee")
        self.error = casadi.Function(
            "error",
            [self.cq, self.cTf],
            [
                casadi.vertcat(
                    cpin.log6(
                        self.cdata.oMf[self.gripper_id].inverse() * cpin.SE3(self.cTf)
                    ).vector,
                )
            ],
        )

        # Key point positions for virtual wall constraints.
        self._ee_pos_fn = casadi.Function(
            "ee_pos", [self.cq], [self.cdata.oMf[self.gripper_id].translation]
        )
        self._wrist_pos_fn = casadi.Function(
            "wrist_pos", [self.cq], [self.cdata.oMi[self._wrist_jid].translation]
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.param_q_prev = self.opti.parameter(self.reduced_robot.model.nq)   # 用于平滑，防止多解跳变
        self.param_q_ref = self.opti.parameter(self.reduced_robot.model.nq)
        self.param_tf = self.opti.parameter(4, 4)
        self.param_elbow_perp = self.opti.parameter(3)
        self.param_elbow_side_sign = self.opti.parameter()
        self.param_elbow_side_normal = self.opti.parameter(3)
        self.opti.set_value(self.param_elbow_perp, np.zeros(3))
        self.opti.set_value(self.param_elbow_side_sign, 0.0)
        self.opti.set_value(self.param_elbow_side_normal, np.zeros(3))

        # self.totalcost = casadi.sumsqr(self.error(self.var_q, self.param_tf))
        # self.regularization = casadi.sumsqr(self.var_q)

        error_vec = self.error(self.var_q, self.param_tf)
        pos_error = error_vec[:3]
        ori_error = error_vec[3:]
        weight_position = 10.0      # 1.0
        weight_orientation = 1.0    # 0.1
        self.totalcost = casadi.sumsqr(weight_position * pos_error) + casadi.sumsqr(weight_orientation * ori_error)
        self.regularization = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.param_q_prev) # 惩罚与上一时刻姿态的偏差
        joint_ref_weight_dm = casadi.DM(self._joint_reference_mask)
        self.branch_ref_cost = casadi.sumsqr(
            (self.var_q - self.param_q_ref) * joint_ref_weight_dm
        )
        elbow_perp_unit_var = self._elbow_perp_unit_fn(self.var_q)
        self.psi_cost = casadi.sumsqr(elbow_perp_unit_var - self.param_elbow_perp)
        elbow_perp_var = self._elbow_perp_fn(self.var_q)
        elbow_side_scalar_var = casadi.dot(elbow_perp_var, self.param_elbow_side_normal)
        side_violation = -self.param_elbow_side_sign * elbow_side_scalar_var
        softplus_side = casadi.log(
            1.0 + casadi.exp(self._elbow_side_penalty_sharpness * side_violation)
        )
        self.elbow_side_cost = softplus_side ** 2

        # 奇异性惩罚：基于雅可比可操作度，远离奇异位形
        # 使用 -log(det(J*J^T) + eps) 作为惩罚项，det 越小惩罚越大
        cpin.computeJointJacobians(self.cmodel, self.cdata, self.cq)
        jacobian = cpin.getFrameJacobian(
            self.cmodel,
            self.cdata,
            self.gripper_id,
            pin.ReferenceFrame.LOCAL
        )
        jj_t = casadi.mtimes(jacobian, jacobian.T)
        singularity_eps = 1e-6
        self.singularity = casadi.Function(
            "singularity",
            [self.cq],
            [-casadi.log(casadi.det(jj_t) + singularity_eps)]
        )
        self.singularity_cost = self.singularity(self.var_q)

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )
        if self.enable_hard_delta_q_constraint:
            self.opti.subject_to(
                self.opti.bounded(
                    self.param_q_prev - self.max_delta_q_rad,
                    self.var_q,
                    self.param_q_prev + self.max_delta_q_rad,
                )
            )

        if self._enable_virtual_wall:
            ee_pos_var = self._ee_pos_fn(self.var_q)
            wrist_pos_var = self._wrist_pos_fn(self.var_q)
            wall_x_min = self._virtual_wall_x + self._virtual_wall_clearance
            self.opti.subject_to(ee_pos_var[0] >= wall_x_min)
            self.opti.subject_to(wrist_pos_var[0] >= wall_x_min)

        # 优化目标：主任务（位姿）+ 正则化（趋向零位）+ 平滑项（趋向上一时刻）
        # 增加 smooth_cost 的权重可以有效防止机械臂在多解之间跳变
        # 增加 singularity_cost 的权重可抑制进入奇异位形
        self.opti.minimize(
            20 * self.totalcost +
            # 0.01 * self.regularization +
            5.0 * self.smooth_cost +
            (self._psi_weight * self.psi_cost if self.enable_psi_constraint else 0.0) +
            (
                self._joint_reference_weight * self.branch_ref_cost
                if self._enable_joint_reference_penalty
                else 0.0
            )
            +
            (
                self._elbow_side_penalty_weight * self.elbow_side_cost
                if self._enable_elbow_side_penalty
                else 0.0
            )
            +
            0.02 * self.singularity_cost
        )

        opts = {
            'ipopt': {
                'print_level': 0,
                'max_iter': 50,   # 降低迭代次数以提高实时性
                'tol': 1e-4    # 放宽收敛容忍度
            },
            'print_time': False
        }
        self.opti.solver("ipopt", opts)

    def ik_fun(self, target_pose, gripper=0, motorstate=None, motorV=None, enable_vis=False):
        gripper = np.array([gripper/2.0, -gripper/2.0])
        if motorstate is not None:
            self.init_data = motorstate

        # 设置初始值和上一时刻参考值
        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.param_q_prev, self.history_data)
        self.opti.set_value(self.param_q_ref, self.history_data)
        self.opti.set_value(
            self.param_elbow_side_normal, self._compute_elbow_side_normal(self.history_data)
        )
        self.opti.set_value(
            self.param_elbow_side_sign, self._compute_elbow_side_sign(self.history_data)
        )
        elbow_perp_ref = self._compute_elbow_perp(self.history_data)
        ep_norm = float(np.linalg.norm(elbow_perp_ref))
        elbow_perp_unit_ref = elbow_perp_ref / max(ep_norm, self._psi_eps)
        self.opti.set_value(self.param_elbow_perp, elbow_perp_unit_ref)

        # 将可视化更新设为可选，默认关闭以提高性能
        if enable_vis:
            self.vis.viewer['ee_target'].set_transform(target_pose)     # for visualization

        self.opti.set_value(self.param_tf, target_pose)

        try:
            # sol = self.opti.solve()
            sol = self.opti.solve_limited()
            sol_q = self.opti.value(self.var_q)

            # 更新历史数据和初始值
            self.init_data = sol_q
            self.history_data = sol_q

            # 将可视化更新设为可选，默认关闭以提高性能
            if enable_vis:
                self.vis.display(sol_q)  # for visualization

            nv = self.reduced_robot.model.nv
            if motorV is not None:
                v = motorV * 0.0
                # v = np.zeros(nv)
            else:
                v = (sol_q - self.init_data) * 0.0
                # v = np.zeros(nv)

            tau_ff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v,
                              np.zeros(self.reduced_robot.model.nv))

            is_collision = self.check_self_collision(sol_q)
            dist = self.get_dist(sol_q, target_pose[:3, 3])
            # if self._enable_virtual_wall:
            #     now = time.time()
            #     if now - self._last_virtual_wall_log_time >= self._virtual_wall_log_period_s:
            #         ee_pos = np.array(self._ee_pos_fn(sol_q)).reshape(-1)
            #         wrist_pos = np.array(self._wrist_pos_fn(sol_q)).reshape(-1)
            #         wall_x_min = self._virtual_wall_x + self._virtual_wall_clearance
            #         msg = (
            #             f"virtual_wall: ee_x={float(ee_pos[0]):.4f}, "
            #             f"wrist_x={float(wrist_pos[0]):.4f}, x_min={wall_x_min:.4f}"
            #         )
            #         if self._logger is not None:
            #             self._logger.info(msg)
            #         else:
            #             print(msg, flush=True)
            #         self._last_virtual_wall_log_time = now
            # print("dist:", dist)
            return sol_q, tau_ff, is_collision

        except Exception as e:
            print(f"ERROR in convergence, plotting debug info.{e}")
            # sol_q = self.opti.debug.value(self.var_q)   # return original value
            return None, '', False

    # def check_self_collision(self, q, gripper=np.array([0, 0])):
    #     pin.forwardKinematics(self.robot.model, self.robot.data, np.concatenate([q, gripper], axis=0))
    #     pin.updateGeometryPlacements(self.robot.model, self.robot.data, self.geom_model, self.geometry_data)
    #     collision = pin.computeCollisions(self.geom_model, self.geometry_data, False)
    #     # print("collision:", collision)
    #     return collision


    def check_self_collision(self, q, gripper=None): # 去掉不再需要的 gripper 参数或不使用它
        # 只传入 IK 求解出的 7 维关节角 q
        pin.forwardKinematics(self.robot.model, self.robot.data, q)
        pin.updateGeometryPlacements(self.robot.model, self.robot.data, self.geom_model, self.geometry_data)
        collision = pin.computeCollisions(self.geom_model, self.geometry_data, False)
        return collision

    def get_dist(self, q, xyz):
        # print("q:", q)
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, np.concatenate([q], axis=0))
        dist = math.sqrt(pow((xyz[0] - self.reduced_robot.data.oMi[6].translation[0]), 2) + pow((xyz[1] - self.reduced_robot.data.oMi[6].translation[1]), 2) + pow((xyz[2] - self.reduced_robot.data.oMi[6].translation[2]), 2))
        return dist

    def get_pose(self, q):
        index = 6
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, np.concatenate([q], axis=0))
        end_pose = xyzrpy2Mat(self.reduced_robot.data.oMi[index].translation[0], self.reduced_robot.data.oMi[index].translation[1], self.reduced_robot.data.oMi[index].translation[2],
                                                math.atan2(self.reduced_robot.data.oMi[index].rotation[2, 1], self.reduced_robot.data.oMi[index].rotation[2, 2]),
                                                math.asin(-self.reduced_robot.data.oMi[index].rotation[2, 0]),
                                                math.atan2(self.reduced_robot.data.oMi[index].rotation[1, 0], self.reduced_robot.data.oMi[index].rotation[0, 0]))
        end_pose = np.dot(end_pose, self.last_matrix)
        return matrix2xyzrpy(end_pose)

    def sync_state(self, q_current):
        """同步 IK 求解器的内部状态到当前真实关节位置，防止启动跳变"""
        self.init_data = np.array(q_current)
        self.history_data = np.array(q_current)
        self.opti.set_value(self.param_q_ref, self.history_data)
        self.opti.set_value(
            self.param_elbow_side_normal, self._compute_elbow_side_normal(self.history_data)
        )
        self.opti.set_value(
            self.param_elbow_side_sign, self._compute_elbow_side_sign(self.history_data)
        )
        elbow_perp_ref = self._compute_elbow_perp(self.history_data)
        ep_norm = float(np.linalg.norm(elbow_perp_ref))
        elbow_perp_unit_ref = elbow_perp_ref / max(ep_norm, self._psi_eps)
        self.opti.set_value(self.param_elbow_perp, elbow_perp_unit_ref)

    def _compute_elbow_perp(self, q):
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, q)
        s = np.array(self.reduced_robot.data.oMi[self._shoulder_jid].translation, dtype=float)
        e = np.array(self.reduced_robot.data.oMi[self._elbow_jid].translation, dtype=float)
        w = np.array(self.reduced_robot.data.oMi[self._wrist_jid].translation, dtype=float)
        sw = w - s
        sw_norm_sq = float(np.dot(sw, sw)) + 1e-8
        se = e - s
        return se - (float(np.dot(se, sw)) / sw_norm_sq) * sw

    def _compute_elbow_side_sign(self, q):
        side_normal = self._compute_elbow_side_normal(q)
        elbow_perp = self._compute_elbow_perp(q)
        side_scalar = float(np.dot(elbow_perp, side_normal))
        return 1.0 if side_scalar >= 0.0 else -1.0

    def _compute_elbow_side_normal(self, q):
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, q)
        s = np.array(self.reduced_robot.data.oMi[self._shoulder_jid].translation, dtype=float)
        w = np.array(self.reduced_robot.data.oMi[self._wrist_jid].translation, dtype=float)
        sw = w - s
        base_z = np.array([0.0, 0.0, 1.0], dtype=float)
        side_normal = np.cross(sw, base_z)
        side_normal /= np.sqrt(float(np.dot(side_normal, side_normal)) + 1e-8)
        return side_normal


class RosOperator(Node):
    def __init__(self):
        super().__init__('teleop_left_nero_node')

        # left [0.1, 1.5708, -1.5708, 1.6707, 0.0, 0.0, 0.0]
        # right [-0.1, 1.5708, 1.5708, 1.6707, 0.0, 0.0, 0.0]
        self.declare_parameter("side", "left")
        self.declare_parameter("init_joint_state", [0.1, 1.5708, -1.5708, 1.6707, 0.0, 0.0, 0.0])
        self.declare_parameter("enable_hard_delta_q_constraint", True)
        self.declare_parameter("ik_max_delta_q_rad", [0.1] * 7)
        self.declare_parameter("enable_psi_constraint", True)
        self.declare_parameter("ik_psi_weight", 36.0)
        self.declare_parameter("ik_psi_eps", 0.05)
        self.declare_parameter("enable_joint_reference_penalty", False)
        self.declare_parameter("ik_joint_reference_weight", 1.0)
        self.declare_parameter("ik_joint_reference_mask", [0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 10.0])
        self.declare_parameter("enable_elbow_side_penalty", False)
        self.declare_parameter("ik_elbow_side_penalty_weight", 2.0)
        self.declare_parameter("ik_elbow_side_penalty_sharpness", 15.0)
        self.declare_parameter("enable_virtual_wall", True)
        self.declare_parameter("ik_virtual_wall_x", -0.42)
        self.declare_parameter("ik_virtual_wall_clearance", 0.01)
        self.side = self.get_parameter("side").get_parameter_value().string_value
        self.init_joint_state = self.get_parameter("init_joint_state").get_parameter_value().double_array_value
        enable_hard_delta_q_constraint = bool(
            self.get_parameter("enable_hard_delta_q_constraint").value
        )
        enable_psi_constraint = bool(self.get_parameter("enable_psi_constraint").value)
        enable_joint_reference_penalty = bool(
            self.get_parameter("enable_joint_reference_penalty").value
        )
        enable_elbow_side_penalty = bool(
            self.get_parameter("enable_elbow_side_penalty").value
        )
        max_delta_q_param = self.get_parameter("ik_max_delta_q_rad").value
        if isinstance(max_delta_q_param, (int, float)):
            max_delta_q = np.array([float(max_delta_q_param)] * 7, dtype=float)
        else:
            max_delta_q = np.array([float(v) for v in max_delta_q_param], dtype=float).reshape(-1)
            if max_delta_q.size == 1:
                max_delta_q = np.repeat(max_delta_q, 7)
            elif max_delta_q.size != 7:
                raise ValueError(
                    f"ik_max_delta_q_rad must be scalar or 7 elements, got {max_delta_q.size}"
                )
        psi_weight = float(self.get_parameter("ik_psi_weight").value)
        psi_eps = float(self.get_parameter("ik_psi_eps").value)
        joint_reference_weight = float(self.get_parameter("ik_joint_reference_weight").value)
        elbow_side_penalty_weight = float(
            self.get_parameter("ik_elbow_side_penalty_weight").value
        )
        elbow_side_penalty_sharpness = float(
            self.get_parameter("ik_elbow_side_penalty_sharpness").value
        )

        enable_virtual_wall = bool(self.get_parameter("enable_virtual_wall").value)
        virtual_wall_x = float(self.get_parameter("ik_virtual_wall_x").value)
        virtual_wall_clearance = float(
            self.get_parameter("ik_virtual_wall_clearance").value
        )
        joint_reference_mask_param = self.get_parameter("ik_joint_reference_mask").value
        if isinstance(joint_reference_mask_param, (int, float)):
            joint_reference_mask = np.array([float(joint_reference_mask_param)] * 7, dtype=float)
        else:
            joint_reference_mask = np.array(
                [float(v) for v in joint_reference_mask_param], dtype=float
            ).reshape(-1)
            if joint_reference_mask.size == 1:
                joint_reference_mask = np.repeat(joint_reference_mask, 7)
            elif joint_reference_mask.size != 7:
                raise ValueError(
                    f"ik_joint_reference_mask must be scalar or 7 elements, got {joint_reference_mask.size}"
                )
        self.pub_joint = self.create_publisher(JointState, f'/control/joint_states_{self.side}_raw', 10)
        print(f"[{self.side}] {self.init_joint_state}")

        self.target_joint_state = self.init_joint_state
        self.current_joint_positions = self.init_joint_state
        # self.last_ik_joint_state = self.init_joint_state
        self.last_ik_joint_state = None

        self.inverse_solution = Arm_IK(
            logger=self.get_logger(),
            max_delta_q_rad=max_delta_q,
            enable_hard_delta_q_constraint=enable_hard_delta_q_constraint,
            enable_psi_constraint=enable_psi_constraint,
            psi_weight=psi_weight,
            psi_eps=psi_eps,
            enable_joint_reference_penalty=enable_joint_reference_penalty,
            joint_reference_weight=joint_reference_weight,
            joint_reference_mask=joint_reference_mask,
            enable_elbow_side_penalty=enable_elbow_side_penalty,
            elbow_side_penalty_weight=elbow_side_penalty_weight,
            elbow_side_penalty_sharpness=elbow_side_penalty_sharpness,
            enable_virtual_wall=enable_virtual_wall,
            virtual_wall_x=virtual_wall_x,
            virtual_wall_clearance=virtual_wall_clearance,
        )

        # self.zero_matrix = xyzquat2Mat([-0.22790897570259602, -0.32071199425236196, 0.13759314472718234],
        #                                [0.09473646682239086, -0.7006316193321209, 0.70067758023687, -0.09587108218573763])

        # self.zero_matrix = xyzquat2Mat([-0.237, -0.291, 0.138],
        #                         [-0.562, 0.428, -0.428, 0.563])
        self.zero_matrix = self.get_ee_pose_matrix_by_urdf(self.current_joint_positions)
        self.start_pose_matrix = xyzrpy2Mat(0, 0, 0, 0, 0, 0)   #该值随便填，不为None即可，后续会被更新

        self.is_teleop_active = False
        self.joint_positions_received = False
        self.refresh_zero_matrix = False
        self.refresh_start_pose_matrix = True
        self.sync_ik_state = False
        self._teleop_status_start_flag = False

        # 订阅回调
        self.create_subscription(JointState, f'/feedback/joint_states_{self.side}', self.joint_states_callback, 1)
        self.create_subscription(PoseStamped, f'/{self.side}_handle_pose', self.handle_pose_callback, 1)
        self.teleop_enable_publisher = self.create_publisher(Bool, f'/{self.side}_teleop_enable', 1)
        self.status_srv = self.create_service(Trigger, f'/{self.side}_teleop_trigger', self.teleop_trigger_callback)

        time.sleep(0.5)
        self.init_pose()
        self.teleop_status_timer = self.create_timer(0.05, self.publish_teleop_status_callback)

    def get_ee_pose_matrix_by_urdf(self, joint_list):
        """通过urdf计算关节对应的末端位姿"""
        index = 7
        pin.forwardKinematics(self.inverse_solution.reduced_robot.model, self.inverse_solution.reduced_robot.data, np.concatenate([joint_list], axis=0))
        end_pose = xyzrpy2Mat(self.inverse_solution.reduced_robot.data.oMi[index].translation[0],
                              self.inverse_solution.reduced_robot.data.oMi[index].translation[1],
                              self.inverse_solution.reduced_robot.data.oMi[index].translation[2],
                              math.atan2(self.inverse_solution.reduced_robot.data.oMi[index].rotation[2, 1], self.inverse_solution.reduced_robot.data.oMi[index].rotation[2, 2]),
                              math.asin(-self.inverse_solution.reduced_robot.data.oMi[index].rotation[2, 0]),
                              math.atan2(self.inverse_solution.reduced_robot.data.oMi[index].rotation[1, 0], self.inverse_solution.reduced_robot.data.oMi[index].rotation[0, 0]))
        end_pose = np.dot(end_pose, self.inverse_solution.last_matrix)
        return end_pose

    def get_timestamp(self):
        """获取当前 ROS 2 时间消息"""
        return self.get_clock().now().to_msg()

    def joint_states_callback(self, msg):
        """
        处理从/feedback/joint_states话题接收到的关节状态数据
        """
        # 确保消息中包含关节位置数据
        if len(msg.position) >= 7:
            # 标记已接收到关节位置数据
            self.joint_positions_received = True
            # 更新当前关节位置
            self.current_joint_positions = list(msg.position[:7])
            # self.get_logger().info(f"接收到当前关节位置: {self.current_joint_positions}")

            # 当非遥操时，实时同步ik逆解
            if not self.is_teleop_active:
                self.sync_ik_state = True
                self.inverse_solution.sync_state(self.current_joint_positions)

    def joint_control_nero(self,j1,j2,j3,j4,j5,j6,j7):
        '''
        发布关节控制指令到/control/joint_states话题
        '''
        joint_states_msgs = JointState()
        joint_states_msgs.header = Header()
        joint_states_msgs.header.stamp = self.get_timestamp()
        joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
        joint_states_msgs.position.append(j1)
        joint_states_msgs.position.append(j2)
        joint_states_msgs.position.append(j3)
        joint_states_msgs.position.append(j4)
        joint_states_msgs.position.append(j5)
        joint_states_msgs.position.append(j6)
        joint_states_msgs.position.append(j7)
        self.pub_joint.publish(joint_states_msgs)
        # self.rate.sleep()
        # print("send joint control piper command")

    # 使用线性插值实现平滑过渡到初始位置
    def init_pose(self):
        # 目标关节位置
        target_joint_state = self.target_joint_state

        # 获取当前关节位置
        # 如果已经接收到关节位置数据，使用实际的当前位置
        # 否则会一步调整到位
        if self.joint_positions_received:
            current_positions = self.current_joint_positions
            self.get_logger().info(f"使用实际的当前关节位置: {current_positions}")

            # 设置过渡时间和控制频率
            duration = 0.5  # 过渡持续时间(秒)
            rate = 50  # 控制频率(Hz)

            # 计算总步数
            steps = int(duration * rate)

            # 计算每一步的增量
            increments = [(target - current) / steps for current, target in zip(current_positions, target_joint_state)]

            # 创建ROS的Rate对象控制循环频率
            rate_obj = self.create_rate(rate)

            # 记录开始时间（用于日志）
            start_time = self.get_clock().now()

            self.get_logger().info("平滑移动到初始位置...")

            # 逐步移动到目标位置k
            for step in range(steps + 1):
                # 计算当前步骤的位置
                interpolated_positions = [current + increment * step for current, increment in zip(current_positions, increments)]

                # 发布关节状态消息
                joint_states_msgs = JointState()
                joint_states_msgs.header = Header()
                joint_states_msgs.header.stamp = self.get_timestamp()
                joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
                joint_states_msgs.position = interpolated_positions

                # 发布消息
                self.pub_joint.publish(joint_states_msgs)

                # 按照指定频率控制循环
                rate_obj.sleep()

            # 确保最后一帧是精确的目标位置
            joint_states_msgs = JointState()
            joint_states_msgs.header = Header()
            joint_states_msgs.header.stamp = self.get_timestamp()
            joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
            joint_states_msgs.position = target_joint_state
            self.pub_joint.publish(joint_states_msgs)

            # 计算实际用时
            elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
            print(f"平滑移动到初始位置完成，用时: {elapsed_time:.2f}秒")

        else:
            print("等待遥操")
            start_time = self.get_clock().now()  # 获取当前时间
            while (self.get_clock().now() - start_time).nanoseconds / 1e9 < 0.5:  # 持续发送0.5秒
                joint_states_msgs = JointState()
                joint_states_msgs.header = Header()
                joint_states_msgs.header.stamp = self.get_timestamp()
                joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
                joint_states_msgs.position = target_joint_state
                self.pub_joint.publish(joint_states_msgs)
                time.sleep(0.02)  # 50hz 发送
            # print("send joint control piper command for 2 seconds")
            # 使用默认的非零位置作为起始点
            # current_positions = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            # rospy.loginfo("未接收到当前关节位置，使用默认初始位置")


    def get_ik_solution(self, x,y,z,roll,pitch,yaw,gripper,b):

        q = quaternion_from_euler(roll, pitch, yaw)
        target = pin.SE3(
            pin.Quaternion(q[3], q[0], q[1], q[2]),
            np.array([x, y, z]),
        )

        joint_states = self.last_ik_joint_state

        # 默认关闭可视化 (enable_vis=False)
        sol_q, tau_ff, is_collision = self.inverse_solution.ik_fun(target.homogeneous, 0, enable_vis=True)

        if  b and sol_q is not None:
            self.joint_control_nero(sol_q[0],sol_q[1],sol_q[2],sol_q[3],sol_q[4],sol_q[5],sol_q[6])
            joint_states = sol_q[:7]
            # print("ctrolling!!!")
        if is_collision :
            print("\33[31m-------------------       Robotic arm self-collision!!!       -----------------------------\033[0m")

        return joint_states

    def handle_pose_callback(self, msg):
        if self.is_teleop_active:
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            (roll, pitch, yaw) = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            current_pose = [x, y, z, roll, pitch, yaw]

            if not self.sync_ik_state and self.last_ik_joint_state is not None:
                # print("sync ik by last")
                self.sync_ik_state = True
                self.inverse_solution.sync_state(self.last_ik_joint_state)

            if self.refresh_zero_matrix:
                self.refresh_zero_matrix = False
                if self.joint_positions_received:
                    # print("init zero_matrix by arm")
                    self.zero_matrix = self.get_ee_pose_matrix_by_urdf(self.current_joint_positions)
                elif self.last_ik_joint_state is not None:
                    # print("init zero_matrix by last")
                    self.zero_matrix = self.get_ee_pose_matrix_by_urdf(self.last_ik_joint_state)

            if self.refresh_start_pose_matrix:
                self.refresh_start_pose_matrix = False
                # print("init start_matrix")
                self.start_pose_matrix = xyzrpy2Mat(x, y, z, roll, pitch, yaw)

            arm_control_pose = calc_pose_incre(self.start_pose_matrix, current_pose, self.zero_matrix)
            self.last_ik_joint_state = self.get_ik_solution(arm_control_pose[0],arm_control_pose[1],
                                 arm_control_pose[2],arm_control_pose[3],
                                 arm_control_pose[4],arm_control_pose[5],
                                 0, True)

    def publish_teleop_status_callback(self):
        if self.is_teleop_active:
            if self.refresh_start_pose_matrix:
                print("wait")
            elif not self._teleop_status_start_flag:
                self._teleop_status_start_flag = True
                print("start")
        elif self._teleop_status_start_flag:
            self._teleop_status_start_flag = False
            print("close")
        enable_msg = Bool()
        enable_msg.data = self.is_teleop_active
        self.teleop_enable_publisher.publish(enable_msg)

    def teleop_trigger_callback(self, request, response):
        self.get_logger().info(f"teleop_trigger_callback: {self.is_teleop_active}")

        self.is_teleop_active = not self.is_teleop_active

        # 关闭遥操，进行初始化
        if not self.is_teleop_active:
            self.refresh_zero_matrix = True
            self.refresh_start_pose_matrix = True
            self.sync_ik_state = False

        return response

def main(args=None):
    rclpy.init(args=args)
    try:
        teleop_node = RosOperator()
        # 4. 关键：使用多线程执行器
        executor = MultiThreadedExecutor()

        executor.add_node(teleop_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
