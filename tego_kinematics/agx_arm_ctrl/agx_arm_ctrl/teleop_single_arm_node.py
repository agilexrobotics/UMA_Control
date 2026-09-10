#!/usr/bin/env python3
import math
from dataclasses import replace
import time
import numpy as np
import scipy.spatial.transform

import casadi
import pinocchio as pin
import meshcat.geometry as mg
from pinocchio import casadi as cpin

# from oculus_reader import OculusReader
from pinocchio.visualize import MeshcatVisualizer

# ros2 department
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

try:
    from .transformations import (
        quaternion_from_euler,
        quaternion_from_matrix,
        euler_from_matrix,
        euler_from_quaternion,
    )
except ImportError:  # Support execution as the installed ROS script.
    from transformations import (
        quaternion_from_euler,
        quaternion_from_matrix,
        euler_from_matrix,
        euler_from_quaternion,
    )
try:
    from .robot_profile import load_profile
except ImportError:  # Support execution as the installed ROS script.
    from robot_profile import load_profile
try:
    from .teleop_state import (
        FeedbackSnapshot,
        build_heading_mapping,
        build_horizontal_target,
        build_translation_mapping,
        build_live_reference_matrices,
        feedback_is_fresh,
    )
except ImportError:  # Support execution as the installed ROS script.
    from teleop_state import (
        FeedbackSnapshot,
        build_heading_mapping,
        build_horizontal_target,
        build_translation_mapping,
        build_live_reference_matrices,
        feedback_is_fresh,
    )
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger


def xyzquat2Mat(pos, quat):
    # pos/quat are lists
    # pos: [x, y, z]
    # quat: [x, y, z, w] (ROS order)
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


class Arm_IK:
    def __init__(
        self,
        profile,
        logger=None,
        enable_visualization=True,
        max_delta_q_rad=None,
        enable_hard_delta_q_constraint=None,
        enable_psi_constraint=None,
        psi_weight=None,
        psi_eps=None,
        enable_joint_reference_penalty=None,
        joint_reference_weight=None,
        joint_reference_mask=None,
        enable_elbow_side_penalty=None,
        elbow_side_penalty_weight=None,
        elbow_side_penalty_sharpness=None,
    ):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)
        self.profile = profile
        self.ee_frame_name = profile.ee_frame if profile.ee_frame else "ee"

        # ---- Model loading: no geometry unless visualization or collision pairs ----
        self.model = pin.buildModelFromUrdf(profile.urdf_path_abs)
        if profile.locked_joints:
            self.model = pin.buildReducedModel(
                self.model,
                profile.locked_joints,
                reference_configuration=np.zeros(self.model.nq),
            )

        # ---- EE frame: URDF name wins, ee_offset fallback (runtime addFrame) ----
        if profile.ee_frame is not None and profile.ee_offset is not None:
            print(
                f"[Arm_IK] both ee_frame and ee_offset set; URDF frame "
                f"'{profile.ee_frame}' wins"
            )
        if profile.ee_frame is None:
            off = profile.ee_offset
            T = xyzrpy2Mat(
                off.xyz[0], off.xyz[1], off.xyz[2], off.rpy[0], off.rpy[1], off.rpy[2]
            )
            qq = quaternion_from_matrix(T)
            self.model.addFrame(
                pin.Frame(
                    "ee",
                    self.model.getJointId(off.parent_joint),
                    pin.SE3(
                        pin.Quaternion(qq[3], qq[0], qq[1], qq[2]),
                        np.array([T[0, 3], T[1, 3], T[2, 3]]),
                    ),
                    pin.FrameType.OP_FRAME,
                )
            )
        # ---- Safety frames from profile (runtime addFrame, no URDF change) ----
        for name, sf in profile.safety_frames.items():
            if sf.auto:
                # Frame at the URDF joint origin; matches oMi[joint].translation for every q.
                jid = self.model.getJointId(sf.parent_joint)
                parent_jid = self.model.parents[jid]
                placement = self.model.jointPlacements[jid]
            else:
                T = xyzrpy2Mat(
                    sf.xyz[0], sf.xyz[1], sf.xyz[2], sf.rpy[0], sf.rpy[1], sf.rpy[2]
                )
                qq = quaternion_from_matrix(T)
                parent_jid = self.model.getJointId(sf.parent_joint)
                placement = pin.SE3(
                    pin.Quaternion(qq[3], qq[0], qq[1], qq[2]),
                    np.array([T[0, 3], T[1, 3], T[2, 3]]),
                )
            self.model.addFrame(
                pin.Frame(name, parent_jid, placement, pin.FrameType.OP_FRAME)
            )
        # Data must be created after addFrame, else oMf lacks the new frames.
        self.data = pin.Data(self.model)
        self.gripper_id = self.model.getFrameId(self.ee_frame_name)

        # ---- Base frame transform (spec §4.1): T_model_target = T_model_base @ T_base_target ----
        # base_frame is a fixed frame (root-to-base chain is fixed), so compute once.
        pin.forwardKinematics(self.model, self.data, pin.neutral(self.model))
        pin.updateFramePlacements(self.model, self.data)
        self._T_model_base = self.data.oMf[
            self.model.getFrameId(profile.base_frame)
        ].homogeneous.copy()

        # ---- Self-collision geometry (only when profile defines collision pairs) ----
        self.geom_model = None
        self.geometry_data = None
        if profile.collision_pairs:
            try:
                self.geom_model = pin.buildGeomFromUrdf(
                    self.model,
                    profile.urdf_path_abs,
                    pin.GeometryType.COLLISION,
                    package_dirs=profile.package_dirs,
                )
                for i, j in profile.collision_pairs:
                    self.geom_model.addCollisionPair(pin.CollisionPair(i, j))
                self.geometry_data = pin.GeometryData(self.geom_model)
            except Exception as e:
                self.geom_model = None
                self.geometry_data = None
                print(f"[Arm_IK] self-collision geometry failed, disabled: {e}")

        # ---- Params: profile defaults, explicit args override ----
        self.enable_hard_delta_q_constraint = bool(
            profile.enable_hard_delta_q_constraint
            if enable_hard_delta_q_constraint is None
            else enable_hard_delta_q_constraint
        )
        max_delta_q_rad = (
            profile.max_delta_q_rad if max_delta_q_rad is None else max_delta_q_rad
        )
        if isinstance(max_delta_q_rad, (int, float)):
            self.max_delta_q_rad = np.array(
                [float(max_delta_q_rad)] * self.model.nq, dtype=float
            )
        else:
            self.max_delta_q_rad = np.array(max_delta_q_rad, dtype=float).reshape(-1)
            if self.max_delta_q_rad.size == 1:
                self.max_delta_q_rad = np.repeat(self.max_delta_q_rad, self.model.nq)
            if self.max_delta_q_rad.size != self.model.nq:
                raise ValueError(
                    f"max_delta_q_rad 必须为标量或 {self.model.nq} 个元素, "
                    f"got {self.max_delta_q_rad.size}"
                )
        self.enable_psi_constraint = bool(
            profile.enable_psi_constraint
            if enable_psi_constraint is None
            else enable_psi_constraint
        )
        self._psi_weight = float(
            profile.psi_weight if psi_weight is None else psi_weight
        )
        self._psi_eps = float(profile.psi_eps if psi_eps is None else psi_eps)
        self._enable_joint_reference_penalty = bool(
            profile.enable_joint_reference_penalty
            if enable_joint_reference_penalty is None
            else enable_joint_reference_penalty
        )
        self._joint_reference_weight = float(
            profile.joint_reference_weight
            if joint_reference_weight is None
            else joint_reference_weight
        )
        if joint_reference_mask is None:
            joint_reference_mask = profile.joint_reference_mask
        if joint_reference_mask is None:
            self._joint_reference_mask = np.zeros(self.model.nq, dtype=float)
        else:
            self._joint_reference_mask = np.array(
                joint_reference_mask, dtype=float
            ).reshape(-1)
            if self._joint_reference_mask.size == 1:
                self._joint_reference_mask = np.repeat(
                    self._joint_reference_mask, self.model.nq
                )
            if self._joint_reference_mask.size != self.model.nq:
                raise ValueError(
                    "joint_reference_mask 必须为标量或 "
                    f"{self.model.nq} 个元素, got {self._joint_reference_mask.size}"
                )
        if np.any(self._joint_reference_mask < 0.0):
            raise ValueError("joint_reference_mask 必须非负")
        if self._psi_eps <= 0.0:
            raise ValueError("psi_eps 必须 > 0")
        self._enable_elbow_side_penalty = bool(
            profile.enable_elbow_side_penalty
            if enable_elbow_side_penalty is None
            else enable_elbow_side_penalty
        )
        self._elbow_side_penalty_weight = float(
            profile.elbow_side_penalty_weight
            if elbow_side_penalty_weight is None
            else elbow_side_penalty_weight
        )
        self._elbow_side_penalty_sharpness = float(
            profile.elbow_side_penalty_sharpness
            if elbow_side_penalty_sharpness is None
            else elbow_side_penalty_sharpness
        )
        if self._elbow_side_penalty_sharpness <= 0.0:
            raise ValueError("elbow_side_penalty_sharpness 必须 > 0")
        self._singularity_weight = float(profile.singularity_weight)

        # psi/elbow machinery: only built when psi or elbow-side penalty is enabled
        self._shoulder_fid = None
        self._elbow_fid = None
        self._wrist_fid = None
        if self.enable_psi_constraint or self._enable_elbow_side_penalty:
            self._shoulder_fid = self.model.getFrameId(profile.shoulder_frame)
            self._elbow_fid = self.model.getFrameId(profile.elbow_frame)
            self._wrist_fid = self.model.getFrameId(profile.wrist_frame)
        self._logger = logger

        # ---- Visualization (optional; dead displayFrames([113,114]) removed) ----
        self.vis = None
        if enable_visualization:
            try:
                collision_model = pin.buildGeomFromUrdf(
                    self.model,
                    profile.urdf_path_abs,
                    pin.GeometryType.COLLISION,
                    package_dirs=profile.package_dirs,
                )
                visual_model = pin.buildGeomFromUrdf(
                    self.model,
                    profile.urdf_path_abs,
                    pin.GeometryType.VISUAL,
                    package_dirs=profile.package_dirs,
                )
                self.vis = MeshcatVisualizer(self.model, collision_model, visual_model)
                self.vis.initViewer(open=False)
                self.vis.loadViewerModel("pinocchio")
                self.vis.display(pin.neutral(self.model))
            except Exception as e:
                self.vis = None
                print(f"[Arm_IK] visualization geometry failed, disabled: {e}")
            if self.vis is not None:
                # EE target axes (ee_target)
                frame_viz_names = ["ee_target"]
                FRAME_AXIS_POSITIONS = (
                    np.array(
                        [
                            [0, 0, 0],
                            [1, 0, 0],
                            [0, 0, 0],
                            [0, 1, 0],
                            [0, 0, 0],
                            [0, 0, 1],
                        ]
                    )
                    .astype(np.float32)
                    .T
                )
                FRAME_AXIS_COLORS = (
                    np.array(
                        [
                            [1, 0, 0],
                            [1, 0.6, 0],
                            [0, 1, 0],
                            [0.6, 1, 0],
                            [0, 0, 1],
                            [0, 0.6, 1],
                        ]
                    )
                    .astype(np.float32)
                    .T
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

        # IK internal state (previous frame / initial guess)
        self.init_data = np.zeros(self.model.nq)
        self.history_data = np.zeros(self.model.nq)

        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.model)
        self.cdata = self.cmodel.createData()

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.model.nq, 1)
        self.cTf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        if self._shoulder_fid is not None:
            _c_shoulder = self.cdata.oMf[self._shoulder_fid].translation
            _c_elbow = self.cdata.oMf[self._elbow_fid].translation
            _c_wrist = self.cdata.oMf[self._wrist_fid].translation
            _c_sw = _c_wrist - _c_shoulder
            _c_sw_norm_sq = casadi.dot(_c_sw, _c_sw) + 1e-8
            _c_se = _c_elbow - _c_shoulder
            _c_elbow_perp = _c_se - (casadi.dot(_c_se, _c_sw) / _c_sw_norm_sq) * _c_sw
            _c_ep_norm = casadi.sqrt(
                casadi.dot(_c_elbow_perp, _c_elbow_perp) + self._psi_eps**2
            )
            _c_elbow_perp_unit = _c_elbow_perp / _c_ep_norm
            self._elbow_perp_fn = casadi.Function(
                "elbow_perp", [self.cq], [_c_elbow_perp]
            )
            self._elbow_perp_unit_fn = casadi.Function(
                "elbow_perp_unit", [self.cq], [_c_elbow_perp_unit]
            )
        else:
            self._elbow_perp_fn = None
            self._elbow_perp_unit_fn = None

        # 6D pose error function
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

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.model.nq)
        self.param_q_prev = self.opti.parameter(
            self.model.nq
        )  # smoothing to avoid multi-solution jumps
        self.param_q_ref = self.opti.parameter(self.model.nq)
        self.param_tf = self.opti.parameter(4, 4)
        self.param_elbow_perp = self.opti.parameter(3)
        self.param_elbow_side_sign = self.opti.parameter()
        self.param_elbow_side_normal = self.opti.parameter(3)
        self.opti.set_value(self.param_elbow_perp, np.zeros(3))
        self.opti.set_value(self.param_elbow_side_sign, 0.0)
        self.opti.set_value(self.param_elbow_side_normal, np.zeros(3))

        error_vec = self.error(self.var_q, self.param_tf)
        pos_error = error_vec[:3]
        ori_error = error_vec[3:]
        weight_position = 10.0  # 1.0
        weight_orientation = 1.0  # 0.1
        self.totalcost = casadi.sumsqr(weight_position * pos_error) + casadi.sumsqr(
            weight_orientation * ori_error
        )
        self.regularization = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(
            self.var_q - self.param_q_prev
        )  # penalize deviation from previous frame
        joint_ref_weight_dm = casadi.DM(self._joint_reference_mask)
        self.branch_ref_cost = casadi.sumsqr(
            (self.var_q - self.param_q_ref) * joint_ref_weight_dm
        )
        if self._elbow_perp_unit_fn is not None:
            elbow_perp_unit_var = self._elbow_perp_unit_fn(self.var_q)
            self.psi_cost = casadi.sumsqr(elbow_perp_unit_var - self.param_elbow_perp)
            elbow_perp_var = self._elbow_perp_fn(self.var_q)
            elbow_side_scalar_var = casadi.dot(
                elbow_perp_var, self.param_elbow_side_normal
            )
            side_violation = -self.param_elbow_side_sign * elbow_side_scalar_var
            softplus_side = casadi.log(
                1.0 + casadi.exp(self._elbow_side_penalty_sharpness * side_violation)
            )
            self.elbow_side_cost = softplus_side**2
        else:
            self.psi_cost = casadi.SX(0)
            self.elbow_side_cost = casadi.SX(0)

        # Singularity penalty: -log(det(J*J^T) + eps)
        # smaller det -> larger penalty
        cpin.computeJointJacobians(self.cmodel, self.cdata, self.cq)
        jacobian = cpin.getFrameJacobian(
            self.cmodel, self.cdata, self.gripper_id, pin.ReferenceFrame.LOCAL
        )
        jj_t = casadi.mtimes(jacobian, jacobian.T)
        singularity_eps = 1e-6
        self.singularity = casadi.Function(
            "singularity", [self.cq], [-casadi.log(casadi.det(jj_t) + singularity_eps)]
        )
        self.singularity_cost = self.singularity(self.var_q)

        # Setting optimization constraints and goals
        self.opti.subject_to(
            self.opti.bounded(
                self.model.lowerPositionLimit, self.var_q, self.model.upperPositionLimit
            )
        )
        if self.enable_hard_delta_q_constraint:
            self.opti.subject_to(
                self.opti.bounded(
                    self.param_q_prev - self.max_delta_q_rad,
                    self.var_q,
                    self.param_q_prev + self.max_delta_q_rad,
                )
            )

        # Plane wall constraints (safety.virtual_walls):
        # keep_side=positive: n·(p - point) >= margin; negative: n·(p - point) <= -margin
        # p is the protected point in wall frame; wrapped in casadi.Function to get MX
        for w in profile.virtual_walls:
            wall_fid = self.model.getFrameId(w.frame)
            n = np.array(w.normal, dtype=float)
            p_w = np.array(w.point, dtype=float)
            for name in w.protected_frames:
                pfid = self.model.getFrameId(name)
                p_local = self.cdata.oMf[wall_fid].inverse() * self.cdata.oMf[pfid]
                dist_expr = casadi.dot(n, p_local.translation - p_w)
                dist_fn = casadi.Function(
                    f"wall_dist_{w.name}_{name}", [self.cq], [dist_expr]
                )
                dist_var = dist_fn(self.var_q)
                if w.keep_side == "positive":
                    self.opti.subject_to(dist_var >= w.margin)
                else:
                    self.opti.subject_to(dist_var <= -w.margin)

        # Box workspace constraint (safety.workspace):
        # protected point in box frame: min+margin <= p <= max-margin
        if profile.workspace_box is not None:
            box = profile.workspace_box
            box_fid = self.model.getFrameId(box.frame)
            box_min = np.array(box.min, dtype=float) + box.margin
            box_max = np.array(box.max, dtype=float) - box.margin
            if np.any(box_min >= box_max):
                raise ValueError(
                    f"safety.workspace 在 margin={box.margin} 下退化为空盒: "
                    f"min={box.min}, max={box.max}"
                )
            # Called through casadi.Function to obtain MX (Opti.subject_to needs MX)
            self._box_point_fns = []
            for name in box.protected_frames:
                pfid = self.model.getFrameId(name)
                p_box = self.cdata.oMf[box_fid].inverse() * self.cdata.oMf[pfid]
                box_point_fn = casadi.Function(
                    f"box_point_{name}", [self.cq], [p_box.translation]
                )
                self._box_point_fns.append(box_point_fn)
                box_point_var = box_point_fn(self.var_q)
                self.opti.subject_to(box_point_var >= box_min)
                self.opti.subject_to(box_point_var <= box_max)

        # Objective: pose task + smoothness + optional psi/elbow/singularity terms
        # higher smooth_cost weight prevents multi-solution jumps
        # higher singularity weight avoids singular configurations
        self.opti.minimize(
            20 * self.totalcost
            + 5.0 * self.smooth_cost
            + (self._psi_weight * self.psi_cost if self.enable_psi_constraint else 0.0)
            + (
                self._joint_reference_weight * self.branch_ref_cost
                if self._enable_joint_reference_penalty
                else 0.0
            )
            + (
                self._elbow_side_penalty_weight * self.elbow_side_cost
                if self._enable_elbow_side_penalty
                else 0.0
            )
            + self._singularity_weight * self.singularity_cost
        )

        opts = {
            "ipopt": {
                "print_level": 0,
                "max_iter": 50,  # fewer iterations for realtime
                "tol": 1e-4,  # relaxed tolerance
            },
            "print_time": False,
        }
        self.opti.solver("ipopt", opts)

    def ik_fun(
        self, target_pose, gripper=0, motorstate=None, motorV=None, enable_vis=False
    ):
        gripper = np.array([gripper / 2.0, -gripper / 2.0])
        if motorstate is not None:
            self.init_data = motorstate

        # Set initial value and previous-frame reference
        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.param_q_prev, self.history_data)
        self.opti.set_value(self.param_q_ref, self.history_data)
        if self._shoulder_fid is not None:
            self.opti.set_value(
                self.param_elbow_side_normal,
                self._compute_elbow_side_normal(self.history_data),
            )
            self.opti.set_value(
                self.param_elbow_side_sign,
                self._compute_elbow_side_sign(self.history_data),
            )
            elbow_perp_ref = self._compute_elbow_perp(self.history_data)
            ep_norm = float(np.linalg.norm(elbow_perp_ref))
            elbow_perp_unit_ref = elbow_perp_ref / max(ep_norm, self._psi_eps)
            self.opti.set_value(self.param_elbow_perp, elbow_perp_unit_ref)

        # Visualization optional, off by default for performance
        if enable_vis and self.vis is not None:
            self.vis.viewer["ee_target"].set_transform(target_pose)  # for visualization

        self.opti.set_value(self.param_tf, self._T_model_base @ target_pose)

        try:
            # sol = self.opti.solve()
            sol = self.opti.solve_limited()
            sol_q = self.opti.value(self.var_q)

            # Update history and initial guess
            self.init_data = sol_q
            self.history_data = sol_q

            # Visualization optional, off by default for performance
            if enable_vis and self.vis is not None:
                self.vis.display(sol_q)  # for visualization

            nv = self.model.nv
            if motorV is not None:
                v = motorV * 0.0
                # v = np.zeros(nv)
            else:
                v = (sol_q - self.init_data) * 0.0
                # v = np.zeros(nv)

            tau_ff = pin.rnea(self.model, self.data, sol_q, v, np.zeros(self.model.nv))

            is_collision = self.check_self_collision(sol_q)
            dist = self.get_dist(sol_q, target_pose[:3, 3])
            return sol_q, tau_ff, is_collision

        except Exception as e:
            # 限流:持续不可行时最多 3 秒打一条,避免刷屏(临时措施;根因排查后应恢复)
            now = time.time()
            if now - getattr(self, "_last_ik_error_time", 0.0) > 3.0:
                print(f"ERROR in convergence, plotting debug info.{e}")
                self._last_ik_error_time = now
            # sol_q = self.opti.debug.value(self.var_q)   # return original value
            return None, "", False

    def check_self_collision(self, q, gripper=None):
        if self.geom_model is None:
            return False
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateGeometryPlacements(
            self.model, self.data, self.geom_model, self.geometry_data
        )
        return pin.computeCollisions(self.geom_model, self.geometry_data, False)

    def get_dist(self, q, xyz):
        pin.forwardKinematics(self.model, self.data, np.asarray(q, dtype=float))
        pin.updateFramePlacements(self.model, self.data)
        ee_pos = self.data.oMf[self.gripper_id].translation
        return math.sqrt(sum((xyz[k] - ee_pos[k]) ** 2 for k in range(3)))

    def get_pose(self, q):
        pin.forwardKinematics(self.model, self.data, np.asarray(q, dtype=float))
        pin.updateFramePlacements(self.model, self.data)
        oMf = self.data.oMf[self.gripper_id]
        return matrix2xyzrpy(oMf.homogeneous)

    def sync_state(self, q_current):
        """同步 IK 求解器的内部状态到当前真实关节位置，防止启动跳变"""
        self.init_data = np.array(q_current)
        self.history_data = np.array(q_current)
        self.opti.set_value(self.param_q_ref, self.history_data)
        if self._shoulder_fid is not None:
            self.opti.set_value(
                self.param_elbow_side_normal,
                self._compute_elbow_side_normal(self.history_data),
            )
            self.opti.set_value(
                self.param_elbow_side_sign,
                self._compute_elbow_side_sign(self.history_data),
            )
            elbow_perp_ref = self._compute_elbow_perp(self.history_data)
            ep_norm = float(np.linalg.norm(elbow_perp_ref))
            elbow_perp_unit_ref = elbow_perp_ref / max(ep_norm, self._psi_eps)
            self.opti.set_value(self.param_elbow_perp, elbow_perp_unit_ref)

    def _compute_elbow_perp(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        s = np.array(self.data.oMf[self._shoulder_fid].translation, dtype=float)
        e = np.array(self.data.oMf[self._elbow_fid].translation, dtype=float)
        w = np.array(self.data.oMf[self._wrist_fid].translation, dtype=float)
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
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        s = np.array(self.data.oMf[self._shoulder_fid].translation, dtype=float)
        w = np.array(self.data.oMf[self._wrist_fid].translation, dtype=float)
        sw = w - s
        base_z = np.array([0.0, 0.0, 1.0], dtype=float)
        side_normal = np.cross(sw, base_z)
        side_normal /= np.sqrt(float(np.dot(side_normal, side_normal)) + 1e-8)
        return side_normal


class RosOperator(Node):
    def __init__(self):
        super().__init__("teleop_left_arm_node")

        # left [0.1, 1.5708, -1.5708, 1.6707, 0.0, 0.0, 0.0]
        # right [-0.1, 1.5708, 1.5708, 1.6707, 0.0, 0.0, 0.0]
        self.declare_parameter("side", "left")
        self.declare_parameter("profile_path", "")
        self.declare_parameter("feedback_timeout_sec", 1.0)
        self.side = self.get_parameter("side").get_parameter_value().string_value
        if self.side not in ("left", "right"):
            raise ValueError(f"unsupported arm side: {self.side}")
        self.feedback_timeout_sec = float(
            self.get_parameter("feedback_timeout_sec").value
        )
        if self.feedback_timeout_sec <= 0.0:
            raise ValueError("feedback_timeout_sec 必须大于 0")
        profile_path = (
            self.get_parameter("profile_path").get_parameter_value().string_value
        )
        if not profile_path:
            raise RuntimeError(
                "必须提供 profile_path 参数,例如 tego_bringup/config/nero_profile.yaml"
            )
        self.profile = load_profile(profile_path)
        self.profile = replace(self.profile, workspace_box=self.profile.workspace_for(self.side))
        self.installation_pos = self.profile.installation_pos_for(self.side)
        nq = self.profile.nq

        self.declare_parameter(
            "enable_hard_delta_q_constraint",
            self.profile.enable_hard_delta_q_constraint,
        )
        if isinstance(self.profile.max_delta_q_rad, (int, float)):
            default_max_delta = [float(self.profile.max_delta_q_rad)] * nq
        else:
            default_max_delta = list(self.profile.max_delta_q_rad)
        self.declare_parameter("ik_max_delta_q_rad", default_max_delta)
        self.declare_parameter(
            "enable_psi_constraint", self.profile.enable_psi_constraint
        )
        self.declare_parameter("ik_psi_weight", self.profile.psi_weight)
        self.declare_parameter("ik_psi_eps", self.profile.psi_eps)
        self.declare_parameter(
            "enable_joint_reference_penalty",
            self.profile.enable_joint_reference_penalty,
        )
        self.declare_parameter(
            "ik_joint_reference_weight", self.profile.joint_reference_weight
        )
        default_mask = self.profile.joint_reference_mask
        if default_mask is None:
            default_mask = [0.0] * nq
        self.declare_parameter("ik_joint_reference_mask", list(default_mask))
        self.declare_parameter(
            "enable_elbow_side_penalty", self.profile.enable_elbow_side_penalty
        )
        self.declare_parameter(
            "ik_elbow_side_penalty_weight", self.profile.elbow_side_penalty_weight
        )
        self.declare_parameter(
            "ik_elbow_side_penalty_sharpness", self.profile.elbow_side_penalty_sharpness
        )

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
            max_delta_q = np.array([float(max_delta_q_param)] * nq, dtype=float)
        else:
            max_delta_q = np.array(
                [float(v) for v in max_delta_q_param], dtype=float
            ).reshape(-1)
            if max_delta_q.size == 1:
                max_delta_q = np.repeat(max_delta_q, nq)
            elif max_delta_q.size != nq:
                raise ValueError(
                    f"ik_max_delta_q_rad 必须为标量或 {nq} 个元素, got {max_delta_q.size}"
                )
        psi_weight = float(self.get_parameter("ik_psi_weight").value)
        psi_eps = float(self.get_parameter("ik_psi_eps").value)
        joint_reference_weight = float(
            self.get_parameter("ik_joint_reference_weight").value
        )
        elbow_side_penalty_weight = float(
            self.get_parameter("ik_elbow_side_penalty_weight").value
        )
        elbow_side_penalty_sharpness = float(
            self.get_parameter("ik_elbow_side_penalty_sharpness").value
        )
        joint_reference_mask_param = self.get_parameter("ik_joint_reference_mask").value
        if isinstance(joint_reference_mask_param, (int, float)):
            joint_reference_mask = np.array(
                [float(joint_reference_mask_param)] * nq, dtype=float
            )
        else:
            joint_reference_mask = np.array(
                [float(v) for v in joint_reference_mask_param], dtype=float
            ).reshape(-1)
            if joint_reference_mask.size == 1:
                joint_reference_mask = np.repeat(joint_reference_mask, nq)
            elif joint_reference_mask.size != nq:
                raise ValueError(
                    f"ik_joint_reference_mask 必须为标量或 {nq} 个元素, got {joint_reference_mask.size}"
                )
        self.pub_joint = self.create_publisher(
            JointState, f"/control/joint_states_{self.side}_raw", 10
        )
        self.feedback_snapshot = FeedbackSnapshot()
        self.last_ik_joint_state = None

        self.inverse_solution = Arm_IK(
            self.profile,
            logger=self.get_logger(),
            enable_visualization=True,
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
        )

        # self.zero_matrix = xyzquat2Mat([-0.22790897570259602, -0.32071199425236196, 0.13759314472718234],
        #                                [0.09473646682239086, -0.7006316193321209, 0.70067758023687, -0.09587108218573763])

        # self.zero_matrix = xyzquat2Mat([-0.237, -0.291, 0.138],
        #                         [-0.562, 0.428, -0.428, 0.563])
        self.zero_matrix = None
        self.start_pose_matrix = xyzrpy2Mat(
            0, 0, 0, 0, 0, 0
        )  # placeholder; updated later
        self.effective_translation_rotation = None
        self.latest_handle_pose = None
        self.latest_handle_pose_at = None

        self.is_teleop_active = False
        self.refresh_zero_matrix = True
        self.refresh_start_pose_matrix = True
        home_values = self.profile.arm_home_state.get(self.side)
        if home_values is None:
            home_values = self.profile.arm_home_state.get("default")
        if home_values is None:
            raise RuntimeError(
                f"profile 缺少 arm_home_state 的 '{self.side}' 或 'default' 键"
            )
        self.direction_anchor = self.get_ee_pose_matrix_by_urdf(
            home_values[: self.profile.nq]
        )
        self.get_logger().info(
            f"side={self.side}, installation_pos={self.installation_pos}"
        )
        self.sync_ik_state = False
        self._teleop_status_start_flag = False

        # Subscriptions
        self.create_subscription(
            JointState,
            f"/feedback/joint_states_{self.side}",
            self.joint_states_callback,
            1,
        )
        self.create_subscription(
            PoseStamped, f"/{self.side}_handle_pose", self.handle_pose_callback, 1
        )
        self.teleop_enable_publisher = self.create_publisher(
            Bool, f"/{self.side}_teleop_enable", 1
        )
        self.status_srv = self.create_service(
            Trigger, f"/{self.side}_teleop_trigger", self.teleop_trigger_callback
        )

        self.teleop_status_timer = self.create_timer(
            0.05, self.publish_teleop_status_callback
        )

    def get_ee_pose_matrix_by_urdf(self, joint_list):
        """通过 URDF 计算关节对应的末端位姿矩阵(ee frame)。"""
        pin.forwardKinematics(
            self.inverse_solution.model,
            self.inverse_solution.data,
            np.asarray(joint_list, dtype=float),
        )
        pin.updateFramePlacements(
            self.inverse_solution.model, self.inverse_solution.data
        )
        oMf = self.inverse_solution.data.oMf[self.inverse_solution.gripper_id]
        if self.installation_pos == "horizontal":
            # IK accepts targets in profile.base_frame, which may differ from root.
            return np.linalg.inv(self.inverse_solution._T_model_base) @ oMf.homogeneous
        return xyzrpy2Mat(*matrix2xyzrpy(oMf.homogeneous))

    def get_timestamp(self):
        """获取当前 ROS 2 时间消息"""
        return self.get_clock().now().to_msg()

    def joint_states_callback(self, msg):
        """
        处理从/feedback/joint_states话题接收到的关节状态数据
        """
        try:
            q = self.profile.feedback_to_q(msg)
        except ValueError as e:
            self.get_logger().error(f"反馈解析失败: {e}")
            return
        # Update the complete feedback sample atomically (positions + timestamp).
        self.feedback_snapshot.update(q.tolist(), time.monotonic())

        # Sync IK state when not teleoperating
        if not self.is_teleop_active:
            self.sync_ik_state = True
            self.inverse_solution.sync_state(q)

    def has_fresh_feedback(self):
        return self.feedback_snapshot.is_fresh(
            time.monotonic(), self.feedback_timeout_sec
        )

    def _read_fresh_feedback(self):
        positions, received_at = self.feedback_snapshot.read()
        if not feedback_is_fresh(
            received_at, time.monotonic(), self.feedback_timeout_sec
        ):
            return None
        return positions

    def _stop_teleop(self, reason="实时关节反馈已超时，停止遥操"):
        was_active = self.is_teleop_active
        self.is_teleop_active = False
        self.refresh_zero_matrix = True
        self.refresh_start_pose_matrix = True
        self.effective_translation_rotation = None
        self.sync_ik_state = False
        if was_active:
            self.get_logger().warning(reason)

    def publish_joint_command(self, q):
        """按 profile 的 command_name 发布关节命令。"""
        if self.is_teleop_active and not self.has_fresh_feedback():
            self._stop_teleop()
            return False
        try:
            names, positions = self.profile.q_to_command(q)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"拒绝非法关节命令: {exc}")
            return False
        joint_states_msgs = JointState()
        joint_states_msgs.header = Header()
        joint_states_msgs.header.stamp = self.get_timestamp()
        joint_states_msgs.name = names
        joint_states_msgs.position = positions
        self.pub_joint.publish(joint_states_msgs)
        return True

    def get_ik_solution(self, x, y, z, roll, pitch, yaw, gripper, b):

        q = quaternion_from_euler(roll, pitch, yaw)
        target = pin.SE3(
            pin.Quaternion(q[3], q[0], q[1], q[2]),
            np.array([x, y, z]),
        )

        joint_states = self.last_ik_joint_state

        # Visualization off by default (enable_vis=False)
        sol_q, tau_ff, is_collision = self.inverse_solution.ik_fun(
            target.homogeneous, 0, enable_vis=True
        )

        if b and sol_q is not None:
            if self.publish_joint_command(sol_q):
                joint_states = sol_q.tolist()
        if is_collision:
            print(
                "\33[31m-------------------       Robotic arm self-collision!!!       -----------------------------\033[0m"
            )

        return joint_states

    def handle_pose_callback(self, msg):
        if self.installation_pos == "horizontal":
            # Cache while disabled so the trigger can validate and anchor atomically.
            position = msg.pose.position
            orientation = msg.pose.orientation
            values = [
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ]
            try:
                if not np.isfinite(values).all():
                    raise ValueError("位姿包含非有限数值")
                current_matrix = xyzquat2Mat(values[:3], values[3:])
            except ValueError:
                self.latest_handle_pose = None
                self.latest_handle_pose_at = None
                self._stop_teleop("手柄位姿无效，停止遥操；请检查位姿后重新开启")
                return
            self.latest_handle_pose = current_matrix
            self.latest_handle_pose_at = time.monotonic()

        if self.is_teleop_active:
            if not self.has_fresh_feedback():
                self._stop_teleop()
                return

            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            roll, pitch, yaw = euler_from_quaternion(
                [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ]
            )
            if not self.sync_ik_state and self.last_ik_joint_state is not None:
                # print("sync ik by last")
                self.sync_ik_state = True
                self.inverse_solution.sync_state(self.last_ik_joint_state)

            if self.refresh_zero_matrix:
                current_joint_positions = self._read_fresh_feedback()
                if current_joint_positions is None:
                    self._stop_teleop()
                    return
                self.zero_matrix = build_live_reference_matrices(
                    current_joint_positions,
                    self.get_ee_pose_matrix_by_urdf,
                )
                self.refresh_zero_matrix = False

            if self.refresh_start_pose_matrix:
                self.refresh_start_pose_matrix = False
                # print("init start_matrix")
                self.start_pose_matrix = xyzrpy2Mat(x, y, z, roll, pitch, yaw)
                _, self.effective_translation_rotation = build_translation_mapping(
                    self.installation_pos,
                    self.direction_anchor[:3, :3],
                    self.start_pose_matrix[:3, :3],
                )

            if self.installation_pos == "horizontal":
                target_matrix = build_horizontal_target(
                    self.zero_matrix,
                    self.start_pose_matrix,
                    current_matrix,
                    self.effective_translation_rotation,
                )
                # This conversion also handles pitch = +/- pi/2 without NaNs.
                arm_control_pose = [
                    *target_matrix[:3, 3],
                    *euler_from_matrix(target_matrix),
                ]
            else:
                # Preserve the legacy side-mount translation and orientation paths.
                current_matrix = xyzrpy2Mat(x, y, z, roll, pitch, yaw)
                delta = np.dot(np.linalg.inv(self.start_pose_matrix), current_matrix)
                target_matrix = np.dot(self.zero_matrix, delta)
                target_matrix[:3, 3] = self.zero_matrix[:3, 3] + np.dot(
                    self.effective_translation_rotation, delta[:3, 3]
                )
                arm_control_pose = matrix2xyzrpy(target_matrix)
            self.last_ik_joint_state = self.get_ik_solution(
                arm_control_pose[0],
                arm_control_pose[1],
                arm_control_pose[2],
                arm_control_pose[3],
                arm_control_pose[4],
                arm_control_pose[5],
                0,
                True,
            )

    def publish_teleop_status_callback(self):
        if self.is_teleop_active and not self.has_fresh_feedback():
            self._stop_teleop()

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

        if not self.is_teleop_active and not self.has_fresh_feedback():
            response.success = False
            response.message = "尚未收到实时关节反馈，暂不能启动遥操"
            return response

        if not self.is_teleop_active and self.installation_pos == "horizontal":
            if self.latest_handle_pose is None or not feedback_is_fresh(
                self.latest_handle_pose_at, time.monotonic(), self.feedback_timeout_sec
            ):
                response.success = False
                response.message = "尚未收到有效的手柄位姿，暂不能启动遥操"
                return response
            try:
                mapping = build_heading_mapping(self.latest_handle_pose[:3, :3])
                arm_start = build_live_reference_matrices(
                    self._read_fresh_feedback(), self.get_ee_pose_matrix_by_urdf
                )
            except ValueError as exc:
                response.success = False
                response.message = str(exc)
                self.get_logger().warning(response.message)
                return response
            # All callbacks use the default mutually exclusive callback group.
            self.zero_matrix = arm_start
            self.start_pose_matrix = self.latest_handle_pose.copy()
            self.effective_translation_rotation = mapping
            self.refresh_zero_matrix = False
            self.refresh_start_pose_matrix = False

        self.is_teleop_active = not self.is_teleop_active

        # Teleop disabled: reinitialize
        if not self.is_teleop_active:
            self.refresh_zero_matrix = True
            self.refresh_start_pose_matrix = True
            self.effective_translation_rotation = None
            self.sync_ik_state = False

        response.success = True
        response.message = "遥操已开启" if self.is_teleop_active else "遥操已关闭"
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        teleop_node = RosOperator()
        # Multi-threaded executor
        executor = MultiThreadedExecutor()

        executor.add_node(teleop_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
