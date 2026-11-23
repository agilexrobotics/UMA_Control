"""Offline checks for Piper L/H models and shared Piper teleop launch wiring.

Run in the UMA Conda environment after sourcing ROS. No nodes or CAN drivers
are started; launch actions are inspected rather than executed.
"""

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [
    str(ROOT / "uma_bringup"),
    str(ROOT / "uma_kinematics/agx_arm_ctrl"),
]

import numpy as np
import pinocchio as pin
import xacro
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from agx_arm_ctrl.robot_profile import load_profile, format_arm_home_state

MODELS = ("piper_x", "piper_l", "piper_h")


def package_share(name):
    paths = {
        "uma_description": ROOT / "uma_description",
        "uma_bringup": ROOT / "uma_bringup",
        "agx_arm_ctrl": ROOT / "uma_kinematics/agx_arm_ctrl",
    }
    return str(paths[name])


def load_launch(relative_path):
    path = ROOT / relative_path
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_context(description, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return context


def resolved_arguments(include, context):
    # IncludeLaunchDescription accepts strings as well as substitutions.
    return {
        name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
        for name, value in include.launch_arguments
    }


class PiperVariantTests(unittest.TestCase):
    def setUp(self):
        self.patch_share = patch(
            "ament_index_python.packages.get_package_share_directory",
            side_effect=package_share,
        )
        self.patch_find = patch.object(
            FindPackageShare, "find", side_effect=package_share
        )
        self.patch_share.start()
        self.patch_find.start()
        self.addCleanup(self.patch_share.stop)
        self.addCleanup(self.patch_find.stop)

    def test_profiles_and_feedback_command_mapping(self):
        for model in MODELS:
            with self.subTest(model=model):
                profile = load_profile(str(ROOT / f"uma_bringup/config/{model}_profile.yaml"))
                self.assertEqual(profile.name, model)
                self.assertEqual(profile.nq, 6)
                self.assertEqual(
                    profile.urdf_path_abs,
                    str(ROOT / f"uma_description/urdf/{model}/urdf/{model}_description.urdf"),
                )
                names = [f"joint{i}" for i in range(1, 7)]
                self.assertEqual([joint.urdf_name for joint in profile.joints], names)
                self.assertEqual(profile.ee_offset.parent_joint, "joint6")
                self.assertEqual(profile.safety_frames["wrist_safety_point"].parent_joint, "joint5")
                q = np.array([0.1, 0.5, -0.5, 0.2, 0.1, -0.2])
                feedback = SimpleNamespace(
                    name=["gripper_joint1", *reversed(names)],
                    position=[0.01, *reversed(q.tolist())],
                )
                np.testing.assert_allclose(profile.feedback_to_q(feedback), q)
                self.assertEqual(profile.q_to_command(q), (names, q.tolist()))
                for side in ("left", "right"):
                    self.assertEqual(profile.installation_pos_for(side), "horizontal")
                    self.assertEqual(profile.effector_type_for(side), "agx_gripper")
                    home = profile.arm_home_state.get(side, profile.arm_home_state["default"])
                    self.assertEqual(len(home), 9)
                    self.assertEqual(len(format_arm_home_state(profile, side).split(",")), 9)
                    self.assertTrue(np.all(np.array(home[:6]) >= profile.model.lowerPositionLimit))
                    self.assertTrue(np.all(np.array(home[:6]) <= profile.model.upperPositionLimit))
                self.assertEqual(profile.handle["zoom_factor"], 1.25)

    def test_ik_builds_end_effector_and_safety_constraints(self):
        from agx_arm_ctrl.teleop_single_arm_node import Arm_IK

        for model in MODELS:
            with self.subTest(model=model):
                profile = load_profile(str(ROOT / f"uma_bringup/config/{model}_profile.yaml"))
                solver = Arm_IK(profile, enable_visualization=False)
                self.assertIsNone(solver.vis)
                self.assertEqual(solver.model.nq, 6)
                self.assertLess(solver.model.getFrameId("ee"), solver.model.nframes)
                self.assertLess(solver.model.getFrameId("wrist_safety_point"), solver.model.nframes)
                self.assertEqual(len(solver._box_point_fns), 1)
                q = np.array([0.1, 0.5, -0.5, 0.2, 0.1, -0.2])
                pin.framesForwardKinematics(solver.model, solver.data, q)
                target = solver.data.oMf[solver.gripper_id].homogeneous.copy()
                error = np.asarray(solver.error(q, target)).reshape(-1)
                self.assertTrue(np.isfinite(error).all())
                np.testing.assert_allclose(error, np.zeros(6), atol=1e-8)

    def test_models_expand_and_all_meshes_exist(self):
        display = load_launch("uma_description/launch/display.launch.py")
        for model in ("piper_l", "piper_h"):
            for effector, side in (("none", "left"), ("agx_gripper", "left"), ("revo2", "left"), ("revo2", "right")):
                with self.subTest(model=model, effector=effector, side=side):
                    path = display._resolve_builtin_model_path(
                        model, effector, side, ROOT / "uma_description"
                    )
                    xml = xacro.process_file(path).toxml()
                    self.assertNotIn("agx_arm_description", xml)
                    tree = ET.fromstring(xml)
                    self.assertEqual(tree.attrib["name"], model)
                    for mesh in tree.iter("mesh"):
                        prefix = "package://uma_description/"
                        uri = mesh.attrib["filename"]
                        self.assertTrue(uri.startswith(prefix), uri)
                        self.assertTrue((ROOT / "uma_description" / uri[len(prefix):]).is_file(), uri)
                    kinematic_model = pin.buildModelFromXML(xml)
                    self.assertTrue(kinematic_model.existJointName("joint6"))
                    if effector == "agx_gripper":
                        self.assertTrue(kinematic_model.existJointName("gripper_joint1"))
                        self.assertTrue(kinematic_model.existJointName("gripper_joint2"))

    def test_single_arm_launch_selects_profile_driver_and_bridge(self):
        module = load_launch("uma_bringup/launch/demo_piper.launch.py")
        for model in ("piper", *MODELS):
            for side in ("left", "right"):
                for backend in ("cpp", "python"):
                    with self.subTest(model=model, side=side, backend=backend):
                        description = module.generate_launch_description()
                        context = launch_context(description, arm_type=model, side=side, backend=backend)
                        path = str(ROOT / f"uma_bringup/config/{model}_profile.yaml")
                        self.assertEqual(context.launch_configurations["profile_path"], path)
                        opaque = next(e for e in description.entities if isinstance(e, OpaqueFunction))
                        driver, teleop, bridge = opaque.execute(context)
                        self.assertEqual(
                            resolved_arguments(driver, context)["arm_type"], model
                        )
                        settings = resolved_arguments(driver, context)
                        self.assertEqual(settings["installation_pos"], "horizontal")
                        self.assertEqual(settings["effector_type"], "agx_gripper")
                        self.assertEqual(settings["topics_end"], f"_{side}")
                        self.assertEqual(resolved_arguments(teleop, context)["profile_path"], path)
                        self.assertEqual(resolved_arguments(bridge, context)["profile_path"], path)
                        suffix = "_python" if backend == "python" else ""
                        bridge.launch_description_source.get_launch_description(context)
                        self.assertTrue(
                            bridge.launch_description_source.location.endswith(
                                f"/{side}_piper{suffix}.launch.py"
                            )
                        )

    def test_dual_arm_launch_routes_ports_and_starts_recording_once(self):
        for model in MODELS:
            for backend in ("cpp", "python"):
                with self.subTest(model=model, backend=backend):
                    module = load_launch(f"uma_bringup/launch/demo_dual_{model}.launch.py")
                    description = module.generate_launch_description()
                    context = launch_context(description, backend=backend)
                    groups = [e for e in description.entities if isinstance(e, GroupAction)]
                    self.assertEqual(len(groups), 2)
                    recording = [e for e in description.entities if isinstance(e, Node)]
                    self.assertEqual(len(recording), 1)
                    self.assertTrue(recording[0].condition.evaluate(context))
                    for side, group in zip(("left", "right"), groups):
                        include = next(e for e in group.get_sub_entities() if isinstance(e, IncludeLaunchDescription))
                        values = resolved_arguments(include, context)
                        self.assertEqual(values["arm_type"], model)
                        self.assertEqual(values["side"], side)
                        self.assertEqual(values["arm_port"], f"{side}_arm")
                        self.assertEqual(values["port_name"], f"{side}_uma")
                        self.assertEqual(values["backend"], backend)
                        self.assertEqual(values["profile_path"], str(ROOT / f"uma_bringup/config/{model}_profile.yaml"))
                        self.assertEqual(values["start_recording_control"], "false")
                    # Custom deployment settings remain forwarded, including the safety flags.
                    context.launch_configurations.update(
                        profile_path="/tmp/custom.yaml", left_arm_port="can10",
                        right_port_name="can11", start_arm="false", start_teleop="false",
                        start_recording_control="false",
                    )
                    self.assertFalse(recording[0].condition.evaluate(context))
                    for group in groups:
                        include = next(e for e in group.get_sub_entities() if isinstance(e, IncludeLaunchDescription))
                        values = resolved_arguments(include, context)
                        self.assertEqual(values["profile_path"], "/tmp/custom.yaml")
                        self.assertEqual(values["start_arm"], "false")
                        self.assertEqual(values["start_teleop"], "false")
                        if values["side"] == "left":
                            self.assertEqual(values["arm_port"], "can10")
                        else:
                            self.assertEqual(values["port_name"], "can11")

    def test_shell_wrappers_forward_arguments_without_launching_hardware(self):
        with tempfile.TemporaryDirectory() as directory:
            # Intercept exec bash so the parent startup script (and ros2) never run.
            fake_bash = Path(directory) / "bash"
            fake_bash.write_text('#!/bin/sh\nprintf "%s\\n" "$@" "LAUNCH_FILE=${LAUNCH_FILE:-}"\n')
            fake_bash.chmod(0o755)
            for model in ("piper_l", "piper_h"):
                for kind in ("single", "dual"):
                    for backend in ("cpp", "python"):
                        with self.subTest(model=model, kind=kind, backend=backend):
                            path = ROOT / f"uma_bringup/scripts/start_{kind}_{model}_{backend}.sh"
                            subprocess.run(["/bin/bash", "-n", str(path)], check=True)
                            result = subprocess.run(
                                ["/bin/bash", str(path), "profile_path:=/tmp/profile with spaces.yaml"],
                                check=True, capture_output=True, text=True,
                                env={**os.environ, "PATH": f"{directory}:{os.environ['PATH']}", "LAUNCH_FILE": ""},
                            )
                            lines = result.stdout.splitlines()
                            base = "piper" if kind == "single" else "nero"
                            self.assertEqual(lines[0], str(ROOT / f"uma_bringup/scripts/start_{kind}_{base}_{backend}.sh"))
                            self.assertIn("profile_path:=/tmp/profile with spaces.yaml", lines)
                            if kind == "single":
                                self.assertIn(f"arm_type:={model}", lines)
                            else:
                                self.assertEqual(lines[-1], f"LAUNCH_FILE=demo_dual_{model}.launch.py")


if __name__ == "__main__":
    unittest.main()
