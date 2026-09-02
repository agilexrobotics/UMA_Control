"""Interactive RViz editor for the safety section of a robot profile."""

from __future__ import annotations

from typing import Any

import rclpy
from geometry_msgs.msg import Point, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

from .geometry import (
    box_center_size,
    normal_from_quaternion,
    quaternion_from_normal,
    resize_box,
)
from .profile_io import (
    ProfileValidationError,
    atomic_save_profile,
    load_profile_document,
    remove_virtual_wall,
    remove_workspace_box,
    set_virtual_wall,
    set_workspace_box,
    flip_virtual_wall_normal,
    validate_safety_document,
)


class SafetyMonitorNode(Node):
    """Show and edit workspace boxes and virtual walls in RViz."""

    MENU_SAVE = "save"
    MENU_RELOAD = "reload"
    MENU_CREATE_WORKSPACE = "create_workspace"
    MENU_CREATE_WALL = "create_wall"
    MENU_DELETE_WORKSPACE = "delete_workspace"
    MENU_DELETE_WALL = "delete_wall"
    MENU_FLIP_WALL_NORMAL = "flip_wall_normal"

    def __init__(self) -> None:
        super().__init__("arm_safety_monitor")
        self.declare_parameter("profile_path", "")
        self.declare_parameter("box_resize_step", 0.05)
        self.declare_parameter("plane_size", 0.8)
        self.declare_parameter("default_box_size", [0.6, 0.6, 0.6])
        self.declare_parameter("default_protected_frames", ["ee"])

        profile_path = self.get_parameter("profile_path").value
        if not profile_path:
            raise ValueError("必须提供 profile_path 参数")
        self.profile_path = str(profile_path)
        self.box_resize_step = float(self.get_parameter("box_resize_step").value)
        self.plane_size = float(self.get_parameter("plane_size").value)
        self.default_box_size = [
            float(value) for value in self.get_parameter("default_box_size").value
        ]
        self.default_protected_frames = [
            str(value)
            for value in self.get_parameter("default_protected_frames").value
        ]
        if len(self.default_box_size) != 3 or any(
            value <= 0.0 for value in self.default_box_size
        ):
            raise ValueError("default_box_size 必须是三个正数")
        if self.plane_size <= 0.0:
            raise ValueError("plane_size 必须大于 0")

        self.document: dict[str, Any] = {}
        self.base_frame = "base_link"
        self.server = InteractiveMarkerServer(self, "arm_safety")
        self.menu = MenuHandler()
        self.menu_ids: dict[str, int] = {}
        self._create_menu_entries()
        self._load_document()

        self.create_service(Trigger, "~/save", self._save_callback)
        self.create_service(Trigger, "~/reload", self._reload_callback)
        self.create_service(Trigger, "~/create_workspace", self._create_workspace_callback)
        self.create_service(Trigger, "~/create_wall", self._create_wall_callback)
        self.get_logger().info(
            f"安全空间编辑器已加载 profile: {self.profile_path}，"
            "RViz 中拖动图形，右键菜单可保存"
        )

    def _create_menu_entries(self) -> None:
        self.menu_ids[self.MENU_SAVE] = self.menu.insert(
            "保存 profile", callback=self._menu_callback
        )
        self.menu_ids[self.MENU_RELOAD] = self.menu.insert(
            "重新加载 profile", callback=self._menu_callback
        )
        self.menu_ids[self.MENU_CREATE_WORKSPACE] = self.menu.insert(
            "新建工作空间 Box", callback=self._menu_callback
        )
        self.menu_ids[self.MENU_CREATE_WALL] = self.menu.insert(
            "新建虚拟墙 Plane", callback=self._menu_callback
        )
        self.menu_ids[self.MENU_DELETE_WORKSPACE] = self.menu.insert(
            "删除工作空间 Box", callback=self._menu_callback
        )
        self.menu_ids[self.MENU_DELETE_WALL] = self.menu.insert(
            "删除当前虚拟墙", callback=self._menu_callback
        )
        self.menu_ids[self.MENU_FLIP_WALL_NORMAL] = self.menu.insert(
            "翻转当前虚拟墙法向", callback=self._menu_callback
        )
        resize_parent = self.menu.insert("调整 Box 尺寸")
        for axis, label in enumerate(("X", "Y", "Z")):
            self.menu_ids[f"resize_{axis}_plus"] = self.menu.insert(
                f"{label} +", parent=resize_parent, callback=self._menu_callback
            )
            self.menu_ids[f"resize_{axis}_minus"] = self.menu.insert(
                f"{label} -", parent=resize_parent, callback=self._menu_callback
            )

    def _load_document(self) -> None:
        self.document = load_profile_document(self.profile_path)
        robot = self.document.get("robot", {})
        if not isinstance(robot, dict):
            raise ProfileValidationError("robot 必须是 mapping")
        self.base_frame = str(robot.get("base_frame", "base_link"))
        validate_safety_document(self.document)
        self._rebuild_markers()

    def _rebuild_markers(self) -> None:
        self.server.clear()
        self._insert_menu_marker()
        safety = self.document.get("safety", {})
        workspace = safety.get("workspace") if isinstance(safety, dict) else None
        if isinstance(workspace, dict):
            self._insert_workspace_marker(workspace)
        walls = safety.get("virtual_walls", []) if isinstance(safety, dict) else []
        for wall in walls:
            if isinstance(wall, dict):
                self._insert_wall_marker(wall)
        self.server.applyChanges()

    def _insert_menu_marker(self) -> None:
        marker = InteractiveMarker()
        marker.header.frame_id = self.base_frame
        marker.name = "safety_menu"
        marker.description = "安全空间设置"
        marker.scale = 0.25
        marker.pose.orientation.w = 1.0
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        button = Marker()
        button.type = Marker.SPHERE
        button.scale.x = 0.12
        button.scale.y = 0.12
        button.scale.z = 0.12
        button.color.r = 0.1
        button.color.g = 0.8
        button.color.b = 0.2
        button.color.a = 0.9
        control.markers.append(button)
        marker.controls.append(control)
        self.server.insert(marker, feedback_callback=self._feedback_callback)
        self.menu.apply(self.server, marker.name)

    def _insert_workspace_marker(self, workspace: dict[str, Any]) -> None:
        center, size = box_center_size(workspace["min"], workspace["max"])
        marker = InteractiveMarker()
        marker.header.frame_id = str(workspace.get("frame", self.base_frame))
        marker.name = "workspace_box"
        marker.description = "workspace"
        marker.scale = max(size)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = center
        marker.pose.orientation.w = 1.0

        control = InteractiveMarkerControl()
        control.name = "move_box"
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        control.always_visible = True
        box = Marker()
        box.type = Marker.CUBE
        box.scale.x, box.scale.y, box.scale.z = size
        box.color.r = 0.1
        box.color.g = 0.5
        box.color.b = 1.0
        box.color.a = 0.22
        control.markers.append(box)
        marker.controls.append(control)
        self.server.insert(marker, feedback_callback=self._feedback_callback)
        self.menu.apply(self.server, marker.name)

    def _insert_wall_marker(self, wall: dict[str, Any]) -> None:
        name = str(wall["name"])
        marker = InteractiveMarker()
        marker.header.frame_id = str(wall.get("frame", self.base_frame))
        marker.name = f"wall_{name}"
        marker.description = name
        marker.scale = max(self.plane_size, 0.2)
        point = wall["point"]
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = point
        qx, qy, qz, qw = quaternion_from_normal(wall["normal"])
        marker.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        control = InteractiveMarkerControl()
        control.name = "move_rotate_wall"
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        control.always_visible = True
        plane = Marker()
        plane.type = Marker.CUBE
        plane.scale.x = self.plane_size
        plane.scale.y = self.plane_size
        plane.scale.z = 0.012
        plane.color.r = 1.0
        plane.color.g = 0.55
        plane.color.b = 0.05
        plane.color.a = 0.35
        control.markers.append(plane)
        arrow = Marker()
        arrow.type = Marker.ARROW
        arrow.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=0.0, y=0.0, z=self.plane_size * 0.35),
        ]
        arrow.scale.x = 0.018
        arrow.scale.y = 0.035
        arrow.scale.z = 0.05
        arrow.color.r = 0.1
        arrow.color.g = 1.0
        arrow.color.b = 0.1
        arrow.color.a = 0.95
        control.markers.append(arrow)
        marker.controls.append(control)
        self.server.insert(marker, feedback_callback=self._feedback_callback)
        self.menu.apply(self.server, marker.name)

    def _feedback_callback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self._update_from_pose(feedback)
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            self._dispatch_menu(feedback)

    def _update_from_pose(self, feedback: InteractiveMarkerFeedback) -> None:
        pose = feedback.pose
        if feedback.marker_name == "workspace_box":
            workspace = self.document["safety"]["workspace"]
            _, size = box_center_size(workspace["min"], workspace["max"])
            set_workspace_box(
                self.document,
                frame=str(workspace.get("frame", self.base_frame)),
                center=[pose.position.x, pose.position.y, pose.position.z],
                size=size,
                margin=workspace.get("margin", 0.0),
                protected_frames=workspace.get("protected_frames", []),
            )
            return

        if feedback.marker_name.startswith("wall_"):
            wall_name = feedback.marker_name[len("wall_"):]
            wall = self._wall_by_name(wall_name)
            if wall is None:
                return
            set_virtual_wall(
                self.document,
                name=wall_name,
                frame=str(wall.get("frame", self.base_frame)),
                point=[pose.position.x, pose.position.y, pose.position.z],
                normal=normal_from_quaternion(
                    [
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ]
                ),
                keep_side=str(wall.get("keep_side", "positive")),
                margin=wall.get("margin", 0.0),
                protected_frames=wall.get("protected_frames", []),
            )

    def _dispatch_menu(self, feedback: InteractiveMarkerFeedback) -> None:
        selected = next(
            (
                name
                for name, menu_id in self.menu_ids.items()
                if menu_id == feedback.menu_entry_id
            ),
            None,
        )
        if selected == self.MENU_SAVE:
            self._save_profile()
        elif selected == self.MENU_RELOAD:
            self._reload_profile()
        elif selected == self.MENU_CREATE_WORKSPACE:
            self._create_workspace()
        elif selected == self.MENU_CREATE_WALL:
            self._create_wall()
        elif selected == self.MENU_FLIP_WALL_NORMAL and feedback.marker_name.startswith(
            "wall_"
        ):
            wall_name = feedback.marker_name[len("wall_"):]
            try:
                normal = flip_virtual_wall_normal(self.document, wall_name)
            except ProfileValidationError as exc:
                self.get_logger().warning(str(exc))
            else:
                self.get_logger().info(
                    f"虚拟墙 {wall_name} 法向已翻转为 {normal}，保存后写入 profile"
                )
                self._rebuild_markers()
        elif selected == self.MENU_DELETE_WORKSPACE:
            remove_workspace_box(self.document)
            self._rebuild_markers()
        elif selected == self.MENU_DELETE_WALL and feedback.marker_name.startswith("wall_"):
            remove_virtual_wall(self.document, feedback.marker_name[len("wall_"):])
            self._rebuild_markers()
        elif selected and selected.startswith("resize_"):
            self._resize_workspace(selected)

    def _resize_workspace(self, menu_name: str) -> None:
        workspace = self.document.get("safety", {}).get("workspace")
        if not isinstance(workspace, dict):
            self.get_logger().warning("当前没有 workspace Box")
            return
        _, axis_text, direction = menu_name.split("_")
        axis = int(axis_text)
        delta = self.box_resize_step if direction == "plus" else -self.box_resize_step
        try:
            box_min, box_max = resize_box(
                workspace["min"], workspace["max"], axis, delta
            )
            workspace["min"], workspace["max"] = box_min, box_max
            self._rebuild_markers()
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def _create_workspace(self) -> None:
        set_workspace_box(
            self.document,
            frame=self.base_frame,
            center=[0.0, 0.0, 0.25],
            size=self.default_box_size,
            margin=0.02,
            protected_frames=self.default_protected_frames,
        )
        self._rebuild_markers()

    def _create_wall(self) -> None:
        safety = self.document.get("safety", {})
        walls = safety.get("virtual_walls", []) if isinstance(safety, dict) else []
        names = {
            str(wall.get("name")) for wall in walls if isinstance(wall, dict)
        }
        index = 1
        while f"rviz_wall_{index}" in names:
            index += 1
        set_virtual_wall(
            self.document,
            name=f"rviz_wall_{index}",
            frame=self.base_frame,
            point=[0.0, 0.0, 0.0],
            normal=[0.0, 0.0, 1.0],
            keep_side="positive",
            margin=0.01,
            protected_frames=self.default_protected_frames,
        )
        self._rebuild_markers()

    def _wall_by_name(self, name: str) -> dict[str, Any] | None:
        safety = self.document.get("safety", {})
        walls = safety.get("virtual_walls", []) if isinstance(safety, dict) else []
        return next(
            (
                wall
                for wall in walls
                if isinstance(wall, dict) and wall.get("name") == name
            ),
            None,
        )

    def _save_profile(self) -> tuple[bool, str]:
        try:
            atomic_save_profile(self.profile_path, self.document)
        except (OSError, ProfileValidationError, TypeError, ValueError) as exc:
            message = f"保存 profile 失败: {exc}"
            self.get_logger().error(message)
            return False, message
        message = f"已保存 profile: {self.profile_path}（重启遥操节点后生效）"
        self.get_logger().info(message)
        return True, message

    def _reload_profile(self) -> tuple[bool, str]:
        try:
            self._load_document()
        except (OSError, ProfileValidationError, TypeError, ValueError) as exc:
            message = f"重新加载 profile 失败: {exc}"
            self.get_logger().error(message)
            return False, message
        message = "已重新加载 profile"
        self.get_logger().info(message)
        return True, message

    def _menu_callback(self, feedback: InteractiveMarkerFeedback) -> None:
        self._feedback_callback(feedback)

    def _save_callback(self, request: Trigger.Request, response: Trigger.Response):
        del request
        response.success, response.message = self._save_profile()
        return response

    def _reload_callback(self, request: Trigger.Request, response: Trigger.Response):
        del request
        response.success, response.message = self._reload_profile()
        return response

    def _create_workspace_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        del request
        try:
            self._create_workspace()
        except (ProfileValidationError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = "已创建 workspace Box，调用 ~/save 保存"
        return response

    def _create_wall_callback(self, request: Trigger.Request, response: Trigger.Response):
        del request
        try:
            self._create_wall()
        except (ProfileValidationError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = "已创建 virtual wall，调用 ~/save 保存"
        return response


def main(args=None) -> None:
    """Run the safety monitor node."""
    rclpy.init(args=args)
    try:
        node = SafetyMonitorNode()
        rclpy.spin(node)
    except (ValueError, ProfileValidationError) as exc:
        print(f"arm_safety_monitor 启动失败: {exc}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
