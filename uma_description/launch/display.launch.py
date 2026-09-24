# Copyright 2026 AgileX Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ast
from pathlib import Path

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARM_TYPES = ('nero', 'piper', 'piper_h', 'piper_l', 'piper_x')

ROBOT_URDF_MAP = {
    arm_type: f'{arm_type}/urdf/{arm_type}_description.urdf'
    for arm_type in ARM_TYPES
}

ROBOT_WITH_GRIPPER_URDF_MAP = {
    arm_type: f'{arm_type}/urdf/{arm_type}_with_gripper_description.xacro'
    for arm_type in ARM_TYPES
}

ROBOT_WITH_REVO2_URDF_MAP = {
    arm_type: {
        side: f'{arm_type}/urdf/{arm_type}_with_{side}_revo2_description.xacro'
        for side in ('left', 'right')
    }
    for arm_type in ARM_TYPES
}


def _resolve_custom_model_path(pkg_path, custom_model):
    # 1) 尝试在 urdf/ 下按相对路径解析
    custom_path = pkg_path / 'urdf' / custom_model
    if custom_path.exists() and custom_path.is_file():
        return str(custom_path)

    # 2) 否则按“用户输入路径”解析，支持 ~ 展开为绝对路径
    expanded = Path(custom_model).expanduser()
    return str(expanded)


def _resolve_builtin_model_path(arm_type, effector_type, revo2_type, pkg_path):
    if effector_type == 'agx_gripper':
        relative_path = ROBOT_WITH_GRIPPER_URDF_MAP[arm_type]
    elif effector_type == 'revo2':
        relative_path = ROBOT_WITH_REVO2_URDF_MAP[arm_type][revo2_type]
    else:
        relative_path = ROBOT_URDF_MAP[arm_type]
    return str(pkg_path / 'urdf' / relative_path)


def _flange_link(arm_type):
    return 'link7' if arm_type == 'nero' else 'link6'


def resolve_model_path(context, *args, **kwargs):
    arm_type = LaunchConfiguration('arm_type').perform(context)
    effector_type = LaunchConfiguration('effector_type').perform(context)
    revo2_type = LaunchConfiguration('revo2_type').perform(context)
    custom_model = LaunchConfiguration('custom_model').perform(context)
    follow = LaunchConfiguration('follow').perform(context)
    control = LaunchConfiguration('control').perform(context)
    try:
        tcp_offset = ast.literal_eval(
            LaunchConfiguration('tcp_offset').perform(context)
        )
    except (SyntaxError, ValueError) as exc:
        raise ValueError(
            'tcp_offset must be a six-element list: [x, y, z, roll, pitch, yaw]'
        ) from exc
    if not isinstance(tcp_offset, (list, tuple)) or len(tcp_offset) != 6:
        raise ValueError(
            'tcp_offset must be a six-element list: [x, y, z, roll, pitch, yaw]'
        )
    try:
        tcp_offset = [float(value) for value in tcp_offset]
    except (TypeError, ValueError) as exc:
        raise ValueError(
            'tcp_offset must contain only numeric values'
        ) from exc
    pkg_path = get_package_share_path('uma_description')

    if custom_model:
        model_path = _resolve_custom_model_path(pkg_path, custom_model)
    else:
        model_path = _resolve_builtin_model_path(
            arm_type, effector_type, revo2_type, pkg_path
        )

    robot_description = ParameterValue(
        Command(['xacro ', model_path]), value_type=str
    )

    joint_state_topic = '/feedback/joint_states' if follow == 'true' else '/control/joint_states'

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        remappings=[
            (
                '/joint_states',
                joint_state_topic,
            )
        ],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{'rate': LaunchConfiguration('pub_rate')}],
        remappings=[('/joint_states', '/control/joint_states')]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
        parameters=[{'rate': 5}],
        remappings=[('/joint_states', '/control/joint_states')]
    )

    rviz_config = LaunchConfiguration('rvizconfig').perform(context)
    if not rviz_config:
        rviz_config = str(get_package_share_path('uma_description') / 'rviz/display.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    nodes = [
        robot_state_publisher_node,
        rviz_node,
    ]

    if control == 'true':
        nodes = [
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            *nodes,
        ]

    if any(v != 0.0 for v in tcp_offset):
        flange = _flange_link(arm_type)
        x, y, z, rx, ry, rz = tcp_offset
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                    '--x', str(x), '--y', str(y), '--z', str(z),
                    '--roll', str(rx), '--pitch', str(ry), '--yaw', str(rz),
                    '--frame-id', flange, '--child-frame-id', 'tcp_link',
                ],
            )
        )

    return nodes


def generate_launch_description():
    urdf_tutorial_path = get_package_share_path('uma_description')
    default_rviz_config_path = urdf_tutorial_path / 'rviz/display.rviz'

    arm_type_arg = DeclareLaunchArgument(
        name='arm_type',
        default_value='nero',
        choices=list(ROBOT_URDF_MAP.keys()),
        description=(
            'Robotic arm type (e.g. nero, piper, piper_x, piper_l, piper_h).'
        ),
    )
    custom_model_arg = DeclareLaunchArgument(
        name='custom_model',
        default_value='',
        description=(
            'Optional custom model path. Prefer absolute path, or relative path '
            'under urdf/. If set, arm_type and effector_type are ignored.'
        ),
    )
    effector_type_arg = DeclareLaunchArgument(
        name='effector_type',
        default_value='none',
        choices=['none', 'agx_gripper', 'revo2'],
        description='End effector type (e.g. agx_gripper, revo2).'
    )
    revo2_type_arg = DeclareLaunchArgument(
        'revo2_type',
        default_value='left',
        choices=['left', 'right'],
        description='Revo2 end effector type (e.g. left, right).'
    )
    pub_rate_arg = DeclareLaunchArgument(
        'pub_rate',
        default_value='200.0',
        description='Publishing rate for the AGX Arm node.'
    )
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui',
    )
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file',
    )
    follow_arg = DeclareLaunchArgument(
        name='follow',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable follow mode',
    )
    control_arg = DeclareLaunchArgument(
        name='control',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to enable publishing control topics.',
    )
    tcp_offset_arg = DeclareLaunchArgument(
        'tcp_offset',
        default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
        description='TCP offset [x, y, z, rx, ry, rz] in meters/radians. '
                    'When non-zero, a tcp_link TF is published relative to the flange.',
    )

    return LaunchDescription([
        arm_type_arg,
        custom_model_arg,
        effector_type_arg,
        revo2_type_arg,
        pub_rate_arg,
        gui_arg,
        rviz_arg,
        follow_arg,
        control_arg,
        tcp_offset_arg,
        OpaqueFunction(function=resolve_model_path),
    ])
