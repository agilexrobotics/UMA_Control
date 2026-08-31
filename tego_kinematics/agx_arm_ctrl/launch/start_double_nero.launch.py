from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    left_can_port = LaunchConfiguration('left_can_port')
    right_can_port = LaunchConfiguration('right_can_port')
    log_level = LaunchConfiguration('log_level')
    run_single_arm_launch = PathJoinSubstitution([
        FindPackageShare('agx_arm_ctrl'),
        'launch',
        'start_single_agx_arm.launch.py'
    ])

    left_nero_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(run_single_arm_launch),
        launch_arguments={
            'node_name': 'nero_ctrl_node_left',
            'can_port': left_can_port,
            'log_level': log_level,
            'arm_type': 'nero',
            'effector_type': 'agx_gripper',
            'topics_end': '_left',
        }.items()
    )

    right_nero_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(run_single_arm_launch),
        launch_arguments={
            'node_name': 'nero_ctrl_node_right',
            'can_port': right_can_port,
            'log_level': log_level,
            'arm_type': 'nero',
            'effector_type': 'agx_gripper',
            'topics_end': '_right',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'left_can_port', default_value='left_nero',
            description='SocketCAN interface connected to the left Nero arm.',
        ),
        DeclareLaunchArgument(
            'right_can_port', default_value='right_nero',
            description='SocketCAN interface connected to the right Nero arm.',
        ),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='ROS logging level for the arm drivers.',
        ),
        left_nero_node,
        right_nero_node
        ])
