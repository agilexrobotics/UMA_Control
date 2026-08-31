from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    run_single_node_launch = PathJoinSubstitution([
        FindPackageShare('tego_bringup'),
        'launch',
        'run_single_node.launch.py'
    ])

    tego_left_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(run_single_node_launch),
        launch_arguments={
            'port_name': 'left_tego',
            'node_name': 'tego_l',
            'angle_reverse': 'false',
            'pika_pose_topic': '/pika_pose_l',
            'remote_joint_topic_in': '/joint_states_raw_l',
            'remote_joint_topic_out': '/joint_states_l',
            'remote_joint_single_topic': '/joint_states_single_l',
            'teleop_enable_topic': '/teleop_enable_l',
            'remote_service_name': 'teleop_trigger_l',
            'world_to_base_y': '0.1',

            'use_piper_modle': 'false',
            'use_pose_dt_zoom': 'true',
            'zoom_factor': '1.4',
            'use_trigger_pose': 'true',
            'gripper_length': '0.14',
        }.items()
    )

    return LaunchDescription([tego_left_node])