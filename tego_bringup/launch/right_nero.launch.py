from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port_name = LaunchConfiguration('port_name')
    run_single_node_launch = PathJoinSubstitution([
        FindPackageShare('tego_bringup'),
        'launch',
        'run_single_node.launch.py'
    ])

    gripper_width_min = LaunchConfiguration('gripper_width_min')
    gripper_width_max = LaunchConfiguration('gripper_width_max')
    pub_rate = LaunchConfiguration('pub_rate')
    log_level = LaunchConfiguration('log_level')
    declared_args = [
        DeclareLaunchArgument('port_name', default_value='right_tego'),
        DeclareLaunchArgument('gripper_width_min', default_value='0.00'),
        DeclareLaunchArgument('gripper_width_max', default_value='0.07'),
        DeclareLaunchArgument('pub_rate', default_value='200.0'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    tego_right_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(run_single_node_launch),
        launch_arguments={
            'port_name': port_name,
            'node_name': 'tego_r',
            'angle_reverse': 'true',
            'pika_pose_topic': '/right_handle_pose',
            'remote_joint_topic_in': '/control/joint_states_right_raw',
            'remote_joint_topic_out': '/control/move_js_right',
            'remote_joint_single_topic': '/feedback/joint_states_right',
            'teleop_enable_topic': '/right_teleop_enable',
            'remote_service_name': 'right_teleop_trigger',
            'world_to_base_y': '-0.15',

            'is_nero': 'true',
            'nero_gripper_topic_name': '/control/joint_states_right',
            'arm_zero_joint' : '-0.1, 1.5708, 1.5708, 1.6707, 0, 0, 0, 0, 0, 0.1',
            'publish_sim_joint': 'false',
            'gripper_width_min': gripper_width_min,
            'gripper_width_max': gripper_width_max,
            'pub_rate': pub_rate,
            'log_level': log_level,
            
            'use_piper_modle': 'false',
            'use_pose_dt_zoom': 'true',
            'zoom_factor': '1.4',
            'use_trigger_pose': 'true',
            'gripper_length': '0.0',
        }.items()
    )

    return LaunchDescription(declared_args + [tego_right_node])
