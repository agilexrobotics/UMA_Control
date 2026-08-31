from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    left_port_name = LaunchConfiguration('left_port_name')
    right_port_name = LaunchConfiguration('right_port_name')
    gripper_width_min = LaunchConfiguration('gripper_width_min')
    gripper_width_max = LaunchConfiguration('gripper_width_max')
    pub_rate = LaunchConfiguration('pub_rate')
    log_level = LaunchConfiguration('log_level')
    declared_args = [
        DeclareLaunchArgument('left_port_name', default_value='left_tego'),
        DeclareLaunchArgument('right_port_name', default_value='right_tego'),
        DeclareLaunchArgument('gripper_width_min', default_value='0.00'),
        DeclareLaunchArgument('gripper_width_max', default_value='0.07'),
        DeclareLaunchArgument('pub_rate', default_value='200.0'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    left_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('tego_bringup'), 'launch', 'left_nero.launch.py'])
        ),
        launch_arguments={
            'port_name': left_port_name,
            'gripper_width_min': gripper_width_min,
            'gripper_width_max': gripper_width_max,
            'pub_rate': pub_rate,
            'log_level': log_level,
        }.items()
    )

    right_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('tego_bringup'), 'launch', 'right_nero.launch.py'])
        ),
        launch_arguments={
            'port_name': right_port_name,
            'gripper_width_min': gripper_width_min,
            'gripper_width_max': gripper_width_max,
            'pub_rate': pub_rate,
            'log_level': log_level,
        }.items()
    )

    return LaunchDescription(declared_args +
    [
        left_arm_launch,
        right_arm_launch
    ])
