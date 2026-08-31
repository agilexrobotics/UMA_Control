import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_path = get_package_share_directory("agx_arm_ctrl")

    left_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_path, "launch", "teleop_left_nero.launch.py")
        )
    )
    right_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_path, "launch", "teleop_right_nero.launch.py")
        )
    )

    return LaunchDescription([left_arm_launch, right_arm_launch])
