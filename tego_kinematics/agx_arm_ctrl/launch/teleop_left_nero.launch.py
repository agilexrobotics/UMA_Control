from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    left_node = Node(
        package="agx_arm_ctrl",
        executable="teleop_single_nero_node.py",
        name="teleop_left_nero_node",
        parameters=[
            {
                "side": "left",
                "init_joint_state": [0.1, 1.5708, -1.5708, 1.6707, 0.0, 0.0, 0.0],
            }
        ],
        prefix="python3",
        additional_env={"PYTHONNOUSERSITE": "1"},
        output="screen",
    )

    return LaunchDescription([left_node])
