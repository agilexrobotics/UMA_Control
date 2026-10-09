from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_name = LaunchConfiguration("port_name")
    node_name = LaunchConfiguration("node_name")
    uma_pose_topic = LaunchConfiguration("uma_pose_topic")
    remote_joint_topic_in = LaunchConfiguration("remote_joint_topic_in")
    remote_joint_topic_out = LaunchConfiguration("remote_joint_topic_out")
    remote_joint_single_topic = LaunchConfiguration("remote_joint_single_topic")
    teleop_enable_topic = LaunchConfiguration("teleop_enable_topic")
    remote_service_name = LaunchConfiguration("remote_service_name")
    world_to_base_y = LaunchConfiguration("world_to_base_y")
    low_press_level = LaunchConfiguration("low_press_level")
    high_press_level = LaunchConfiguration("high_press_level")
    double_press_timeout = LaunchConfiguration("double_press_timeout")
    long_press_timeout = LaunchConfiguration("long_press_timeout")
    pub_rate = LaunchConfiguration("pub_rate")
    log_level = LaunchConfiguration("log_level")

    arm_home_state = LaunchConfiguration("arm_home_state")
    imu_zero_rpy = LaunchConfiguration("imu_zero_rpy")
    base_to_imu_roll = LaunchConfiguration("base_to_imu_roll")
    base_to_imu_pitch = LaunchConfiguration("base_to_imu_pitch")
    base_to_imu_yaw = LaunchConfiguration("base_to_imu_yaw")
    check_get = LaunchConfiguration("check_get")
    wait_feedback = LaunchConfiguration("wait_feedback")
    publish_sim_joint = LaunchConfiguration("publish_sim_joint")
    gripper_width_min = LaunchConfiguration("gripper_width_min")
    gripper_width_max = LaunchConfiguration("gripper_width_max")
    add_angle = LaunchConfiguration("add_angle")
    angle_value = LaunchConfiguration("angle_value")
    angle_reverse = LaunchConfiguration("angle_reverse")
    base_reverse = LaunchConfiguration("base_reverse")
    imu_reverse = LaunchConfiguration("imu_reverse")
    show_imu_data = LaunchConfiguration("show_imu_data")
    use_piper_mode = LaunchConfiguration("use_piper_mode")
    handle_seg0 = LaunchConfiguration("handle_seg0")
    handle_seg1 = LaunchConfiguration("handle_seg1")
    handle_seg2 = LaunchConfiguration("handle_seg2")
    handle_seg3 = LaunchConfiguration("handle_seg3")
    use_pose_dt_zoom = LaunchConfiguration("use_pose_dt_zoom")
    zoom_factor = LaunchConfiguration("zoom_factor")
    use_trigger_pose = LaunchConfiguration("use_trigger_pose")
    gripper_length = LaunchConfiguration("gripper_length")

    is_nero = LaunchConfiguration("is_nero")
    nero_gripper_topic_name = LaunchConfiguration("nero_gripper_topic_name")
    effector_type = LaunchConfiguration("effector_type")

    declared_args = [
        DeclareLaunchArgument("port_name", default_value="can0"),
        DeclareLaunchArgument("node_name", default_value="uma_ros_node"),
        DeclareLaunchArgument("uma_pose_topic", default_value="/uma_pose"),
        DeclareLaunchArgument(
            "remote_joint_topic_in", default_value="/joint_states_raw"
        ),
        DeclareLaunchArgument("remote_joint_topic_out", default_value="/joint_states"),
        DeclareLaunchArgument(
            "remote_joint_single_topic", default_value="joint_states_single"
        ),
        DeclareLaunchArgument("teleop_enable_topic", default_value="teleop_enable"),
        DeclareLaunchArgument("remote_service_name", default_value="teleop_trigger"),
        DeclareLaunchArgument("world_to_base_y", default_value="0.0"),
        DeclareLaunchArgument("low_press_level", default_value="10"),
        DeclareLaunchArgument("high_press_level", default_value="50"),
        DeclareLaunchArgument("double_press_timeout", default_value="0.30"),
        DeclareLaunchArgument("long_press_timeout", default_value="2.0"),
        DeclareLaunchArgument("pub_rate", default_value="200.0"),
        DeclareLaunchArgument("log_level", default_value="info"),
        # (0, 1.493698, -2.967)
        DeclareLaunchArgument("arm_home_state"),
        # (0, -1.5707963, 0)
        DeclareLaunchArgument("imu_zero_rpy", default_value="0, 0, 0"),
        # (93 5 -6) (-136.604 87.5361 135.85) (-79 -78 -6) (89 44 -1)  (97 52 20) (86 51 -3) (-89 10 -5)
        DeclareLaunchArgument("base_to_imu_roll", default_value="93.0"),
        DeclareLaunchArgument("base_to_imu_pitch", default_value="5.0"),
        DeclareLaunchArgument("base_to_imu_yaw", default_value="-6.0"),
        DeclareLaunchArgument("check_get", default_value="true"),
        DeclareLaunchArgument("wait_feedback", default_value="true"),
        DeclareLaunchArgument("publish_sim_joint", default_value="true"),
        DeclareLaunchArgument("gripper_width_min", default_value="-0.00"),
        DeclareLaunchArgument("gripper_width_max", default_value="0.10"),
        DeclareLaunchArgument("angle_value", default_value="0.35"),
        DeclareLaunchArgument("angle_reverse", default_value="false"),
        DeclareLaunchArgument("base_reverse", default_value="false"),
        DeclareLaunchArgument("imu_reverse", default_value="false"),
        DeclareLaunchArgument("show_imu_data", default_value="false"),
        DeclareLaunchArgument("add_angle", default_value="false"),
        DeclareLaunchArgument("use_piper_mode", default_value="false"),
        DeclareLaunchArgument(
            "handle_seg0", default_value="[0.0, 0.0, 0.123, 0.0, 0.0, 0.0]"
        ),
        DeclareLaunchArgument(
            "handle_seg1",
            default_value="[0.0, 0.0, 0.0, 1.5707963, -0.1359, 3.1415926]",
        ),
        DeclareLaunchArgument(
            "handle_seg2", default_value="[0.28503, 0.0, 0.0, 0.0, 0.0, -1.7939]"
        ),
        DeclareLaunchArgument(
            "handle_seg3", default_value="[-0.022, -0.25075, 0.0, 1.5707963, 0.0, 0.0]"
        ),
        DeclareLaunchArgument("use_pose_dt_zoom", default_value="false"),
        DeclareLaunchArgument("zoom_factor", default_value="2.0"),
        DeclareLaunchArgument("use_trigger_pose", default_value="false"),
        DeclareLaunchArgument("gripper_length", default_value="0.14"),
        DeclareLaunchArgument("is_nero", default_value="false"),
        DeclareLaunchArgument(
            "effector_type",
            default_value="none",
            choices=["none", "agx_gripper", "revo2"],
        ),
        DeclareLaunchArgument(
            "nero_gripper_topic_name", default_value="gripper_states"
        ),
    ]

    uma_ros_node = Node(
        package="uma_ros",
        executable="uma_ros_node",
        name=node_name,
        output="screen",
        ros_arguments=["--log-level", log_level],
        parameters=[
            {
                "port_name": port_name,
                "node_name": node_name,
                "remote_joint_topic_in": remote_joint_topic_in,
                "remote_joint_topic_out": remote_joint_topic_out,
                "remote_service_name": remote_service_name,
                "world_to_base_y": world_to_base_y,
                "low_press_level": low_press_level,
                "high_press_level": high_press_level,
                "double_press_timeout": double_press_timeout,
                "long_press_timeout": long_press_timeout,
                "pub_rate": pub_rate,
                "is_nero": is_nero,
                "effector_type": effector_type,
                "nero_gripper_topic_name": nero_gripper_topic_name,
                "arm_home_state": arm_home_state,
                "imu_zero_rpy": imu_zero_rpy,
                "base_to_imu_roll": base_to_imu_roll,
                "base_to_imu_pitch": base_to_imu_pitch,
                "base_to_imu_yaw": base_to_imu_yaw,
                "check_get": check_get,
                "wait_feedback": wait_feedback,
                "publish_sim_joint": publish_sim_joint,
                "gripper_width_min": gripper_width_min,
                "gripper_width_max": gripper_width_max,
                "angle_value": angle_value,
                "angle_reverse": angle_reverse,
                "base_reverse": base_reverse,
                "imu_reverse": imu_reverse,
                "show_imu_data": show_imu_data,
                "use_piper_mode": use_piper_mode,
                "handle_seg0": handle_seg0,
                "handle_seg1": handle_seg1,
                "handle_seg2": handle_seg2,
                "handle_seg3": handle_seg3,
                "use_pose_dt_zoom": use_pose_dt_zoom,
                "zoom_factor": zoom_factor,
                "use_trigger_pose": use_trigger_pose,
                "gripper_length": gripper_length,
                "trigger_action": False,
                "init_action": False,
                "add_angle": False,
            }
        ],
        remappings=[
            ("/uma_pose", uma_pose_topic),
            ("/joint_states_single", remote_joint_single_topic),
            ("/teleop_enable", teleop_enable_topic),
        ],
    )

    return LaunchDescription(declared_args + [uma_ros_node])
