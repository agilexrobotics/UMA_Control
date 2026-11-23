"""Start two upright piper_x arms with UMA teleoperation."""

from uma_bringup.piper_launch import dual_piper_launch_description


def generate_launch_description():
    return dual_piper_launch_description("piper_x")
