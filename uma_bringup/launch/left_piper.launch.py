"""Start the left piper handle bridge (cpp backend)."""


from uma_bringup.bridge_launch import make_single_bridge_launch


def generate_launch_description():
    return make_single_bridge_launch("piper", "left", "cpp")
