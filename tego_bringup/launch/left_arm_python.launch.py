"""Start the left nero handle bridge (python backend)."""


from tego_bringup.bridge_launch import make_single_bridge_launch


def generate_launch_description():
    return make_single_bridge_launch("nero", "left", "python")
