from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
CONTROL_ROOT = PACKAGE_ROOT.parents[2]


def _active_text_files():
    yield from CONTROL_ROOT.glob("README.md")
    yield from CONTROL_ROOT.rglob("*.py")


def test_active_configs_use_tego_port_names():
    files = list(_active_text_files())
    contents = "\n".join(path.read_text() for path in files)

    assert "left_tego" in contents
    assert "right_tego" in contents


def test_fast_ik_is_removed_from_active_sources():
    files = [path for path in _active_text_files() if "test" not in path.parts]
    contents = "\n".join(path.read_text() for path in files)

    assert "fast_ik" not in contents.lower()


def test_removed_teleop_variants_are_not_packaged():
    removed_files = [
        PACKAGE_ROOT / "agx_arm_ctrl" / "teleop_single_nero_curobo_node.py",
        PACKAGE_ROOT / "agx_arm_ctrl" / "teleop_single_nero_stereo_sew_node.py",
        PACKAGE_ROOT / "agx_arm_ctrl" / "teleop_single_nero_oringi_node.py",
        PACKAGE_ROOT / "agx_arm_ctrl" / "aa.py",
    ]
    removed_files.extend(PACKAGE_ROOT.glob("launch/*_curobo.launch.py"))
    removed_files.extend(PACKAGE_ROOT.glob("launch/*_stereo_sew.launch.py"))

    assert all(not path.exists() for path in removed_files)

    setup_text = (PACKAGE_ROOT / "setup.py").read_text().lower()
    assert "curobo" not in setup_text
    assert "stereo_sew" not in setup_text
    assert "oringi" not in setup_text


def test_teleop_uses_the_renamed_description_layout():
    source = (PACKAGE_ROOT / "agx_arm_ctrl" / "teleop_single_nero_node.py").read_text()

    assert "get_package_share_directory('tego_description')" in source
    assert "agx_arm_urdf" not in source
    assert "'urdf', 'nero', 'urdf', 'nero_description.urdf'" in source
