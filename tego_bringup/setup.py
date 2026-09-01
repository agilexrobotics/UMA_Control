from glob import glob
import os

from setuptools import setup


package_name = "tego_bringup"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*")),
        (os.path.join("share", package_name, "udev"), glob("udev/*")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="AgileX Robotics",
    maintainer_email="agilex@agilex.ai",
    description="Bringup and launch files for the tego controller.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [],
    },
)
