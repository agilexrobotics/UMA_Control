from glob import glob
import os

from setuptools import find_packages, setup

package_name = "arm_safety_monitor"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    test_suite="test",
    tests_require=["pytest"],
    maintainer="jinhuadeng",
    maintainer_email="2540488682@qq.com",
    description="RViz safety workspace and virtual-wall editor for robot profiles.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "arm_safety_monitor_node = arm_safety_monitor.safety_monitor_node:main",
        ],
    },
)
