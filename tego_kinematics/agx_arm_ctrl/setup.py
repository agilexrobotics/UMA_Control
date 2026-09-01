from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'agx_arm_ctrl'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('lib', package_name), glob('agx_arm_ctrl/*.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'casadi',
        'pin',
        'meshcat',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='AgileX Robotic Arm ROS Package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'agx_arm_ctrl_single = agx_arm_ctrl.agx_arm_ctrl_single_node:main',
            'teleop_single_arm = agx_arm_ctrl.teleop_single_arm_node:main',
        ],
    },
)
