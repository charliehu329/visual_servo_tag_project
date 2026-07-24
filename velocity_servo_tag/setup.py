import os
from glob import glob

from setuptools import find_packages, setup


package_name = "velocity_servo_tag"


setup(
    name=package_name,
    version="0.2.0",

    packages=find_packages(
        exclude=[
            "test",
        ]
    ),

    data_files=[
        # ROS 2包索引。
        (
            "share/ament_index/resource_index/packages",
            [
                "resource/" + package_name,
            ],
        ),

        # package.xml。
        (
            os.path.join(
                "share",
                package_name,
            ),
            [
                "package.xml",
            ],
        ),

        # 所有YAML配置文件。
        (
            os.path.join(
                "share",
                package_name,
                "config",
            ),
            glob(
                "config/*.yaml"
            ),
        ),

        # 本包使用的URDF。
        (
            os.path.join(
                "share",
                package_name,
                "config",
                "urdf",
            ),
            glob(
                "config/urdf/*.urdf"
            ),
        ),

        # 所有launch文件。
        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob(
                "launch/*.launch.py"
            ),
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="harry",
    maintainer_email="harry@todo.todo",

    description=(
        "Complete AprilTag and Simulink visual-servo "
        "package for Franka Research 3."
    ),

    license="Apache-2.0",

    tests_require=[
        "pytest",
    ],

    entry_points={
        "console_scripts": [
            (
                "apriltag_detector = "
                "velocity_servo_tag.vision."
                "apriltag_detector:main"
            ),
            (
                "vision_double_node = "
                "velocity_servo_tag.vision."
                "vision_double_node:main"
            ),
            (
                "velocity_command_node = "
                "velocity_servo_tag."
                "velocity_command_node:main"
            ),
        ],
    },
)
