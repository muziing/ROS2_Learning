"""
需求: 在当前launch文件中包含其他launch文件
"""

import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


other_launch_path = os.path.join(
    get_package_share_directory("cpp01_launch"),
    "launch/py",
    "py04_args_launch.py",
)


def generate_launch_description():
    include = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(other_launch_path),
        launch_arguments=[("back_r", "0")],
    )

    node_list = [include]
    return LaunchDescription(node_list)
