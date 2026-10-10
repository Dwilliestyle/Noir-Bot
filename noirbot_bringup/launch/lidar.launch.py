import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("noirbot_bringup"),
        "config",
        "ydlidar_x4pro.yaml",
    )

    lidar_node = Node(
        package="ydlidar_driver",
        executable="ydlidar_node",
        name="ydlidar_node",
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription([lidar_node])
