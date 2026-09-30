import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    noirbot_description_dir = get_package_share_directory("noirbot_description")
    noirbot_bringup_dir = get_package_share_directory("noirbot_bringup")

    # Assumes noirbot.urdf.xacro already xacro:includes noirbot_ros2_control.xacro.
    xacro_file = os.path.join(
        noirbot_description_dir, "urdf", "noirbot.urdf.xacro"
    )
    robot_description = xacro.process_file(xacro_file).toxml()

    controllers_yaml = os.path.join(
        noirbot_bringup_dir, "config", "noirbot_controllers.yaml"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Reads its own hardware plugin from robot_description, so it doesn't
    # need a separate hardware-specific node.
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers_yaml],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Spawn diff_drive_controller only once joint_state_broadcaster is up,
    # same ordering Bumperbot/Indigobot use to avoid a race on startup.
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delayed_diff_drive_spawner,
    ])