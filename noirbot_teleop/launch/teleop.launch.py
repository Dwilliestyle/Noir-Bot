from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Include the robot bringup launch file
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('noirbot_bringup'),
                'launch',
                'robot.launch.py'
            ])
        ])
    )
    teleop_mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="keyboard",
        description="Teleop input mode: 'keyboard' or 'joystick'",
    )

    mode = LaunchConfiguration("mode")

    # --- Keyboard teleop ---
    keyboard_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",
        parameters=[{"stamped": True}],
        remappings=[("/cmd_vel", "/diff_drive_controller/cmd_vel")],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'keyboard'"])),
    )

    # --- Joystick teleop group ---
    teleop_joy_config = PathJoinSubstitution(
        [get_package_share_directory("noirbot_teleop"), "config", "teleop_joy.yaml"]
    )

    joystick_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'joystick'"])),
        actions=[
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                output="screen",
                parameters=[teleop_joy_config, {"publish_stamped_twist": True}],
                remappings=[("/cmd_vel", "/diff_drive_controller/cmd_vel")],
            ),
        ],
    )

    return LaunchDescription([
        robot_bringup,
        teleop_mode_arg,
        keyboard_node,
        joystick_group,
    ])
