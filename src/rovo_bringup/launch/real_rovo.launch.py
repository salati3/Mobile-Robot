#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue  # <-- important


def generate_launch_description():
    # --- Launch arguments ---
    use_sim_time = LaunchConfiguration("use_sim_time")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_description_package = DeclareLaunchArgument(
        "description_package",
        default_value="rovo_ur_description",
        description="Package with robot URDF/xacro",
    )

    # Real robot xacro
    declare_description_file = DeclareLaunchArgument(
        "description_file",
        default_value="kiros.xacro",
        description="Real robot xacro file",
    )

    # Controllers config (adjust package if needed)
    default_controllers_path = PathJoinSubstitution(
        [FindPackageShare("rovo_ur_description"), "config", "controllers.yaml"]
    )

    declare_controllers_file = DeclareLaunchArgument(
        "controllers_file",
        default_value=default_controllers_path,
        description="Full path to the controllers configuration file",
    )

    # --- Robot description via xacro ---
    robot_description_path = PathJoinSubstitution(
        [FindPackageShare(description_package), "urdf", description_file]
    )

    # xacro → string, explicitly marked as str
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", robot_description_path]),
            value_type=str,
        )
    }

    # --- controller_manager (ros2_control_node) ---
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_file, robot_description],
        output="both",
    )

    # --- Robot State Publisher ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}, robot_description],
    )

    # --- Spawner: joint_state_broadcaster ---
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="both",
    )

    # --- Spawner: diff_drive_base_controller ---
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller", "--controller-manager", "/controller_manager"],
        output="both",
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_description_package,
            declare_description_file,
            declare_controllers_file,
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
        ]
    )
