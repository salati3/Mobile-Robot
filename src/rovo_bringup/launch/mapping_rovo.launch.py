#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    bringup_pkg     = FindPackageShare("rovo_bringup")
    description_pkg = FindPackageShare("rovo_ur_description")
    sensors_pkg     = FindPackageShare("rovo_sensors")

    # --- Configs ---
    real_xacro = PathJoinSubstitution([description_pkg, "urdf", "kiros.xacro"])
    controllers_file = PathJoinSubstitution([description_pkg, "config", "controllers.yaml"])
    slam_params = PathJoinSubstitution([bringup_pkg, "config", "slam_toolbox.yaml"])
    rviz_config = PathJoinSubstitution([description_pkg, "rviz", "rovo_ur.rviz"])
    lidars_launch = PathJoinSubstitution([sensors_pkg, "launch", "lidars.launch.py"])

    # --- Robot Description ---
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", real_xacro]), value_type=str
        )
    }

    # 1. Hardware & Control
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    joint_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    diff_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller", "-c", "/controller_manager"],
    )

    lidars = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidars_launch)
    )

    # 2. SLAM Toolbox
    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
    )

    # 3. Lifecycle Manager (The Alarm Clock) <-- NEW ADDITION
    # This node tells slam_toolbox to wake up (Configure -> Activate)
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["slam_toolbox"]},  # Must match the 'name' above
        ],
    )

    # 4. RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Delays
    delay_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_spawner,
            on_exit=[diff_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        control_node,
        robot_state_publisher,
        joint_spawner,
        delay_spawners,
        lidars,
        slam_node,
        lifecycle_manager,  # <-- Don't forget to add this!
        rviz_node,
    ])