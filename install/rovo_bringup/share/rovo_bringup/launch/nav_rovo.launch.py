#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- 1. Package Paths ---
    bringup_pkg = FindPackageShare("rovo_bringup")
    description_pkg = FindPackageShare("rovo_ur_description")
    sensors_pkg = FindPackageShare("rovo_sensors")
    nav2_pkg = FindPackageShare("nav2_bringup")
    
    # --- 2. Default Paths ---
    default_map = PathJoinSubstitution([bringup_pkg, "maps", "rovo_lab.yaml"])
    default_params = PathJoinSubstitution([bringup_pkg, "config", "nav2_params.yaml"])
    rviz_config = PathJoinSubstitution([description_pkg, "rviz", "rovo_ur.rviz"]) 
    controllers_file = PathJoinSubstitution([description_pkg, "config", "controllers.yaml"])
    
    # --- 3. Launch Configurations ---
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    # --- 4. Robot Description ---
    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", PathJoinSubstitution([description_pkg, "urdf", "kiros.xacro"])]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # --- 5. Robot State Publisher ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )



    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description, 
            controllers_file, 
        ],
        output="screen",
    )

    lidars = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([sensors_pkg, "launch", "lidars.launch.py"]))
    )

    # --- 7. Controllers ---
    joint_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    
    # We pass the param file to the spawner to be absolutely sure
    diff_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller", 
            "-c", "/controller_manager",
            "--param-file", controllers_file
        ],
        remappings=[('/diff_drive_base_controller/odom', '/odom')]
    )

    # --- 8. Navigation (The Fix for Remapping) ---
    # We wrap the include in a GroupAction to apply remappings to all nodes inside it
    nav2_launch = GroupAction(
        actions=[
            # Connect Nav2 output (smoothed) to Robot input
            SetRemap(src='/cmd_vel_smoothed', dst='/diff_drive_base_controller/cmd_vel'),
            # Connect Nav2 output (raw) just in case smoother is disabled
            SetRemap(src='/cmd_vel', dst='/diff_drive_base_controller/cmd_vel'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([nav2_pkg, "launch", "bringup_launch.py"])
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map": map_file,
                    "params_file": params_file,
                    "autostart": "true",
                    "use_composition": "False",
                    "use_stamped_cmd_vel": "True",  # Ensure Nav2 sends TwistStamped
                }.items(),
            )
        ]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    delay_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_spawner,
            on_exit=[diff_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("params_file", default_value=default_params),
        
        control_node,
        robot_state_publisher,
        lidars,
        joint_spawner,
        delay_diff_drive,
        
        nav2_launch,
        rviz,
    ])