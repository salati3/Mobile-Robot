from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    description_pkg = FindPackageShare("rovo_ur_description")
    
    # Configs
    real_xacro = PathJoinSubstitution([description_pkg, "urdf", "kiros.xacro"])
    controllers_file = PathJoinSubstitution([description_pkg, "config", "controllers.yaml"])
    rviz_config_file = PathJoinSubstitution([description_pkg, "rviz", "rovo_ur.rviz"])

    # URDF
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", real_xacro]), value_type=str
        )
    }

    # 1. Hardware Interface
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="screen",
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    # 3. Spawners
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

    # --- NEW: Fake Map Frame ---
    # This connects 'map' to 'odom' so RViz Grid works (Fixed Frame: map)
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    # 4. RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Delay spawners until controller manager is ready
    delay_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_spawner,
            on_exit=[diff_spawner],
        )
    )

    return LaunchDescription([
        map_to_odom,        # <--- Added this
        control_node,
        robot_state_publisher,
        joint_spawner,
        diff_spawner,
        rviz_node,
    ])