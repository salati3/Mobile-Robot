# view_rovo.launch.py  — deterministic plugin path + clock bridge + spawners
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    TimerAction,
    ExecuteProcess,
    RegisterEventHandler,
    EmitEvent,
    SetLaunchConfiguration,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
    TextSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _maybe_shutdown(context, *args, **kwargs):
    rviz_alive = LaunchConfiguration('rviz_alive').perform(context)
    gz_alive   = LaunchConfiguration('gz_alive').perform(context)
    if rviz_alive == 'false' and gz_alive == 'false':
        return [EmitEvent(event=Shutdown())]
    return []


def _set_plugin_paths(context, *args, **kwargs):
    # APT plugin location
    ros_lib = Path("/opt/ros/jazzy/lib")

    # Workspace install/lib (for overlays)
    share_dir = FindPackageShare("rovo_ur_description").perform(context)
    install_dir = Path(share_dir).parents[2]  # .../install
    ws_lib = install_dir / "lib"

    def _extend(var):
        old = os.environ.get(var, "")
        new = f"{ros_lib}:{ws_lib}"
        return SetEnvironmentVariable(var, f"{new}:{old}" if old else new)

    return [
        _extend("GZ_SIM_SYSTEM_PLUGIN_PATH"),
        _extend("IGN_GAZEBO_SYSTEM_PLUGIN_PATH"),  # legacy; harmless to set
    ]


def generate_launch_description():
    # Paths
    desc_share = FindPackageShare("rovo_ur_description")
    xacro_file = PathJoinSubstitution([desc_share, "urdf", "rovo_ur.xacro"])
    mesh_abs   = PathJoinSubstitution([desc_share, "meshes", "rovo.dae"])
    rviz_cfg   = PathJoinSubstitution([desc_share, "rviz",  "rovo_ur.rviz"])
    controllers_yaml = PathJoinSubstitution([desc_share, "config", "controllers.yaml"])

    # Build robot_description from xacro: ABS mesh + ABS controllers.yaml
    xacro_cmd = [
        "xacro ", xacro_file,
        " mesh_file:=", TextSubstitution(text="file://"), mesh_abs,
        " controllers_yaml:=", controllers_yaml,
        # " mesh_scale:=0.001 0.001 0.001",
    ]
    robot_description = {"robot_description": ParameterValue(Command(xacro_cmd), value_type=str)}

    # Environment setup
    set_qt = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    set_paths = OpaqueFunction(function=_set_plugin_paths)

    # Gazebo (gz-sim)
    gz = ExecuteProcess(cmd=["gz", "sim", "-r", "empty.sdf"], output="screen")

    # Bridge /clock from gz -> ROS 2
    clock_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    # Bridge BOTH potential Gazebo topics, and remap world clock -> /clock
    arguments=[
        "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
        "/world/empty/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
        "/world/default/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
    ],
    remappings=[
        ("/world/empty/clock", "/clock"),
        ("/world/default/clock", "/clock"),
    ],
    output="screen",
)



    # Robot state publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )


    # Insert model into gz
    spawn = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "run", "ros_gz_sim", "create", "-name", "rovo", "-topic", "/robot_description"],
                output="screen",
            )
        ],
    )

    # map -> base_footprint static TF
    
    static_map_to_odom = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="map_to_odom_publisher", # Renamed for clarity
    arguments=[
        "0", "0", "0", "0", "0", "0", # x, y, z, roll, pitch, yaw
        "map",       # parent frame
        "odom"       # child frame
    ],
    output="screen",
)

    # Spawn controllers (plugin provides /controller_manager inside gz)
    spawn_js = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "60"
                ],
                output="screen",
            )
        ],
    )

    spawn_diff = TimerAction(
        period=3.6,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "diff_drive_base_controller",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "60"
                ],
                output="screen",
            )
        ],
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
    )

    # Exit coordination: only shut down when BOTH have exited
    init_flags = [
        SetLaunchConfiguration('rviz_alive', 'true'),
        SetLaunchConfiguration('gz_alive',   'true'),
    ]
    on_rviz_exit = RegisterEventHandler(OnProcessExit(
        target_action=rviz,
        on_exit=[
            SetLaunchConfiguration('rviz_alive', 'false'),
            OpaqueFunction(function=_maybe_shutdown),
        ],
    ))
    on_gz_exit = RegisterEventHandler(OnProcessExit(
        target_action=gz,
        on_exit=[
            SetLaunchConfiguration('gz_alive', 'false'),
            OpaqueFunction(function=_maybe_shutdown),
        ],
    ))

    return LaunchDescription([
        set_qt,
        OpaqueFunction(function=_set_plugin_paths),  # ensure plugin is found
        *init_flags,
        gz,
        clock_bridge,
        rsp,
        spawn,
        spawn_js,
        spawn_diff,
        static_map_to_odom,
        rviz,
        on_rviz_exit,
        on_gz_exit,
    ])