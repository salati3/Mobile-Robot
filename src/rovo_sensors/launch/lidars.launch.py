from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    nano_config = PathJoinSubstitution(
        [FindPackageShare("rovo_sensors"), "config", "nano_scan.yaml"]
    )

    # 1. THE DRIVER
    # We REMAP its output to 'scan_raw' so it doesn't confuse AMCL
    nano_node = Node(
        package="sick_safetyscanners2",
        executable="sick_safetyscanners2_node",
        name="nano_scan",
        output="screen",
        emulate_tty=True,
        parameters=[{
            'frame_id': 'nano_scan_link',
            'sensor_ip': '192.168.0.33',
            'host_ip': '192.168.0.30',
            'interface_ip': '192.168.0.30',  # <--- CRITICAL
            'host_udp_port': 6061,
            'channel': 0,
            'channel_enabled': True,
            'skip': 0,
            'angle_start': 0.0,
            'angle_end': 0.0,
            'time_offset': -1.0,             # <--- CRITICAL (Fixes the lag)
            'general_system_state': True,
            'derived_settings': True,
            'measurement_data': True,
            'intrusion_data': False,
            'application_io_data': False,
            'use_persistent_config': False,
            'use_ros_time': True,
            'min_intensities': 0.0
        }],
        remappings=[('scan', 'scan_raw')]
    )

    # 2. THE TIME FIXER
    # Takes 'scan_raw', fixes time, outputs correct 'scan'
    fixer_node = Node(
        package="rovo_sensors",
        executable="scan_fixer.py",
        name="scan_fixer",
        output="screen"
    )

    return LaunchDescription([
        nano_node,
        fixer_node,
    ])