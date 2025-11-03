Move command:

ros2 topic pub --rate 10 /diff_drive_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0}}, twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1}}}"


Launch Rovo:
ros2 launch rovo_bringup view_rovo.launch.py


