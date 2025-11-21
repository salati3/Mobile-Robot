Move command (gazebo demo as well as the real base when using `use_stamped_vel: true`):

```
ros2 topic pub --rate 10 /diff_drive_base_controller/cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {stamp: {sec: 0}}, twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1}}}"
```

Launch Rovo:

```
ros2 launch rovo_bringup view_rovo.launch.py
```

Send a one-shot command on the real robot (matches `controllers.yaml`, which listens to `TwistStamped`):

```
ros2 topic pub -1 /diff_drive_base_controller/cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {stamp: {sec: 0}}, twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```



ros2 topic pub -r 10 /diff_drive_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"


If you prefer plain `geometry_msgs/msg/Twist`, change `use_stamped_vel` to `false` in `controllers.yaml`.

Real CAN control is gated by `<param name="enable_tx">true</param>` inside `kiros.xacro`; flip it to `false` if you only want to monitor feedback without sending torque/velocity commands.


Bring up can in linux:
# 1) (Optional) bring it down first in case it's in a weird state
sudo ip link set can0 down || true

# 2) Configure CAN interface type + bitrate (example: 500 kbit/s)
sudo ip link set can0 type can bitrate 500000

# 3) Bring the interface up
sudo ip link set can0 up

# 4) Check status
ip -details link show can0
