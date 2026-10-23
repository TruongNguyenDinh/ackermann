# Ackermann wheel odometry (C++)

The `ackermann_odometry` executable subscribes to `/joint_states` and publishes
`/odom` plus `odom -> base_footprint`. The reference point is the rear axle
projected onto the ground. LiDAR placement does not change wheel odometry;
the robot description supplies the sensor transform.

Input must contain measured, signed velocities in **rad/s** for both
`rear_left_wheel_joint` and `rear_right_wheel_joint`, with the measurement
timestamp in `header.stamp`. Positive wheel velocities mean forward motion.
Do not feed commanded velocities or the visualization joint-state publisher
as encoder feedback. Position-only encoders need velocity conversion upstream.
For a complete RViz model, the hardware joint-state source should also provide
positions for the front steering joints and the four wheel joints.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `wheel_radius` | `0.063` | Wheel radius in meters |
| `rear_track` | `0.3118` | Rear wheel center spacing in meters |
| `max_dt` | `0.5` | Maximum accepted measurement interval in seconds |
| `publish_tf` | `true` | Broadcast odometry TF; disable if an EKF owns this TF |

Missing/non-finite wheel data, duplicate timestamps, backwards timestamps and
intervals exceeding `max_dt` are not integrated. Following invalid wheel data,
the first valid sample establishes a new time baseline. Skipping an interval
avoids a false jump but cannot recover motion during missing measurements.
Integration uses a constant-twist circular arc, including reverse motion.
Covariances retain the previous node's zero defaults; they have not been
calibrated for sensor fusion. Set measured odometry and IMU covariances before
using an EKF. This node does not fuse IMU or LiDAR data.

From the workspace root in a sourced ROS 2 Humble shell:

```bash
colcon build --packages-select ackermann_kinematics ackermann_odometry_cpp ackermann_odometry
source install/setup.bash
ros2 run ackermann_odometry_cpp ackermann_odometry
# Separate terminal for command conversion:
ros2 run ackermann_kinematics ackerman_kinematic
```

Run only one wheel odometry node and one publisher for `odom -> base_footprint`.
The existing `ackermann_odometry/launch/sim_slam.launch.py` now selects this C++
executable. Despite its legacy filename, it uses wall time and disables the
robot-only viewer's synthetic joint-state publisher and RViz process. It still needs a real
encoder source on `/joint_states` and a real LiDAR driver on `/scan`; these
hardware drivers are not supplied by this workspace.

On the physical robot, the encoder driver publishes measured `/joint_states`.
This node consumes them and provides `/odom` and `odom -> base_footprint`.
The RPLIDAR driver should publish `sensor_msgs/LaserScan` on `/scan` with
`header.frame_id = laser_frame`. The existing `slam_toolbox` configuration
uses that scan to localize against a map and publish `map -> odom`; Nav2
costmaps also consume `/scan` for obstacles. The sensor model and serial
driver are not yet selected here, so check the actual scan range and rate.

The IMU frame is described as `imu_link`, but there is no current IMU driver
or fusion node in the workspace. A future driver can publish
`sensor_msgs/Imu` on `/imu/data` with `header.frame_id = imu_link`. A
`robot_localization` EKF can then fuse IMU with wheel odometry. At that point,
remap this node's raw `/odom` output for EKF input, set its `publish_tf` to
`false`, and let the EKF be the sole publisher of `odom -> base_footprint`.
Measure the physical IMU mounting pose before relying on orientation data.

The C++ command converter retains `/cmd_vel` in m/s and rad/s, steering output
in radians, and wheel output in rad/s. It now scales both rear speeds together
at saturation and rate-limits a common curvature. The `steering_axis_offset`
default of `0.0369` m matches the existing URDF, giving `0.238` m between front
steering pivots; this offset still requires physical measurement. Rear wheel
track remains `0.3118` m. Per-wheel steering limit remains 15 degrees and wheel
speed limit remains 20 rad/s. Steering state is estimated from commands, not
measured steering feedback. `command_timeout` defaults to 0.5 seconds and
stops the wheels while keeping steering in place; it operates while the node
is running and does not replace a motor-controller watchdog.

```bash
colcon test --packages-select ackermann_odometry_cpp
colcon test-result --verbose
```

Tests start the compiled node with private topic names and cover straight,
curved and reverse motion, invalid samples, time gaps and optional TF output.
