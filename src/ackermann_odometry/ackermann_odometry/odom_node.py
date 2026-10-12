import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster


class AckermannOdometry(Node):

    def __init__(self):
        super().__init__('ackermann_odometry')

        # ==========================
        # Robot geometry
        # ==========================
        # Current defaults match the Xacro estimate; tune from physical
        # measurements when rear track and wheel radius are confirmed.
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('rear_track', 0.3118)
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('rear_track').value

        # ==========================
        # Robot pose
        # ==========================
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = None

        # ==========================
        # Subscriber
        # ==========================
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # ==========================
        # Publishers
        # ==========================
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

    def joint_state_callback(self, msg):
        left_rear_velocity = 0.0
        right_rear_velocity = 0.0
        # Trích xuất thông số khớp từ JointState
        for i, name in enumerate(msg.name):
            if i < len(msg.velocity):
                if name == 'rear_left_wheel_joint':
                    left_rear_velocity = msg.velocity[i]
                elif name == 'rear_right_wheel_joint':
                    right_rear_velocity = msg.velocity[i]

        # Đồng bộ thời gian theo timestamp thực của sensor
        current_time = Time.from_msg(msg.header.stamp)

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds * 1e-9

        if dt <= 0.0:
            # Gazebo can restart its simulation clock at zero while this
            # node remains alive. Rebase the integration timestamp so the
            # first JointState from the new clock epoch does not freeze
            # odometry until it catches up with the old timestamp.
            self.last_time = current_time
            return

        self.last_time = current_time

        # ==========================
        # Kinematics computation
        # ==========================
        v_left = left_rear_velocity * self.wheel_radius
        v_right = right_rear_velocity * self.wheel_radius
        v = (v_left + v_right) / 2.0
        yaw_rate = (v_right - v_left) / self.track_width

        # ==========================
        # Integrate pose
        # ==========================
        # base_footprint is at the rear axle, where wheel odometry's
        # longitudinal velocity has no lateral offset term.
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += yaw_rate * dt

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # ==========================
        # Broadcast TF (sử dụng msg.header.stamp)
        # ==========================
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)

        # ==========================
        # Publish Odometry (sử dụng msg.header.stamp)
        # ==========================
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = yaw_rate

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
