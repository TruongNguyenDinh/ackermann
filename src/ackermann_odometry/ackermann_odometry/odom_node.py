import math

import rclpy
from rclpy.node import Node

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

        self.wheel_radius = 0.05      # m
        self.wheelbase = 0.3187      # m

        # ==========================
        # Robot pose
        # ==========================

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = None

        # ==========================
        # Joint states
        # ==========================

        self.left_rear_velocity = 0.0
        self.right_rear_velocity = 0.0

        self.left_steering_angle = 0.0
        self.right_steering_angle = 0.0

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

        # 50 Hz
        self.timer = self.create_timer(
            0.02,
            self.update_odometry
        )

    def joint_state_callback(self, msg):

        for i, name in enumerate(msg.name):

            if i >= len(msg.position):
                continue

            # --------------------------
            # Steering
            # --------------------------

            if name == 'front_left_steer_joint':
                self.left_steering_angle = msg.position[i]

            elif name == 'front_right_steer_joint':
                self.right_steering_angle = msg.position[i]

            # --------------------------
            # Rear wheel velocity
            # --------------------------

            if i < len(msg.velocity):

                if name == 'rear_left_wheel_joint':
                    self.left_rear_velocity = msg.velocity[i]

                elif name == 'rear_right_wheel_joint':
                    self.right_rear_velocity = msg.velocity[i]

    def update_odometry(self):

        now = self.get_clock().now()

        if self.last_time is None:
            self.last_time = now
            return

        dt = (
            now - self.last_time
        ).nanoseconds * 1e-9

        self.last_time = now

        if dt <= 0.0:
            return

        # ==========================
        # Wheel angular velocity
        # ==========================

        omega_left = self.left_rear_velocity
        omega_right = self.right_rear_velocity

        # ==========================
        # Linear velocity
        # ==========================

        v_left = omega_left * self.wheel_radius
        v_right = omega_right * self.wheel_radius

        v = (v_left + v_right) / 2.0

        # ==========================
        # Steering angle
        # ==========================

        delta_left = self.left_steering_angle
        delta_right = self.right_steering_angle

        delta = (delta_left + delta_right) / 2.0

        # ==========================
        # Bicycle model
        # ==========================

        yaw_rate = (
            v / self.wheelbase
        ) * math.tan(delta)

        # ==========================
        # Integrate pose
        # ==========================

        self.x += (
            v * math.cos(self.yaw) * dt
        )

        self.y += (
            v * math.sin(self.yaw) * dt
        )

        self.yaw += yaw_rate * dt

        # ==========================
        # Quaternion
        # ==========================

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # ==========================
        # TF
        # ==========================

        transform = TransformStamped()

        transform.header.stamp = now.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)

        # ==========================
        # Odometry message
        # ==========================

        odom = Odometry()

        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

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