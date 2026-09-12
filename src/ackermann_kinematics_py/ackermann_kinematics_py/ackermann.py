import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class AckermannKinematics:

    def __init__(
        self,
        wheelbase: float,
        track_width: float,
        wheel_radius: float
    ):
        self.L = wheelbase
        self.W = track_width
        self.Rw = wheel_radius

    def calculate(self, v: float, omega: float):

        if abs(v) < 1e-6:
            return {
                "steering_left": 0.0,
                "steering_right": 0.0,
                "rear_left_wheel": 0.0,
                "rear_right_wheel": 0.0,
            }

        if abs(omega) < 1e-6:
            wheel_velocity = v / self.Rw
            return {
                "steering_left": 0.0,
                "steering_right": 0.0,
                "rear_left_wheel": wheel_velocity,
                "rear_right_wheel": wheel_velocity,
            }

        R = v / omega

        if R > 0.0:
            R_inner = R - self.W / 2.0
            R_outer = R + self.W / 2.0

            steering_left = math.atan2(self.L, R_inner)
            steering_right = math.atan2(self.L, R_outer)

            v_left = omega * R_inner
            v_right = omega * R_outer
        else:
            R_abs = abs(R)
            R_outer = R_abs + self.W / 2.0
            R_inner = R_abs - self.W / 2.0

            steering_left = -math.atan2(self.L, R_outer)
            steering_right = -math.atan2(self.L, R_inner)

            v_left = -omega * R_outer
            v_right = -omega * R_inner

        wheel_left = v_left / self.Rw
        wheel_right = v_right / self.Rw

        return {
            "steering_left": steering_left,
            "steering_right": steering_right,
            "rear_left_wheel": wheel_left,
            "rear_right_wheel": wheel_right,
        }


class AckermannKinematicsNode(Node):

    def __init__(self):

        super().__init__("ackermann_kinematics")

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter("wheelbase", 0.3187)
        self.declare_parameter("track_width", 0.238)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("max_steering_angle", 0.4363)
        self.declare_parameter("max_wheel_velocity", 20.0)

        # Toc do danh lai toi da (rad/s). Day la gia tri MOI - gioi han
        # xe khong the danh lai nhanh hon vat ly thuc te cho phep, tranh
        # truot banh khi Nav2 doi lenh cmd_vel dot ngot o cac path kho.
        # Xe co dan chuan thuong ~0.3-0.6 rad/s; dieu chinh theo dac tinh
        # servo lai thuc te neu biet.
        self.declare_parameter("max_steering_rate", 0.5)

        self.wheelbase = self.get_parameter("wheelbase").value
        self.track_width = self.get_parameter("track_width").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.max_steering_angle = self.get_parameter("max_steering_angle").value
        self.max_wheel_velocity = self.get_parameter("max_wheel_velocity").value
        self.max_steering_rate = self.get_parameter("max_steering_rate").value

        self.kinematics = AckermannKinematics(
            wheelbase=self.wheelbase,
            track_width=self.track_width,
            wheel_radius=self.wheel_radius,
        )

        # Trang thai goc lai HIEN TAI da publish (khong phai gia tri dich)
        # de tinh rate-limit tang dan moi callback
        self._current_steering_left = 0.0
        self._current_steering_right = 0.0
        self._last_time = None

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            "/steering_controller/commands",
            10,
        )

        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            "/wheel_velocity_controller/commands",
            10,
        )

        self.get_logger().info("========================================")
        self.get_logger().info(" Ackermann Controller Started")
        self.get_logger().info(f" wheelbase        : {self.wheelbase:.3f} m")
        self.get_logger().info(f" track width      : {self.track_width:.3f} m")
        self.get_logger().info(f" wheel radius     : {self.wheel_radius:.3f} m")
        self.get_logger().info(f" max steering     : {self.max_steering_angle:.3f} rad")
        self.get_logger().info(f" max steering rate: {self.max_steering_rate:.3f} rad/s")
        self.get_logger().info(f" max wheel vel    : {self.max_wheel_velocity:.3f} rad/s")
        self.get_logger().info("========================================")

    def _rate_limit(self, target: float, current: float, dt: float) -> float:
        """Gioi han buoc thay doi goc lai theo max_steering_rate * dt."""

        if dt <= 0.0:
            return current

        max_delta = self.max_steering_rate * dt
        delta = target - current

        if delta > max_delta:
            delta = max_delta
        elif delta < -max_delta:
            delta = -max_delta

        return current + delta

    def cmd_vel_callback(self, msg: Twist):

        now = time.monotonic()

        if self._last_time is None:
            dt = 0.0
        else:
            dt = now - self._last_time

        self._last_time = now

        v = msg.linear.x
        omega = msg.angular.z

        result = self.kinematics.calculate(v, omega)

        # ------------------------------------------------------------
        # Steering limit (bien do)
        # ------------------------------------------------------------

        target_left = max(
            -self.max_steering_angle,
            min(result["steering_left"], self.max_steering_angle)
        )

        target_right = max(
            -self.max_steering_angle,
            min(result["steering_right"], self.max_steering_angle)
        )

        # ------------------------------------------------------------
        # Rate limit (toc do bien thien) - FIX MOI
        # ------------------------------------------------------------

        steering_left = self._rate_limit(
            target_left, self._current_steering_left, dt
        )
        steering_right = self._rate_limit(
            target_right, self._current_steering_right, dt
        )

        self._current_steering_left = steering_left
        self._current_steering_right = steering_right

        # ------------------------------------------------------------
        # Wheel velocity limit
        # ------------------------------------------------------------

        wheel_left = max(
            -self.max_wheel_velocity,
            min(result["rear_left_wheel"], self.max_wheel_velocity)
        )

        wheel_right = max(
            -self.max_wheel_velocity,
            min(result["rear_right_wheel"], self.max_wheel_velocity)
        )

        steering_msg = Float64MultiArray()
        steering_msg.data = [steering_left, steering_right]
        self.steering_pub.publish(steering_msg)

        wheel_msg = Float64MultiArray()
        wheel_msg.data = [wheel_left, wheel_right]
        self.wheel_pub.publish(wheel_msg)

        self.get_logger().info(
            f"v={v:.3f} omega={omega:.3f} | "
            f"steer_target=({target_left:.3f}, {target_right:.3f}) | "
            f"steer_out=({steering_left:.3f}, {steering_right:.3f}) | "
            f"rear=({wheel_left:.3f}, {wheel_right:.3f})"
        )


def main(args=None):

    rclpy.init(args=args)
    node = AckermannKinematicsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        steering_msg = Float64MultiArray()
        steering_msg.data = [0.0, 0.0]

        wheel_msg = Float64MultiArray()
        wheel_msg.data = [0.0, 0.0]

        node.steering_pub.publish(steering_msg)
        node.wheel_pub.publish(wheel_msg)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()