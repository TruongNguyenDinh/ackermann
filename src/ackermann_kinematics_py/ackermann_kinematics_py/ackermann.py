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
        wheel_radius: float,
        front_track: float = None,
        rear_track: float = None,
    ):
        self.L = wheelbase
        # track_width remains as a compatibility fallback for existing users.
        self.front_track = front_track if front_track is not None else track_width
        self.rear_track = rear_track if rear_track is not None else track_width
        self.Rw = wheel_radius

    def calculate(self, v: float, omega: float):

        # ============================================================
        # 1. XE DỪNG
        # ============================================================

        if abs(v) < 1e-6:
            return {
                "steering_left": 0.0,
                "steering_right": 0.0,
                "rear_left_wheel": 0.0,
                "rear_right_wheel": 0.0,
            }

        # ============================================================
        # 2. ĐI THẲNG
        # ============================================================

        if abs(omega) < 1e-6:

            wheel_velocity = v / self.Rw

            return {
                "steering_left": 0.0,
                "steering_right": 0.0,
                "rear_left_wheel": wheel_velocity,
                "rear_right_wheel": wheel_velocity,
            }

        R = v / omega

        R_left = R - self.front_track / 2.0
        R_right = R + self.front_track / 2.0

        if R > 0.0:


            steering_left = math.atan2(
                self.L,
                R_left
            )

            steering_right = math.atan2(
                self.L,
                R_right
            )

        else:

            steering_left = -math.atan2(
                self.L,
                abs(R_left)
            )

            steering_right = -math.atan2(
                self.L,
                abs(R_right)
            )

        rear_left_radius = R - self.rear_track / 2.0
        rear_right_radius = R + self.rear_track / 2.0
        v_left = omega * rear_left_radius
        v_right = omega * rear_right_radius

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

        self.declare_parameter(
            "wheelbase",
            0.3187
        )

        self.declare_parameter(
            "track_width",
            0.3118
        )

        # Separate axle tracks; defaults preserve the previous symmetric
        # estimate until the drawing dimensions are measured.
        self.declare_parameter("front_track", 0.3118)
        self.declare_parameter("rear_track", 0.3118)

        self.declare_parameter(
            "wheel_radius",
            0.05
        )

        self.declare_parameter(
            "max_steering_angle",
            0.261799
        )

        self.declare_parameter(
            "max_wheel_velocity",
            20.0
        )

        self.declare_parameter(
            "max_steering_rate",
            1.0
        )

        self.wheelbase = self.get_parameter(
            "wheelbase"
        ).value

        self.track_width = self.get_parameter(
            "track_width"
        ).value
        self.front_track = self.get_parameter("front_track").value
        self.rear_track = self.get_parameter("rear_track").value

        self.wheel_radius = self.get_parameter(
            "wheel_radius"
        ).value

        self.max_steering_angle = self.get_parameter(
            "max_steering_angle"
        ).value

        self.max_wheel_velocity = self.get_parameter(
            "max_wheel_velocity"
        ).value

        self.max_steering_rate = self.get_parameter(
            "max_steering_rate"
        ).value

        self.kinematics = AckermannKinematics(
            wheelbase=self.wheelbase,
            track_width=self.track_width,
            wheel_radius=self.wheel_radius,
            front_track=self.front_track,
            rear_track=self.rear_track,
        )

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

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            " Ackermann Controller Started"
        )

        self.get_logger().info(
            f" wheelbase        : {self.wheelbase:.4f} m"
        )

        self.get_logger().info(
            f" front track      : {self.front_track:.4f} m"
        )
        self.get_logger().info(
            f" rear track       : {self.rear_track:.4f} m"
        )

        self.get_logger().info(
            f" wheel radius     : {self.wheel_radius:.4f} m"
        )

        self.get_logger().info(
            f" max steering     : {self.max_steering_angle:.4f} rad"
        )

        self.get_logger().info(
            f" max steering rate: {self.max_steering_rate:.4f} rad/s"
        )

        self.get_logger().info(
            f" max wheel vel    : {self.max_wheel_velocity:.4f} rad/s"
        )

        self.get_logger().info(
            " base_footprint   : REAR AXLE REFERENCE"
        )

        self.get_logger().info(
            "========================================"
        )

    def _rate_limit(
        self,
        target: float,
        current: float,
        dt: float
    ) -> float:

        if dt <= 0.0:
            return current

        max_delta = self.max_steering_rate * dt

        delta = target - current

        if delta > max_delta:
            delta = max_delta

        elif delta < -max_delta:
            delta = -max_delta

        return current + delta

    # ================================================================
    # CMD VEL CALLBACK
    # ================================================================

    def cmd_vel_callback(
        self,
        msg: Twist
    ):

        # ============================================================
        # TIME
        # ============================================================

        now = time.monotonic()

        if self._last_time is None:
            dt = 0.0

        else:
            dt = now - self._last_time

        self._last_time = now

        # ============================================================
        # CMD VEL
        # ============================================================

        v = msg.linear.x
        omega = msg.angular.z
        stopped = abs(v) < 1e-3
        if stopped:
            # MPPI may briefly command zero speed while selecting the next
            # feasible Ackermann arc. Keep the steering angle during that
            # pause instead of centering it and forcing the controller to
            # steer from scratch on the next command.
            v = 0.0
        else:
            # Keep cmd_vel feasible for the physical steering limit. The
            # planner uses the minimum rear-axle radius for the same geometry.
            minimum_turning_radius = (
                self.wheelbase / math.tan(self.max_steering_angle)
                + self.front_track / 2.0
            )
            max_yaw_rate = abs(v) / minimum_turning_radius
            if abs(omega) > max_yaw_rate:
                omega = math.copysign(max_yaw_rate, omega)

        # ============================================================
        # INVERSE ACKERMANN KINEMATICS
        # ============================================================

        result = self.kinematics.calculate(
            v,
            omega
        )

        # ============================================================
        # STEERING ANGLE LIMIT
        # ============================================================

        target_left = max(
            -self.max_steering_angle,
            min(
                result["steering_left"],
                self.max_steering_angle
            )
        )

        target_right = max(
            -self.max_steering_angle,
            min(
                result["steering_right"],
                self.max_steering_angle
            )
        )

        if stopped:
            target_left = self._current_steering_left
            target_right = self._current_steering_right

        # ============================================================
        # STEERING RATE LIMIT
        # ============================================================

        steering_left = self._rate_limit(
            target_left,
            self._current_steering_left,
            dt
        )

        steering_right = self._rate_limit(
            target_right,
            self._current_steering_right,
            dt
        )

        self._current_steering_left = steering_left
        self._current_steering_right = steering_right
        # Match wheel speeds to the steering angles that can actually be
        # reached after rate limiting. Sending target-curvature wheel speeds
        # while the steering is still moving causes the vehicle to cut across
        # the controller's predicted path.
        if (
            abs(v) < 1e-6
            or abs(steering_left) < 1e-6
            or abs(steering_right) < 1e-6
        ):
            actual_omega = 0.0
        else:
            radius_left = self.wheelbase / math.tan(steering_left)
            radius_right = self.wheelbase / math.tan(steering_right)
            turning_radius = (radius_left + radius_right) / 2.0
            actual_omega = (
                v / turning_radius if abs(turning_radius) > 1e-6 else 0.0
            )

        wheel_left_target = (
            v - actual_omega * self.rear_track / 2.0
        ) / self.wheel_radius
        wheel_right_target = (
            v + actual_omega * self.rear_track / 2.0
        ) / self.wheel_radius

        # WHEEL VELOCITY LIMIT
        wheel_left = max(
            -self.max_wheel_velocity,
            min(
                wheel_left_target,
                self.max_wheel_velocity
            )
        )

        wheel_right = max(
            -self.max_wheel_velocity,
            min(
                wheel_right_target,
                self.max_wheel_velocity
            )
        )
        # PUBLISH STEERING

        steering_msg = Float64MultiArray()

        steering_msg.data = [steering_left,steering_right]

        self.steering_pub.publish(steering_msg)

        # PUBLISH WHEEL VELOCITY
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [wheel_left,wheel_right]
        self.wheel_pub.publish(wheel_msg)
        # DEBUG
        self.get_logger().debug(
            f"v={v:.3f} "
            f"omega={omega:.3f} | "
            f"target=({target_left:.3f}, "
            f"{target_right:.3f}) | "
            f"steer=({steering_left:.3f}, "
            f"{steering_right:.3f}) | "
            f"wheel=({wheel_left:.3f}, "
            f"{wheel_right:.3f})"
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
        steering_msg.data = [0.0,0.0]
        node.steering_pub.publish(steering_msg)

        wheel_msg = Float64MultiArray()

        wheel_msg.data = [0.0,0.0]
        node.wheel_pub.publish(wheel_msg)
        node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()
