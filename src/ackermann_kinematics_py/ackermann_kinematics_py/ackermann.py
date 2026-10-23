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

        # Convert body yaw rate to the equivalent bicycle steering angle.
        # The front wheel angles and rear wheel speeds below are calculated
        # directly from delta and track width, without constructing a turn
        # radius.
        delta = math.atan(self.L * omega / v)
        tan_delta = math.tan(delta)

        steering_left = math.atan2(
            self.L * tan_delta,
            self.L - self.front_track / 2.0 * tan_delta,
        )
        steering_right = math.atan2(
            self.L * tan_delta,
            self.L + self.front_track / 2.0 * tan_delta,
        )

        wheel_left, wheel_right = self.rear_wheel_speeds(v, delta)

        return {
            "steering_left": steering_left,
            "steering_right": steering_right,
            "rear_left_wheel": wheel_left,
            "rear_right_wheel": wheel_right,
        }

    def rear_wheel_speeds(self, v: float, delta: float):
        """Return rear wheel angular speeds from body speed and steering."""
        steering_factor = self.rear_track / (2.0 * self.L) * math.tan(delta)
        wheel_speed = v / self.Rw
        return (
            wheel_speed * (1.0 - steering_factor),
            wheel_speed * (1.0 + steering_factor),
        )


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

        # Symmetric axle tracks derived from frame width, tire width and
        # the specified 16.5 mm inner clearance.
        self.declare_parameter("front_track", 0.3118)
        self.declare_parameter("rear_track", 0.3118)

        self.declare_parameter(
            "wheel_radius",
            0.063
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

        self.declare_parameter(
            "steering_alignment_tolerance",
            0.035
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

        self.steering_alignment_tolerance = self.get_parameter(
            "steering_alignment_tolerance"
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
        self._last_motion_direction = 0
        self._pending_steering_direction = 0
        self._pending_steering_left = 0.0
        self._pending_steering_right = 0.0

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

    def _estimate_yaw_rate(
        self,
        velocity: float,
        steering_left: float,
        steering_right: float,
    ) -> float:
        """Estimate body yaw rate from the current front wheel angles.

        During steering rate limiting the two wheels may not yet form an
        exact Ackermann pair. Estimate the yaw rate that best satisfies both
        wheel rolling constraints instead of treating either near-zero angle
        as a straight steering command.
        """
        numerator = 0.0
        denominator = 0.0

        for steering, lateral_position in (
            (steering_left, self.front_track / 2.0),
            (steering_right, -self.front_track / 2.0),
        ):
            constraint_scale = (
                self.wheelbase * math.cos(steering)
                + lateral_position * math.sin(steering)
            )
            numerator += (
                velocity * constraint_scale * math.sin(steering)
            )
            denominator += constraint_scale * constraint_scale

        if denominator <= 1e-12:
            return 0.0

        return numerator / denominator

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
            # feasible Ackermann arc. Hold the steering target during an
            # ordinary pause, but continue aligning it if a gear change is
            # waiting for the steering actuators.
            v = 0.0
        else:
            # Limit body steering so the inner front wheel stays within its
            # physical steering joint limit.
            max_wheel_tangent = math.tan(self.max_steering_angle)
            max_body_tangent = (
                self.wheelbase * max_wheel_tangent
                / (
                    self.wheelbase
                    + self.front_track / 2.0 * max_wheel_tangent
                )
            )
            max_yaw_rate = abs(v) * max_body_tangent / self.wheelbase
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
            if self._pending_steering_direction != 0:
                target_left = self._pending_steering_left
                target_right = self._pending_steering_right
            else:
                target_left = self._current_steering_left
                target_right = self._current_steering_right
        else:
            direction = 1 if v > 0.0 else -1
            steering_error = max(
                abs(target_left - self._current_steering_left),
                abs(target_right - self._current_steering_right),
            )
            if (
                self._last_motion_direction == 0
                and steering_error > self.steering_alignment_tolerance
            ) or (
                self._last_motion_direction != 0
                and direction != self._last_motion_direction
            ) or self._pending_steering_direction != 0:
                self._pending_steering_direction = direction
                self._pending_steering_left = target_left
                self._pending_steering_right = target_right

        steering_wait = self._pending_steering_direction != 0

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
        if steering_wait:
            aligned = (
                abs(self._pending_steering_left - steering_left)
                <= self.steering_alignment_tolerance
                and abs(self._pending_steering_right - steering_right)
                <= self.steering_alignment_tolerance
            )
            if aligned:
                self._last_motion_direction = (
                    self._pending_steering_direction
                )
                self._pending_steering_direction = 0
                steering_wait = False
        elif not stopped:
            self._last_motion_direction = 1 if v > 0.0 else -1
        # Match wheel speeds to the steering angles that can actually be
        # reached after rate limiting. Sending target-curvature wheel speeds
        # while the steering is still moving causes the vehicle to cut across
        # the controller's predicted path.
        if abs(v) < 1e-6:
            actual_omega = 0.0
            actual_delta = 0.0
        else:
            actual_omega = self._estimate_yaw_rate(
                v,
                steering_left,
                steering_right,
            )
            actual_delta = math.atan(
                self.wheelbase * actual_omega / v
            )

        wheel_left_target, wheel_right_target = (
            self.kinematics.rear_wheel_speeds(v, actual_delta)
        )

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
        if steering_wait:
            # A car cannot change travel direction and instantly adopt the
            # opposite steering angle. Keep the wheels stopped until both
            # steering joints reach the angle required for the next arc.
            wheel_left = 0.0
            wheel_right = 0.0
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
            f"steering_wait={steering_wait} | "
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
