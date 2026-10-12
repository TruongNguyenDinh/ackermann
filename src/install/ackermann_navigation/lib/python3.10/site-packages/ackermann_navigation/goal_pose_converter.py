"""Convert RViz center-of-vehicle goals to rear-axle Nav2 goals."""

import copy
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class GoalPoseConverter(Node):
    """Shift center-referenced goals to the Ackermann rear axle."""

    def __init__(self):
        super().__init__('goal_pose_converter')
        self.declare_parameter('wheelbase', 0.3187)
        self.wheelbase = float(self.get_parameter('wheelbase').value)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose_center',
            self.goal_callback,
            10,
        )
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.get_logger().info(
            'Converting center goals to rear-axle goals '
            f'(wheelbase={self.wheelbase:.4f} m)'
        )

    def goal_callback(self, center_goal):
        """Publish the rear-axle point half a wheelbase behind the goal."""
        orientation = center_goal.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z +
                orientation.x * orientation.y
            ),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
        offset = self.wheelbase / 2.0

        rear_axle_goal = copy.deepcopy(center_goal)
        rear_axle_goal.pose.position.x -= offset * math.cos(yaw)
        rear_axle_goal.pose.position.y -= offset * math.sin(yaw)
        self.goal_pub.publish(rear_axle_goal)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
