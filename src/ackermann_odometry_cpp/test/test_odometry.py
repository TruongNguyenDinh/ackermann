"""Exercise the compiled node using encoder samples on private test topics."""

import math
import os
import subprocess
import time
import uuid

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


@pytest.mark.parametrize('publish_tf', [True, False])
def test_encoder_odometry(publish_tf):
    prefix = '/odom_check_' + uuid.uuid4().hex
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('odometry_check', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    odometry = []
    transforms = []
    publisher = node.create_publisher(
        JointState, prefix + '/joint_states', qos_profile_sensor_data)
    node.create_subscription(Odometry, prefix + '/odom', odometry.append, 10)
    node.create_subscription(TFMessage, prefix + '/tf', transforms.append, 10)
    process = subprocess.Popen([
        os.environ['ODOMETRY_EXECUTABLE'], '--ros-args',
        '-p', 'publish_tf:=' + str(publish_tf).lower(),
        '-r', '/joint_states:=' + prefix + '/joint_states',
        '-r', '/odom:=' + prefix + '/odom',
        '-r', '/tf:=' + prefix + '/tf',
    ])

    def spin(duration=0.08):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)

    def send(stamp, velocity=0.0, yaw_rate=0.0, malformed=None):
        msg = JointState()
        nanoseconds = round(stamp * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(nanoseconds, 10**9)
        # Reversed joint order verifies lookup by name, not array index.
        msg.name = ['rear_right_wheel_joint', 'rear_left_wheel_joint']
        msg.velocity = [
            (velocity + yaw_rate * 0.3118 / 2) / 0.063,
            (velocity - yaw_rate * 0.3118 / 2) / 0.063,
        ]
        if malformed == 'missing':
            msg.velocity.pop()
        elif malformed == 'nan':
            msg.velocity[0] = float('nan')
        publisher.publish(msg)
        spin()

    try:
        deadline = time.monotonic() + 10
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            assert process.poll() is None
            spin()
        assert publisher.get_subscription_count() == 1
        spin(0.3)
        send(1.0, 1.0)
        assert not odometry
        send(1.1, 1.0)
        assert odometry[-1].pose.pose.position.x == pytest.approx(0.1)
        send(1.2, 0.5, 0.4)
        pose = odometry[-1].pose.pose
        assert pose.position.x == pytest.approx(0.1 + 1.25 * math.sin(0.04))
        assert pose.position.y == pytest.approx(1.25 * (1 - math.cos(0.04)))
        assert pose.orientation.z == pytest.approx(math.sin(0.02))
        assert odometry[-1].twist.twist.angular.z == pytest.approx(0.4)
        send(1.3, -0.5, -0.4)
        assert odometry[-1].pose.pose.position.x == pytest.approx(0.1)
        assert odometry[-1].pose.pose.position.y == pytest.approx(0.0, abs=1e-12)
        count = len(odometry)
        send(1.4, malformed='missing')
        send(1.5, malformed='nan')
        send(1.6)
        assert len(odometry) == count
        send(1.7)
        assert len(odometry) == count + 1
        send(1.7)  # Duplicate sample.
        send(1.0)  # Reset clock.
        send(5.0, 1.0)  # Gap: do not extrapolate speed over unobserved time.
        assert len(odometry) == count + 1
        send(5.1)
        assert odometry[-1].pose.pose.position.x == pytest.approx(0.1)
        assert odometry[-1].header.frame_id == 'odom'
        assert odometry[-1].child_frame_id == 'base_footprint'
        if publish_tf:
            transform = transforms[-1].transforms[0]
            assert transform.header == odometry[-1].header
            assert transform.child_frame_id == 'base_footprint'
            assert transform.transform.translation.x == pytest.approx(0.1)
        else:
            assert not transforms
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
