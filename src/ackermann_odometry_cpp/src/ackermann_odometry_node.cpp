#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

class AckermannOdometry : public rclcpp::Node
{
public:
  AckermannOdometry()
  : Node("ackermann_odometry")
  {
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.063);
    rear_track_ = declare_parameter<double>("rear_track", 0.3118);
    max_dt_ = declare_parameter<double>("max_dt", 0.5);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    for (const double value : {wheel_radius_, rear_track_, max_dt_}) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(
                "wheel_radius, rear_track and max_dt must be positive and finite");
      }
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&AckermannOdometry::joint_state_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(),
      "Rear axle odometry: radius=%.4f m, track=%.4f m; TF odom -> base_footprint: %s",
      wheel_radius_, rear_track_, publish_tf_ ? "enabled" : "disabled");
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    double left = 0.0;
    double right = 0.0;
    bool has_left = false;
    bool has_right = false;
    for (size_t i = 0; i < msg->name.size() && i < msg->velocity.size(); ++i) {
      if (msg->name[i] == "rear_left_wheel_joint") {
        left = msg->velocity[i];
        has_left = true;
      } else if (msg->name[i] == "rear_right_wheel_joint") {
        right = msg->velocity[i];
        has_right = true;
      }
    }
    // Both measured wheel velocities (rad/s) must belong to this sample.
    // Do not treat a missing encoder as a stopped wheel or integrate across bad data.
    if (!has_left || !has_right || !std::isfinite(left) || !std::isfinite(right)) {
      has_stamp_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping JointState: both finite rear wheel velocities are required");
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
    if (!has_stamp_) {
      last_stamp_ = stamp;
      has_stamp_ = true;
      return;
    }
    const double dt = (stamp - last_stamp_).seconds();
    if (dt == 0.0) {
      return;
    }
    last_stamp_ = stamp;
    if (dt < 0.0 || dt > max_dt_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 2000,
        "Skipping odometry interval %.3f s: clock moved backwards or encoder gap exceeded max_dt",
        dt);
      return;
    }

    const double v_left = left * wheel_radius_;
    const double v_right = right * wheel_radius_;
    const double velocity = (v_left + v_right) / 2.0;
    const double yaw_rate = (v_right - v_left) / rear_track_;
    const double turn = yaw_rate * dt;
    const double half_turn = turn / 2.0;
    // Exact constant-twist arc integration, continuous through straight motion.
    const double sinc = std::abs(half_turn) < 1e-6 ?
      1.0 - half_turn * half_turn / 6.0 : std::sin(half_turn) / half_turn;
    const double distance = velocity * dt * sinc;
    x_ += distance * std::cos(yaw_ + half_turn);
    y_ += distance * std::sin(yaw_ + half_turn);
    yaw_ = std::remainder(yaw_ + turn, 2.0 * std::acos(-1.0));

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
    odom.pose.pose.orientation.w = std::cos(yaw_ / 2.0);
    odom.twist.twist.linear.x = velocity;
    odom.twist.twist.angular.z = yaw_rate;
    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = odom.child_frame_id;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  double wheel_radius_{0.063};
  double rear_track_{0.3118};
  double max_dt_{0.5};
  bool publish_tf_{true};
  bool has_stamp_{false};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannOdometry>());
  rclcpp::shutdown();
  return 0;
}
