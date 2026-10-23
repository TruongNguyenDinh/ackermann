#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{
constexpr double kStoppedVelocity = 1e-3;
constexpr double kZero = 1e-6;

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

struct KinematicResult
{
  double steering_left{0.0};
  double steering_right{0.0};
  double rear_left_wheel{0.0};
  double rear_right_wheel{0.0};
};

class AckermannKinematics
{
public:
  AckermannKinematics(
    double wheelbase, double steering_track, double rear_track,
    double wheel_radius)
  : wheelbase_(wheelbase), steering_track_(steering_track), rear_track_(rear_track),
    wheel_radius_(wheel_radius)
  {
  }

  KinematicResult calculate(double velocity, double yaw_rate) const
  {
    KinematicResult result;
    if (std::abs(velocity) < kZero) {
      return result;
    }

    if (std::abs(yaw_rate) < kZero) {
      result.rear_left_wheel = velocity / wheel_radius_;
      result.rear_right_wheel = result.rear_left_wheel;
      return result;
    }

    // Convert body yaw rate to bicycle steering angle, then derive the
    // individual front wheel angles directly from Ackermann geometry.
    const double delta = std::atan(wheelbase_ * yaw_rate / velocity);
    const double tan_delta = std::tan(delta);
    result.steering_left = std::atan2(
      wheelbase_ * tan_delta,
      wheelbase_ - steering_track_ / 2.0 * tan_delta);
    result.steering_right = std::atan2(
      wheelbase_ * tan_delta,
      wheelbase_ + steering_track_ / 2.0 * tan_delta);

    const auto wheel_speeds = rear_wheel_speeds(velocity, delta);
    result.rear_left_wheel = wheel_speeds.first;
    result.rear_right_wheel = wheel_speeds.second;
    return result;
  }

  std::pair<double, double> rear_wheel_speeds(double velocity, double delta) const
  {
    const double steering_factor =
      rear_track_ / (2.0 * wheelbase_) * std::tan(delta);
    const double wheel_speed = velocity / wheel_radius_;
    return {
      wheel_speed * (1.0 - steering_factor),
      wheel_speed * (1.0 + steering_factor)};
  }

private:
  double wheelbase_;
  double steering_track_;
  double rear_track_;
  double wheel_radius_;
};

class AckermannKinematicsNode : public rclcpp::Node
{
public:
  AckermannKinematicsNode()
  : Node("ackermann_kinematics")
  {
    const double wheelbase = declare_parameter<double>("wheelbase", 0.3187);
    const double track_width = declare_parameter<double>("track_width", 0.3118);
    const double front_track = declare_parameter<double>("front_track", track_width);
    const double rear_track = declare_parameter<double>("rear_track", track_width);
    // Match the existing URDF: steering pivots are inboard of wheel centers.
    const double steering_axis_offset = declare_parameter<double>("steering_axis_offset", 0.0369);
    const double steering_track = front_track - 2.0 * steering_axis_offset;
    const double wheel_radius = declare_parameter<double>("wheel_radius", 0.063);
    max_steering_angle_ = declare_parameter<double>("max_steering_angle", 0.261799);
    max_wheel_velocity_ = declare_parameter<double>("max_wheel_velocity", 20.0);
    max_steering_rate_ = declare_parameter<double>("max_steering_rate", 1.0);
    steering_alignment_tolerance_ =
      declare_parameter<double>("steering_alignment_tolerance", 0.035);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);

    for (const double value : {wheelbase, front_track, steering_track, rear_track, wheel_radius,
        max_steering_angle_, max_wheel_velocity_, max_steering_rate_, command_timeout_})
    {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Geometry and motion limits must be finite and positive");
      }
    }
    if (!std::isfinite(steering_axis_offset) || steering_axis_offset < 0.0 ||
      max_steering_angle_ >= std::acos(-1.0) / 2.0 ||
      !std::isfinite(steering_alignment_tolerance_) || steering_alignment_tolerance_ < 0.0)
    {
      throw std::invalid_argument("Invalid steering angle or alignment tolerance");
    }

    wheelbase_ = wheelbase;
    steering_track_ = steering_track;
    kinematics_ = std::make_unique<AckermannKinematics>(
      wheelbase, steering_track, rear_track, wheel_radius);

    steering_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/steering_controller/commands", 10);
    wheel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/wheel_velocity_controller/commands", 10);
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&AckermannKinematicsNode::cmd_vel_callback, this, std::placeholders::_1));
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {
        if (has_last_time_ && std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_time_).count() > command_timeout_)
        {
          publish_stop();
          RCLCPP_WARN(get_logger(), "cmd_vel timed out; stopping rear wheels");
        }
      });

    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), " Ackermann Controller Started");
    RCLCPP_INFO(get_logger(), " wheelbase        : %.4f m", wheelbase);
    RCLCPP_INFO(get_logger(), " front track      : %.4f m", front_track);
    RCLCPP_INFO(get_logger(), " steering pivots  : %.4f m", steering_track);
    RCLCPP_INFO(get_logger(), " rear track       : %.4f m", rear_track);
    RCLCPP_INFO(get_logger(), " wheel radius     : %.4f m", wheel_radius);
    RCLCPP_INFO(get_logger(), " max steering     : %.4f rad", max_steering_angle_);
    RCLCPP_INFO(get_logger(), " max steering rate: %.4f rad/s", max_steering_rate_);
    RCLCPP_INFO(get_logger(), " max wheel vel    : %.4f rad/s", max_wheel_velocity_);
    RCLCPP_INFO(get_logger(), " base_footprint   : REAR AXLE REFERENCE");
    RCLCPP_INFO(get_logger(), "========================================");
  }

  void publish_stop()
  {
    std_msgs::msg::Float64MultiArray steering_msg;
    // Keep steering in place when stopping; do not snap it to straight ahead.
    steering_msg.data = {current_steering_left_, current_steering_right_};
    steering_pub_->publish(steering_msg);

    std_msgs::msg::Float64MultiArray wheel_msg;
    wheel_msg.data = {0.0, 0.0};
    wheel_pub_->publish(wheel_msg);
    pending_steering_direction_ = 0;
    last_motion_direction_ = 0;
    has_last_time_ = false;
  }

private:
  double rate_limit(double target, double current, double dt) const
  {
    if (dt <= 0.0) {
      return current;
    }
    const double max_delta = max_steering_rate_ * dt;
    return current + clamp(target - current, -max_delta, max_delta);
  }

  double estimate_yaw_rate(
    double velocity, double steering_left, double steering_right) const
  {
    double numerator = 0.0;
    double denominator = 0.0;
    const std::pair<double, double> wheels[] = {
      {steering_left, steering_track_ / 2.0},
      {steering_right, -steering_track_ / 2.0}};

    for (const auto & wheel : wheels) {
      const double steering = wheel.first;
      const double lateral_position = wheel.second;
      const double constraint_scale =
        wheelbase_ * std::cos(steering) + lateral_position * std::sin(steering);
      numerator += velocity * constraint_scale * std::sin(steering);
      denominator += constraint_scale * constraint_scale;
    }

    return denominator <= 1e-12 ? 0.0 : numerator / denominator;
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
      publish_stop();
      RCLCPP_WARN(get_logger(), "Rejected non-finite cmd_vel");
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    double dt = 0.0;
    if (has_last_time_) {
      dt = std::min(command_timeout_, std::chrono::duration<double>(now - last_time_).count());
    }
    last_time_ = now;
    has_last_time_ = true;

    double velocity = msg->linear.x;
    double yaw_rate = msg->angular.z;
    const bool stopped = std::abs(velocity) < kStoppedVelocity;
    if (stopped) {
      velocity = 0.0;
    } else {
      const double max_wheel_tangent = std::tan(max_steering_angle_);
      const double max_body_tangent = wheelbase_ * max_wheel_tangent /
        (wheelbase_ + steering_track_ / 2.0 * max_wheel_tangent);
      const double max_yaw_rate = std::abs(velocity) * max_body_tangent / wheelbase_;
      if (std::abs(yaw_rate) > max_yaw_rate) {
        yaw_rate = std::copysign(max_yaw_rate, yaw_rate);
      }
    }

    const auto result = kinematics_->calculate(velocity, yaw_rate);
    double target_left = clamp(
      result.steering_left, -max_steering_angle_, max_steering_angle_);
    double target_right = clamp(
      result.steering_right, -max_steering_angle_, max_steering_angle_);

    if (stopped) {
      if (pending_steering_direction_ != 0) {
        target_left = pending_steering_left_;
        target_right = pending_steering_right_;
      } else {
        target_left = current_steering_left_;
        target_right = current_steering_right_;
      }
    } else {
      const int direction = velocity > 0.0 ? 1 : -1;
      const double steering_error = std::max(
        std::abs(target_left - current_steering_left_),
        std::abs(target_right - current_steering_right_));
      if ((last_motion_direction_ == 0 &&
        steering_error > steering_alignment_tolerance_) ||
        (last_motion_direction_ != 0 && direction != last_motion_direction_) ||
        pending_steering_direction_ != 0)
      {
        pending_steering_direction_ = direction;
        pending_steering_left_ = target_left;
        pending_steering_right_ = target_right;
      }
    }

    bool steering_wait = pending_steering_direction_ != 0;
    const double limited_left = rate_limit(target_left, current_steering_left_, dt);
    const double limited_right = rate_limit(target_right, current_steering_right_, dt);
    const auto curvature = [this](double angle, double lateral_position) {
        return std::sin(angle) /
               (wheelbase_ * std::cos(angle) + lateral_position * std::sin(angle));
      };
    const double left_curvature = curvature(limited_left, steering_track_ / 2.0);
    const double right_curvature = curvature(limited_right, -steering_track_ / 2.0);
    // Both wheels share one curvature. Choose the slower wheel's permitted
    // step so neither steering rate limit is exceeded during a transition.
    const double limited_curvature = target_left >= current_steering_left_ ?
      std::min(left_curvature, right_curvature) : std::max(left_curvature, right_curvature);
    const auto limited_steering = kinematics_->calculate(1.0, limited_curvature);
    const double steering_left = limited_steering.steering_left;
    const double steering_right = limited_steering.steering_right;
    current_steering_left_ = steering_left;
    current_steering_right_ = steering_right;

    if (steering_wait) {
      const bool aligned =
        std::abs(pending_steering_left_ - steering_left) <= steering_alignment_tolerance_ &&
        std::abs(pending_steering_right_ - steering_right) <= steering_alignment_tolerance_;
      if (aligned) {
        last_motion_direction_ = pending_steering_direction_;
        pending_steering_direction_ = 0;
        steering_wait = false;
      }
    } else if (!stopped) {
      last_motion_direction_ = velocity > 0.0 ? 1 : -1;
    }

    double actual_delta = 0.0;
    if (std::abs(velocity) >= kZero) {
      const double actual_omega = estimate_yaw_rate(
        velocity, steering_left, steering_right);
      actual_delta = std::atan(wheelbase_ * actual_omega / velocity);
    }
    const auto wheel_targets = kinematics_->rear_wheel_speeds(velocity, actual_delta);
    // Scale both wheels together to preserve curvature at the motor speed limit.
    const double peak_speed = std::max(
      std::abs(wheel_targets.first), std::abs(wheel_targets.second));
    const double speed_scale = peak_speed > max_wheel_velocity_ ?
      max_wheel_velocity_ / peak_speed : 1.0;
    double wheel_left = wheel_targets.first * speed_scale;
    double wheel_right = wheel_targets.second * speed_scale;
    if (steering_wait) {
      wheel_left = 0.0;
      wheel_right = 0.0;
    }

    std_msgs::msg::Float64MultiArray steering_msg;
    steering_msg.data = {steering_left, steering_right};
    steering_pub_->publish(steering_msg);

    std_msgs::msg::Float64MultiArray wheel_msg;
    wheel_msg.data = {wheel_left, wheel_right};
    wheel_pub_->publish(wheel_msg);

    RCLCPP_DEBUG(
      get_logger(),
      "v=%.3f omega=%.3f target=(%.3f, %.3f) steer=(%.3f, %.3f) "
      "steering_wait=%s wheel=(%.3f, %.3f)",
      velocity, yaw_rate, target_left, target_right, steering_left, steering_right,
      steering_wait ? "true" : "false", wheel_left, wheel_right);
  }

  double wheelbase_{0.0};
  double steering_track_{0.0};
  double max_steering_angle_{0.0};
  double max_wheel_velocity_{0.0};
  double max_steering_rate_{0.0};
  double steering_alignment_tolerance_{0.0};
  double command_timeout_{0.5};
  double current_steering_left_{0.0};
  double current_steering_right_{0.0};
  int last_motion_direction_{0};
  int pending_steering_direction_{0};
  double pending_steering_left_{0.0};
  double pending_steering_right_{0.0};
  bool has_last_time_{false};
  std::chrono::steady_clock::time_point last_time_{};
  std::unique_ptr<AckermannKinematics> kinematics_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AckermannKinematicsNode>();
  rclcpp::spin(node);
  node->publish_stop();
  rclcpp::shutdown();
  return 0;
}
