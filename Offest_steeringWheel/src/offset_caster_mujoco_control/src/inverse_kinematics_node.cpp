#include "offset_caster_mujoco_control/inverse_kinematics.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace offset_caster_mujoco_control
{

class InverseKinematicsNode : public rclcpp::Node
{
public:
  InverseKinematicsNode()
  : Node("inverse_kinematics_node"),
    last_twist_time_(0, 0, get_clock()->get_clock_type())
  {
    const double wheel_radius = declare_parameter<double>("wheel_radius", 0.075);
    const double caster_offset = declare_parameter<double>("caster_offset", 0.105);
    const double max_steering_speed = declare_parameter<double>("max_steering_speed", 6.283);
    const double max_wheel_speed = declare_parameter<double>("max_wheel_speed", 40.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);
    const double update_rate = declare_parameter<double>("update_rate", 50.0);

    const auto pivot_x = declare_parameter<std::vector<double>>(
      "pivot_x", {0.5, 0.5, -0.5, -0.5});
    const auto pivot_y = declare_parameter<std::vector<double>>(
      "pivot_y", {0.35, -0.35, -0.35, 0.35});
    steering_joint_names_ = declare_parameter<std::vector<std::string>>(
      "steering_joint_names",
      {"steering_joint_1", "steering_joint_2", "steering_joint_3", "steering_joint_4"});
    wheel_joint_names_ = declare_parameter<std::vector<std::string>>(
      "wheel_joint_names",
      {"wheel_joint_1", "wheel_joint_2", "wheel_joint_3", "wheel_joint_4"});

    validate_parameters(pivot_x, pivot_y, update_rate);

    std::array<CasterGeometry, kCasterCount> geometry{};
    for (std::size_t i = 0; i < kCasterCount; ++i) {
      geometry[i] = {pivot_x[i], pivot_y[i], wheel_radius, caster_offset};
    }
    kinematics_ = std::make_unique<InverseKinematics>(
      geometry, max_steering_speed, max_wheel_speed);

    command_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/offset_caster/joint_velocity_command", 10);
    twist_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&InverseKinematicsNode::twist_callback, this, std::placeholders::_1));
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&InverseKinematicsNode::joint_state_callback, this, std::placeholders::_1));

    const auto update_period = std::chrono::duration<double>(1.0 / update_rate);
    update_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(update_period),
      std::bind(&InverseKinematicsNode::update, this));

    RCLCPP_INFO(
      get_logger(),
      "Inverse kinematics ready: /cmd_vel + /joint_states -> "
      "/offset_caster/joint_velocity_command");
  }

private:
  void validate_parameters(
    const std::vector<double> & pivot_x,
    const std::vector<double> & pivot_y,
    const double update_rate) const
  {
    if (pivot_x.size() != kCasterCount || pivot_y.size() != kCasterCount ||
      steering_joint_names_.size() != kCasterCount ||
      wheel_joint_names_.size() != kCasterCount)
    {
      throw std::invalid_argument("Exactly four caster positions and joint names are required");
    }
    if (!std::isfinite(command_timeout_) || command_timeout_ <= 0.0) {
      throw std::invalid_argument("command_timeout must be finite and positive");
    }
    if (!std::isfinite(update_rate) || update_rate <= 0.0) {
      throw std::invalid_argument("update_rate must be finite and positive");
    }
  }

  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    const ChassisTwist candidate{
      message->linear.x,
      message->linear.y,
      message->angular.z};

    if (!std::isfinite(candidate.linear_x) || !std::isfinite(candidate.linear_y) ||
      !std::isfinite(candidate.angular_z))
    {
      RCLCPP_WARN(get_logger(), "Rejected non-finite /cmd_vel command");
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    target_twist_ = candidate;
    last_twist_time_ = now();
    has_twist_ = true;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    for (std::size_t caster_index = 0; caster_index < kCasterCount; ++caster_index) {
      const auto name_iterator = std::find(
        message->name.begin(), message->name.end(), steering_joint_names_[caster_index]);
      if (name_iterator == message->name.end()) {
        continue;
      }

      const auto message_index = static_cast<std::size_t>(
        std::distance(message->name.begin(), name_iterator));
      if (message_index >= message->position.size() ||
        !std::isfinite(message->position[message_index]))
      {
        continue;
      }

      steering_angles_[caster_index] = message->position[message_index];
      steering_angle_received_[caster_index] = true;
    }

    has_all_steering_angles_ = std::all_of(
      steering_angle_received_.begin(), steering_angle_received_.end(),
      [](const bool received) {return received;});
  }

  void update()
  {
    ChassisTwist twist{};
    std::array<double, kCasterCount> steering_angles{};
    bool has_steering_feedback = false;
    bool command_timed_out = true;
    const auto update_time = now();

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      steering_angles = steering_angles_;
      has_steering_feedback = has_all_steering_angles_;

      if (has_twist_) {
        const double command_age = (update_time - last_twist_time_).seconds();
        command_timed_out = command_age < 0.0 || command_age > command_timeout_;
        if (!command_timed_out) {
          twist = target_twist_;
        }
      }
    }

    if (!has_steering_feedback) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for all four steering joint positions on /joint_states");
      return;
    }

    const JointVelocityCommand command = kinematics_->solve(twist, steering_angles);

    trajectory_msgs::msg::JointTrajectory message;
    message.header.stamp = update_time;
    message.joint_names.reserve(2 * kCasterCount);
    message.joint_names.insert(
      message.joint_names.end(), steering_joint_names_.begin(), steering_joint_names_.end());
    message.joint_names.insert(
      message.joint_names.end(), wheel_joint_names_.begin(), wheel_joint_names_.end());

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.velocities.reserve(2 * kCasterCount);
    point.velocities.insert(
      point.velocities.end(), command.steering.begin(), command.steering.end());
    point.velocities.insert(point.velocities.end(), command.wheel.begin(), command.wheel.end());
    message.points.push_back(std::move(point));
    command_publisher_->publish(message);

    if (command_timed_out) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000, "No recent /cmd_vel; publishing zero command");
    }
    if (command.scale < 1.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Joint command uniformly scaled by %.3f to preserve caster kinematics", command.scale);
    }
  }

  std::unique_ptr<InverseKinematics> kinematics_;
  std::vector<std::string> steering_joint_names_;
  std::vector<std::string> wheel_joint_names_;
  double command_timeout_{0.5};

  std::mutex state_mutex_;
  ChassisTwist target_twist_{};
  std::array<double, kCasterCount> steering_angles_{};
  std::array<bool, kCasterCount> steering_angle_received_{};
  rclcpp::Time last_twist_time_;
  bool has_twist_{false};
  bool has_all_steering_angles_{false};

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace offset_caster_mujoco_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<offset_caster_mujoco_control::InverseKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
