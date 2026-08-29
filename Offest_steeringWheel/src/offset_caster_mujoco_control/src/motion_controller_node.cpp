#include "offset_caster_mujoco_control/frame_transforms.hpp"
#include "offset_caster_mujoco_control/motion_controller.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <offset_caster_interfaces/msg/motion_command.hpp>
#include <offset_caster_interfaces/msg/motion_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace offset_caster_mujoco_control
{

class MotionControllerNode : public rclcpp::Node
{
public:
  MotionControllerNode()
  : Node("motion_controller_node")
  {
    MotionControllerParameters parameters;
    const double update_rate = declare_parameter<double>("update_rate", 50.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);
    state_timeout_ = declare_parameter<double>("state_timeout", 0.5);
    parameters.max_linear_speed = declare_parameter<double>("max_linear_speed", 0.5);
    parameters.max_angular_speed = declare_parameter<double>("max_angular_speed", 0.8);
    parameters.max_linear_acceleration = declare_parameter<double>(
      "max_linear_acceleration", 1.0);
    parameters.max_angular_acceleration = declare_parameter<double>(
      "max_angular_acceleration", 2.0);
    parameters.position_gain = declare_parameter<double>("position_gain", 1.0);
    parameters.yaw_gain = declare_parameter<double>("yaw_gain", 2.0);
    parameters.yaw_rate_damping = declare_parameter<double>("yaw_rate_damping", 0.15);
    parameters.position_tolerance = declare_parameter<double>("position_tolerance", 0.03);
    parameters.yaw_tolerance = declare_parameter<double>("yaw_tolerance", 0.03);
    parameters.reached_dwell_time = declare_parameter<double>("reached_dwell_time", 0.2);

    if (!std::isfinite(update_rate) || update_rate <= 0.0 ||
      !std::isfinite(command_timeout_) || command_timeout_ <= 0.0 ||
      !std::isfinite(state_timeout_) || state_timeout_ <= 0.0)
    {
      throw std::invalid_argument("Update rate and timeouts must be finite and positive");
    }
    update_period_ = 1.0 / update_rate;
    controller_ = std::make_unique<MotionController>(parameters);

    twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_publisher_ = create_publisher<offset_caster_interfaces::msg::MotionStatus>(
      "/offset_caster/motion_status", 10);
    command_subscription_ =
      create_subscription<offset_caster_interfaces::msg::MotionCommand>(
      "/offset_caster/motion_command", 10,
      std::bind(&MotionControllerNode::command_callback, this, std::placeholders::_1));
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&MotionControllerNode::odometry_callback, this, std::placeholders::_1));
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&MotionControllerNode::imu_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(update_period_)),
      std::bind(&MotionControllerNode::update, this));

    RCLCPP_INFO(
      get_logger(), "Six-mode motion controller ready: /offset_caster/motion_command -> /cmd_vel");
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  static bool finite(const offset_caster_interfaces::msg::MotionCommand & command)
  {
    return std::isfinite(command.x) && std::isfinite(command.y) &&
           std::isfinite(command.target_yaw) && std::isfinite(command.yaw_rate);
  }

  void command_callback(const offset_caster_interfaces::msg::MotionCommand::SharedPtr message)
  {
    if (!is_valid_motion_mode(message->mode) || !finite(*message)) {
      fault_detail_ = "rejected invalid motion command";
      RCLCPP_WARN(get_logger(), "%s", fault_detail_.c_str());
      return;
    }
    latest_command_ = *message;
    last_command_time_ = std::chrono::steady_clock::now();
    has_command_ = true;
    stop_sent_ = false;
    fault_detail_.clear();
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    robot_state_.world_x = message->pose.pose.position.x;
    robot_state_.world_y = message->pose.pose.position.y;
    last_odometry_time_ = std::chrono::steady_clock::now();
    has_odometry_ = true;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    try {
      robot_state_.yaw = quaternion_to_yaw(
        message->orientation.w, message->orientation.x,
        message->orientation.y, message->orientation.z);
    } catch (const std::exception & error) {
      fault_detail_ = error.what();
      return;
    }
    robot_state_.yaw_rate = message->angular_velocity.z;
    if (!std::isfinite(robot_state_.yaw_rate)) {
      fault_detail_ = "received non-finite IMU yaw rate";
      return;
    }
    last_imu_time_ = std::chrono::steady_clock::now();
    has_imu_ = true;
  }

  static double age_seconds(const SteadyTime & now, const SteadyTime & then)
  {
    return std::chrono::duration<double>(now - then).count();
  }

  void publish_twist(const ChassisTwist & twist)
  {
    geometry_msgs::msg::Twist message;
    message.linear.x = twist.linear_x;
    message.linear.y = twist.linear_y;
    message.angular.z = twist.angular_z;
    twist_publisher_->publish(message);
  }

  void publish_status(
    const MotionControllerOutput & output, const MotionState state,
    const std::string & detail)
  {
    offset_caster_interfaces::msg::MotionStatus message;
    message.header.stamp = now();
    message.header.frame_id = "world";
    message.command_id = latest_command_.command_id;
    message.mode = latest_command_.mode;
    message.state = static_cast<std::uint8_t>(state);
    message.active_command = latest_command_;
    message.output_twist.linear.x = output.twist.linear_x;
    message.output_twist.linear.y = output.twist.linear_y;
    message.output_twist.angular.z = output.twist.angular_z;
    message.position_error = output.position_error;
    message.yaw_error = output.yaw_error;
    message.detail = detail;
    status_publisher_->publish(message);
  }

  void publish_stop_once()
  {
    if (!stop_sent_) {
      publish_twist({});
      stop_sent_ = true;
    }
  }

  void update()
  {
    MotionControllerOutput output;
    if (!has_command_) {
      publish_status(output, MotionState::Idle, "waiting for command");
      return;
    }

    const auto steady_now = std::chrono::steady_clock::now();
    if (latest_command_.mode == offset_caster_interfaces::msg::MotionCommand::STOP) {
      controller_->stop();
      publish_stop_once();
      publish_status(output, MotionState::Idle, "stopped");
      return;
    }

    if (age_seconds(steady_now, last_command_time_) > command_timeout_) {
      controller_->stop();
      publish_stop_once();
      publish_status(output, MotionState::Fault, "motion command timed out");
      return;
    }
    if (!has_odometry_ || !has_imu_) {
      publish_stop_once();
      publish_status(output, MotionState::Fault, "waiting for odometry and IMU");
      return;
    }
    if (age_seconds(steady_now, last_odometry_time_) > state_timeout_ ||
      age_seconds(steady_now, last_imu_time_) > state_timeout_)
    {
      controller_->stop();
      publish_stop_once();
      publish_status(output, MotionState::Fault, "odometry or IMU timed out");
      return;
    }
    if (!fault_detail_.empty()) {
      controller_->stop();
      publish_stop_once();
      publish_status(output, MotionState::Fault, fault_detail_);
      return;
    }

    const MotionCommandData command{
      latest_command_.command_id,
      static_cast<MotionMode>(latest_command_.mode),
      latest_command_.x,
      latest_command_.y,
      latest_command_.target_yaw,
      latest_command_.yaw_rate};
    std::string error;
    if (!controller_->set_command(command, robot_state_, error)) {
      publish_stop_once();
      publish_status(output, MotionState::Fault, error);
      return;
    }

    output = controller_->update(robot_state_, update_period_);
    publish_twist(output.twist);
    stop_sent_ = false;
    publish_status(output, output.state, output.detail);
  }

  std::unique_ptr<MotionController> controller_;
  RobotPlanarState robot_state_{};
  offset_caster_interfaces::msg::MotionCommand latest_command_{};
  double update_period_{0.02};
  double command_timeout_{0.5};
  double state_timeout_{0.5};
  SteadyTime last_command_time_{};
  SteadyTime last_odometry_time_{};
  SteadyTime last_imu_time_{};
  bool has_command_{false};
  bool has_odometry_{false};
  bool has_imu_{false};
  bool stop_sent_{false};
  std::string fault_detail_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::Publisher<offset_caster_interfaces::msg::MotionStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<offset_caster_interfaces::msg::MotionCommand>::SharedPtr
  command_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace offset_caster_mujoco_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<offset_caster_mujoco_control::MotionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
