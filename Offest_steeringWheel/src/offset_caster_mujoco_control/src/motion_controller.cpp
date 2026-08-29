#include "offset_caster_mujoco_control/motion_controller.hpp"

#include "offset_caster_mujoco_control/frame_transforms.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace offset_caster_mujoco_control
{

namespace
{

bool finite(const MotionCommandData & command)
{
  return std::isfinite(command.x) && std::isfinite(command.y) &&
         std::isfinite(command.target_yaw) && std::isfinite(command.yaw_rate);
}

bool finite(const RobotPlanarState & state)
{
  return std::isfinite(state.world_x) && std::isfinite(state.world_y) &&
         std::isfinite(state.yaw) && std::isfinite(state.yaw_rate);
}

double clamp_delta(const double target, const double current, const double maximum_delta)
{
  return current + std::clamp(target - current, -maximum_delta, maximum_delta);
}

ChassisTwist clamp_twist(
  const ChassisTwist & twist, const double max_linear_speed,
  const double max_angular_speed)
{
  ChassisTwist result = twist;
  const double magnitude = std::hypot(result.linear_x, result.linear_y);
  if (magnitude > max_linear_speed) {
    const double scale = max_linear_speed / magnitude;
    result.linear_x *= scale;
    result.linear_y *= scale;
  }
  result.angular_z = std::clamp(result.angular_z, -max_angular_speed, max_angular_speed);
  return result;
}

}  // namespace

bool is_valid_motion_mode(const std::uint8_t mode)
{
  return mode <= static_cast<std::uint8_t>(MotionMode::WorldPositionDynamicYaw);
}

MotionController::MotionController(const MotionControllerParameters & parameters)
: parameters_(parameters)
{
  const double values[] = {
    parameters_.max_linear_speed, parameters_.max_angular_speed,
    parameters_.max_linear_acceleration, parameters_.max_angular_acceleration,
    parameters_.position_gain, parameters_.yaw_gain, parameters_.yaw_rate_damping,
    parameters_.position_tolerance, parameters_.yaw_tolerance,
    parameters_.reached_dwell_time};
  for (const double value : values) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("Motion-controller parameters must be finite and non-negative");
    }
  }
  if (parameters_.max_linear_speed == 0.0 || parameters_.max_angular_speed == 0.0 ||
    parameters_.max_linear_acceleration == 0.0 ||
    parameters_.max_angular_acceleration == 0.0)
  {
    throw std::invalid_argument("Motion-controller limits must be positive");
  }
}

bool MotionController::set_command(
  const MotionCommandData & command,
  const RobotPlanarState & robot_state,
  std::string & error)
{
  if (!finite(command) || !finite(robot_state)) {
    error = "Command and robot state must contain finite values";
    return false;
  }
  const auto mode_value = static_cast<std::uint8_t>(command.mode);
  if (!is_valid_motion_mode(mode_value)) {
    error = "Unknown motion mode";
    return false;
  }
  if ((has_active_command_ || target_reached_) && command.command_id == command_.command_id) {
    return true;
  }

  command_ = command;
  command_.target_yaw = normalize_angle(command_.target_yaw);
  reached_duration_ = 0.0;
  target_reached_ = false;
  maximum_progress_ = 0.0;
  trajectory_start_yaw_ = normalize_angle(robot_state.yaw);
  trajectory_yaw_delta_ = normalize_angle(command_.target_yaw - trajectory_start_yaw_);
  initial_position_distance_ = std::hypot(
    command_.x - robot_state.world_x, command_.y - robot_state.world_y);
  has_active_command_ = command_.mode != MotionMode::Stop;
  if (!has_active_command_) {
    previous_twist_ = {};
  }
  error.clear();
  return true;
}

MotionControllerOutput MotionController::update(
  const RobotPlanarState & robot_state, const double dt)
{
  if (!finite(robot_state) || !std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("Robot state and update period must be finite and valid");
  }

  MotionControllerOutput output;
  if (!has_active_command_) {
    output.state = target_reached_ ? MotionState::Reached : MotionState::Idle;
    output.detail = target_reached_ ? "target reached" : "idle";
    return output;
  }

  ChassisTwist desired{};
  double yaw_reference = command_.target_yaw;
  const bool world_velocity =
    command_.mode == MotionMode::WorldVelocityFixedYaw ||
    command_.mode == MotionMode::WorldVelocityDynamicYaw;
  const bool body_velocity =
    command_.mode == MotionMode::BodyVelocityFixedYaw ||
    command_.mode == MotionMode::BodyVelocityDynamicYaw;
  const bool position_mode =
    command_.mode == MotionMode::WorldPositionFixedYaw ||
    command_.mode == MotionMode::WorldPositionDynamicYaw;

  if (world_velocity) {
    const auto body = world_to_body(command_.x, command_.y, robot_state.yaw);
    desired.linear_x = body[0];
    desired.linear_y = body[1];
  } else if (body_velocity) {
    desired.linear_x = command_.x;
    desired.linear_y = command_.y;
  } else if (position_mode) {
    const double error_x = command_.x - robot_state.world_x;
    const double error_y = command_.y - robot_state.world_y;
    output.position_error = std::hypot(error_x, error_y);
    const auto body = world_to_body(
      parameters_.position_gain * error_x,
      parameters_.position_gain * error_y,
      robot_state.yaw);
    desired.linear_x = body[0];
    desired.linear_y = body[1];

    if (command_.mode == MotionMode::WorldPositionDynamicYaw) {
      double progress = 1.0;
      if (initial_position_distance_ > parameters_.position_tolerance) {
        progress = 1.0 - output.position_error / initial_position_distance_;
      }
      maximum_progress_ = std::max(maximum_progress_, std::clamp(progress, 0.0, 1.0));
      yaw_reference = normalize_angle(
        trajectory_start_yaw_ + smoothstep(maximum_progress_) * trajectory_yaw_delta_);
    }
  }

  const bool dynamic_velocity =
    command_.mode == MotionMode::WorldVelocityDynamicYaw ||
    command_.mode == MotionMode::BodyVelocityDynamicYaw;
  if (dynamic_velocity) {
    desired.angular_z = command_.yaw_rate;
    output.yaw_error = 0.0;
  } else {
    output.yaw_error = normalize_angle(yaw_reference - robot_state.yaw);
    desired.angular_z =
      parameters_.yaw_gain * output.yaw_error -
      parameters_.yaw_rate_damping * robot_state.yaw_rate;
  }

  desired = clamp_twist(
    desired, parameters_.max_linear_speed, parameters_.max_angular_speed);

  if (position_mode) {
    const bool inside_tolerance =
      output.position_error <= parameters_.position_tolerance &&
      std::abs(normalize_angle(command_.target_yaw - robot_state.yaw)) <=
      parameters_.yaw_tolerance;
    reached_duration_ = inside_tolerance ? reached_duration_ + dt : 0.0;
    if (reached_duration_ >= parameters_.reached_dwell_time) {
      has_active_command_ = false;
      target_reached_ = true;
      previous_twist_ = {};
      output.twist = {};
      output.state = MotionState::Reached;
      output.detail = "target reached";
      output.yaw_error = normalize_angle(command_.target_yaw - robot_state.yaw);
      return output;
    }
  }

  output.twist = apply_acceleration_limits(desired, dt);
  output.state = MotionState::Active;
  output.detail = "active";
  return output;
}

ChassisTwist MotionController::apply_acceleration_limits(
  const ChassisTwist & desired, const double dt)
{
  ChassisTwist output;
  const double linear_delta = parameters_.max_linear_acceleration * dt;
  output.linear_x = clamp_delta(desired.linear_x, previous_twist_.linear_x, linear_delta);
  output.linear_y = clamp_delta(desired.linear_y, previous_twist_.linear_y, linear_delta);
  output.angular_z = clamp_delta(
    desired.angular_z, previous_twist_.angular_z,
    parameters_.max_angular_acceleration * dt);
  output = clamp_twist(
    output, parameters_.max_linear_speed, parameters_.max_angular_speed);
  previous_twist_ = output;
  return output;
}

void MotionController::stop()
{
  has_active_command_ = false;
  target_reached_ = false;
  previous_twist_ = {};
  command_.mode = MotionMode::Stop;
}

const MotionCommandData & MotionController::command() const
{
  return command_;
}

}  // namespace offset_caster_mujoco_control
