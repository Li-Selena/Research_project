#include "offset_caster_mujoco_control/inverse_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace offset_caster_mujoco_control
{

namespace
{

bool is_finite(const ChassisTwist & twist)
{
  return std::isfinite(twist.linear_x) && std::isfinite(twist.linear_y) &&
         std::isfinite(twist.angular_z);
}

}  // namespace

InverseKinematics::InverseKinematics(
  const std::array<CasterGeometry, kCasterCount> & geometry,
  const double max_steering_speed,
  const double max_wheel_speed)
: geometry_(geometry),
  max_steering_speed_(max_steering_speed),
  max_wheel_speed_(max_wheel_speed)
{
  if (!std::isfinite(max_steering_speed_) || max_steering_speed_ <= 0.0 ||
    !std::isfinite(max_wheel_speed_) || max_wheel_speed_ <= 0.0)
  {
    throw std::invalid_argument("Joint speed limits must be finite and positive");
  }

  for (const auto & caster : geometry_) {
    if (!std::isfinite(caster.pivot_x) || !std::isfinite(caster.pivot_y) ||
      !std::isfinite(caster.wheel_radius) || caster.wheel_radius <= 0.0 ||
      !std::isfinite(caster.caster_offset) || caster.caster_offset <= 0.0)
    {
      throw std::invalid_argument("Caster geometry must be finite with positive radii");
    }
  }
}

JointVelocityCommand InverseKinematics::solve(
  const ChassisTwist & twist,
  const std::array<double, kCasterCount> & steering_angles) const
{
  if (!is_finite(twist)) {
    throw std::invalid_argument("Chassis twist must be finite");
  }

  JointVelocityCommand command;
  double max_steering_ratio = 0.0;
  double max_wheel_ratio = 0.0;

  for (std::size_t i = 0; i < kCasterCount; ++i) {
    const double delta = steering_angles[i];
    if (!std::isfinite(delta)) {
      throw std::invalid_argument("Steering angles must be finite");
    }

    const auto & caster = geometry_[i];
    const double cosine = std::cos(delta);
    const double sine = std::sin(delta);

    // delta is the MJCF steering-joint angle. At delta = 0 the wheel's
    // positive rolling direction is +x_B, while the caster offset points -x_B.
    command.wheel[i] =
      (twist.linear_x * cosine + twist.linear_y * sine +
      twist.angular_z * (-caster.pivot_y * cosine + caster.pivot_x * sine)) /
      caster.wheel_radius;

    command.steering[i] =
      (-twist.linear_x * sine + twist.linear_y * cosine +
      twist.angular_z *
      (caster.pivot_x * cosine + caster.pivot_y * sine - caster.caster_offset)) /
      caster.caster_offset;

    max_steering_ratio = std::max(
      max_steering_ratio, std::abs(command.steering[i]) / max_steering_speed_);
    max_wheel_ratio = std::max(
      max_wheel_ratio, std::abs(command.wheel[i]) / max_wheel_speed_);
  }

  const double largest_ratio = std::max(max_steering_ratio, max_wheel_ratio);
  command.scale = largest_ratio > 1.0 ? 1.0 / largest_ratio : 1.0;

  for (std::size_t i = 0; i < kCasterCount; ++i) {
    command.steering[i] *= command.scale;
    command.wheel[i] *= command.scale;
  }

  return command;
}

}  // namespace offset_caster_mujoco_control
