#include "offset_caster_mujoco_control/frame_transforms.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace offset_caster_mujoco_control
{

double normalize_angle(const double angle)
{
  if (!std::isfinite(angle)) {
    throw std::invalid_argument("Angle must be finite");
  }
  return std::atan2(std::sin(angle), std::cos(angle));
}

double quaternion_to_yaw(
  const double w, const double x, const double y, const double z)
{
  if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    throw std::invalid_argument("Quaternion must be finite");
  }
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

std::array<double, 2> world_to_body(
  const double world_x, const double world_y, const double yaw)
{
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  return {
    cosine * world_x + sine * world_y,
    -sine * world_x + cosine * world_y};
}

std::array<double, 2> body_to_world(
  const double body_x, const double body_y, const double yaw)
{
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  return {
    cosine * body_x - sine * body_y,
    sine * body_x + cosine * body_y};
}

double smoothstep(const double progress)
{
  if (!std::isfinite(progress)) {
    throw std::invalid_argument("Progress must be finite");
  }
  const double clamped = std::clamp(progress, 0.0, 1.0);
  return clamped * clamped * (3.0 - 2.0 * clamped);
}

}  // namespace offset_caster_mujoco_control
