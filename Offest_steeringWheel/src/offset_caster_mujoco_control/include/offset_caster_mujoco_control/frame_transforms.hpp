#ifndef OFFSET_CASTER_MUJOCO_CONTROL__FRAME_TRANSFORMS_HPP_
#define OFFSET_CASTER_MUJOCO_CONTROL__FRAME_TRANSFORMS_HPP_

#include <array>

namespace offset_caster_mujoco_control
{

[[nodiscard]] double normalize_angle(double angle);

[[nodiscard]] double quaternion_to_yaw(double w, double x, double y, double z);

[[nodiscard]] std::array<double, 2> world_to_body(
  double world_x, double world_y, double yaw);

[[nodiscard]] std::array<double, 2> body_to_world(
  double body_x, double body_y, double yaw);

[[nodiscard]] double smoothstep(double progress);

}  // namespace offset_caster_mujoco_control

#endif  // OFFSET_CASTER_MUJOCO_CONTROL__FRAME_TRANSFORMS_HPP_
