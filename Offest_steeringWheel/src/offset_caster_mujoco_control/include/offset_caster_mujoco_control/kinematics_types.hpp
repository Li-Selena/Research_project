#ifndef OFFSET_CASTER_MUJOCO_CONTROL__KINEMATICS_TYPES_HPP_
#define OFFSET_CASTER_MUJOCO_CONTROL__KINEMATICS_TYPES_HPP_

#include <cstddef>

namespace offset_caster_mujoco_control
{

constexpr std::size_t kCasterCount = 4;

struct CasterGeometry
{
  double pivot_x;
  double pivot_y;
  double wheel_radius;
  double caster_offset;
};

struct ChassisTwist
{
  double linear_x;
  double linear_y;
  double angular_z;
};

}  // namespace offset_caster_mujoco_control

#endif  // OFFSET_CASTER_MUJOCO_CONTROL__KINEMATICS_TYPES_HPP_
