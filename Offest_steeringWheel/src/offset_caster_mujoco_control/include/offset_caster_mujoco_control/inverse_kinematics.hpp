#ifndef OFFSET_CASTER_MUJOCO_CONTROL__INVERSE_KINEMATICS_HPP_
#define OFFSET_CASTER_MUJOCO_CONTROL__INVERSE_KINEMATICS_HPP_

#include <array>
#include "offset_caster_mujoco_control/kinematics_types.hpp"

namespace offset_caster_mujoco_control
{

struct JointVelocityCommand
{
  std::array<double, kCasterCount> steering{};
  std::array<double, kCasterCount> wheel{};
  double scale{1.0};
};

class InverseKinematics
{
public:
  InverseKinematics(
    const std::array<CasterGeometry, kCasterCount> & geometry,
    double max_steering_speed,
    double max_wheel_speed);

  [[nodiscard]] JointVelocityCommand solve(
    const ChassisTwist & twist,
    const std::array<double, kCasterCount> & steering_angles) const;

private:
  std::array<CasterGeometry, kCasterCount> geometry_;
  double max_steering_speed_;
  double max_wheel_speed_;
};

}  // namespace offset_caster_mujoco_control

#endif  // OFFSET_CASTER_MUJOCO_CONTROL__INVERSE_KINEMATICS_HPP_
