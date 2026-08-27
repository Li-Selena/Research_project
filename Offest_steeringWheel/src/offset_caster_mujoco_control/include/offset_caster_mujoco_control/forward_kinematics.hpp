#ifndef OFFSET_CASTER_MUJOCO_CONTROL__FORWARD_KINEMATICS_HPP_
#define OFFSET_CASTER_MUJOCO_CONTROL__FORWARD_KINEMATICS_HPP_

#include "offset_caster_mujoco_control/kinematics_types.hpp"

#include <array>

namespace offset_caster_mujoco_control
{

struct CasterJointState
{
  double steering_angle;
  double steering_speed;
  double wheel_angle;
  double wheel_speed;
};

struct ForwardKinematicsResult
{
  ChassisTwist twist{};
  double residual_rms{0.0};
};

class ForwardKinematics
{
public:
  explicit ForwardKinematics(
    const std::array<CasterGeometry, kCasterCount> & geometry);

  [[nodiscard]] ForwardKinematicsResult solve(
    const std::array<CasterJointState, kCasterCount> & joint_states) const;

private:
  std::array<CasterGeometry, kCasterCount> geometry_;
};

}  // namespace offset_caster_mujoco_control

#endif  // OFFSET_CASTER_MUJOCO_CONTROL__FORWARD_KINEMATICS_HPP_
