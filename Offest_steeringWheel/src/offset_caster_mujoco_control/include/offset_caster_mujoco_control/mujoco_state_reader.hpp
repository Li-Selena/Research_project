#ifndef OFFSET_CASTER_MUJOCO_CONTROL__MUJOCO_STATE_READER_HPP_
#define OFFSET_CASTER_MUJOCO_CONTROL__MUJOCO_STATE_READER_HPP_

#include "offset_caster_mujoco_control/forward_kinematics.hpp"

#include <mujoco/mujoco.h>

#include <array>
#include <string>

namespace offset_caster_mujoco_control
{

class MujocoStateReader
{
public:
  MujocoStateReader(
    const mjModel * model,
    const std::array<std::string, kCasterCount> & steering_joint_names,
    const std::array<std::string, kCasterCount> & wheel_joint_names);

  [[nodiscard]] std::array<CasterJointState, kCasterCount> read(
    const mjData * data) const;

private:
  const mjModel * model_;
  std::array<int, kCasterCount> steering_qpos_addresses_{};
  std::array<int, kCasterCount> steering_dof_addresses_{};
  std::array<int, kCasterCount> wheel_qpos_addresses_{};
  std::array<int, kCasterCount> wheel_dof_addresses_{};
};

}  // namespace offset_caster_mujoco_control

#endif  // OFFSET_CASTER_MUJOCO_CONTROL__MUJOCO_STATE_READER_HPP_
