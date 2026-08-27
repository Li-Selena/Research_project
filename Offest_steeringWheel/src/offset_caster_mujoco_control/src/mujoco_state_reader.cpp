#include "offset_caster_mujoco_control/mujoco_state_reader.hpp"

#include <stdexcept>
#include <string>

namespace offset_caster_mujoco_control
{

namespace
{

int require_hinge_joint(const mjModel * model, const std::string & name)
{
  const int joint_id = mj_name2id(model, mjOBJ_JOINT, name.c_str());
  if (joint_id < 0) {
    throw std::runtime_error("MuJoCo joint not found: " + name);
  }
  if (model->jnt_type[joint_id] != mjJNT_HINGE) {
    throw std::runtime_error("MuJoCo joint is not a hinge: " + name);
  }
  return joint_id;
}

}  // namespace

MujocoStateReader::MujocoStateReader(
  const mjModel * model,
  const std::array<std::string, kCasterCount> & steering_joint_names,
  const std::array<std::string, kCasterCount> & wheel_joint_names)
: model_(model)
{
  if (model_ == nullptr) {
    throw std::invalid_argument("MuJoCo model must not be null");
  }

  for (std::size_t i = 0; i < kCasterCount; ++i) {
    const int steering_joint_id = require_hinge_joint(model_, steering_joint_names[i]);
    const int wheel_joint_id = require_hinge_joint(model_, wheel_joint_names[i]);
    steering_qpos_addresses_[i] = model_->jnt_qposadr[steering_joint_id];
    steering_dof_addresses_[i] = model_->jnt_dofadr[steering_joint_id];
    wheel_qpos_addresses_[i] = model_->jnt_qposadr[wheel_joint_id];
    wheel_dof_addresses_[i] = model_->jnt_dofadr[wheel_joint_id];
  }
}

std::array<CasterJointState, kCasterCount> MujocoStateReader::read(
  const mjData * data) const
{
  if (data == nullptr) {
    throw std::invalid_argument("MuJoCo data must not be null");
  }

  std::array<CasterJointState, kCasterCount> states{};
  for (std::size_t i = 0; i < kCasterCount; ++i) {
    states[i] = {
      data->qpos[steering_qpos_addresses_[i]],
      data->qvel[steering_dof_addresses_[i]],
      data->qpos[wheel_qpos_addresses_[i]],
      data->qvel[wheel_dof_addresses_[i]]};
  }
  return states;
}

}  // namespace offset_caster_mujoco_control
