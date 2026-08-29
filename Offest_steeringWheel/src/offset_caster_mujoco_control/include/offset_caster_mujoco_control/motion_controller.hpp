#ifndef OFFSET_CASTER_MUJOCO_CONTROL__MOTION_CONTROLLER_HPP_
#define OFFSET_CASTER_MUJOCO_CONTROL__MOTION_CONTROLLER_HPP_

#include "offset_caster_mujoco_control/kinematics_types.hpp"

#include <cstdint>
#include <string>

namespace offset_caster_mujoco_control
{

enum class MotionMode : std::uint8_t
{
  Stop = 0,
  WorldVelocityFixedYaw = 1,
  WorldVelocityDynamicYaw = 2,
  BodyVelocityFixedYaw = 3,
  BodyVelocityDynamicYaw = 4,
  WorldPositionFixedYaw = 5,
  WorldPositionDynamicYaw = 6
};

enum class MotionState : std::uint8_t
{
  Idle = 0,
  Active = 1,
  Reached = 2,
  Fault = 3
};

struct MotionCommandData
{
  std::uint32_t command_id{0};
  MotionMode mode{MotionMode::Stop};
  double x{0.0};
  double y{0.0};
  double target_yaw{0.0};
  double yaw_rate{0.0};
};

struct RobotPlanarState
{
  double world_x{0.0};
  double world_y{0.0};
  double yaw{0.0};
  double yaw_rate{0.0};
};

struct MotionControllerParameters
{
  double max_linear_speed{0.5};
  double max_angular_speed{0.8};
  double max_linear_acceleration{1.0};
  double max_angular_acceleration{2.0};
  double position_gain{1.0};
  double yaw_gain{2.0};
  double yaw_rate_damping{0.15};
  double position_tolerance{0.03};
  double yaw_tolerance{0.03};
  double reached_dwell_time{0.2};
};

struct MotionControllerOutput
{
  ChassisTwist twist{};
  MotionState state{MotionState::Idle};
  double position_error{0.0};
  double yaw_error{0.0};
  std::string detail{"idle"};
};

class MotionController
{
public:
  explicit MotionController(const MotionControllerParameters & parameters);

  [[nodiscard]] bool set_command(
    const MotionCommandData & command,
    const RobotPlanarState & robot_state,
    std::string & error);

  [[nodiscard]] MotionControllerOutput update(
    const RobotPlanarState & robot_state, double dt);

  void stop();

  [[nodiscard]] const MotionCommandData & command() const;

private:
  [[nodiscard]] ChassisTwist apply_acceleration_limits(
    const ChassisTwist & desired, double dt);

  MotionControllerParameters parameters_;
  MotionCommandData command_{};
  ChassisTwist previous_twist_{};
  double trajectory_start_yaw_{0.0};
  double trajectory_yaw_delta_{0.0};
  double initial_position_distance_{0.0};
  double maximum_progress_{0.0};
  double reached_duration_{0.0};
  bool has_active_command_{false};
  bool target_reached_{false};
};

[[nodiscard]] bool is_valid_motion_mode(std::uint8_t mode);

}  // namespace offset_caster_mujoco_control

#endif  // OFFSET_CASTER_MUJOCO_CONTROL__MOTION_CONTROLLER_HPP_
