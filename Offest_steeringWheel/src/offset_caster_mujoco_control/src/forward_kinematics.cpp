#include "offset_caster_mujoco_control/forward_kinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace offset_caster_mujoco_control
{

namespace
{

constexpr std::size_t kEquationCount = 2 * kCasterCount;
constexpr std::size_t kUnknownCount = 3;
using Matrix = std::array<std::array<double, kUnknownCount>, kEquationCount>;
using Vector = std::array<double, kEquationCount>;

ChassisTwist solve_least_squares_qr(const Matrix & matrix, const Vector & right_hand_side)
{
  std::array<std::array<double, kEquationCount>, kUnknownCount> orthonormal_columns{};
  std::array<std::array<double, kUnknownCount>, kUnknownCount> upper_triangular{};
  std::array<double, kUnknownCount> projected_rhs{};

  for (std::size_t column = 0; column < kUnknownCount; ++column) {
    std::array<double, kEquationCount> work{};
    double original_norm_squared = 0.0;
    for (std::size_t row = 0; row < kEquationCount; ++row) {
      work[row] = matrix[row][column];
      original_norm_squared += work[row] * work[row];
    }

    for (std::size_t previous = 0; previous < column; ++previous) {
      double projection = 0.0;
      for (std::size_t row = 0; row < kEquationCount; ++row) {
        projection += orthonormal_columns[previous][row] * work[row];
      }
      upper_triangular[previous][column] = projection;
      for (std::size_t row = 0; row < kEquationCount; ++row) {
        work[row] -= projection * orthonormal_columns[previous][row];
      }
    }

    double norm_squared = 0.0;
    for (const double value : work) {
      norm_squared += value * value;
    }
    const double norm = std::sqrt(norm_squared);
    const double rank_tolerance =
      100.0 * std::numeric_limits<double>::epsilon() *
      std::max(1.0, std::sqrt(original_norm_squared));
    if (norm <= rank_tolerance) {
      throw std::runtime_error("Forward-kinematics matrix is rank deficient");
    }

    upper_triangular[column][column] = norm;
    for (std::size_t row = 0; row < kEquationCount; ++row) {
      orthonormal_columns[column][row] = work[row] / norm;
      projected_rhs[column] +=
        orthonormal_columns[column][row] * right_hand_side[row];
    }
  }

  std::array<double, kUnknownCount> solution{};
  for (std::size_t reverse = 0; reverse < kUnknownCount; ++reverse) {
    const std::size_t row = kUnknownCount - 1 - reverse;
    double value = projected_rhs[row];
    for (std::size_t column = row + 1; column < kUnknownCount; ++column) {
      value -= upper_triangular[row][column] * solution[column];
    }
    solution[row] = value / upper_triangular[row][row];
  }

  return {solution[0], solution[1], solution[2]};
}

}  // namespace

ForwardKinematics::ForwardKinematics(
  const std::array<CasterGeometry, kCasterCount> & geometry)
: geometry_(geometry)
{
  for (const auto & caster : geometry_) {
    if (!std::isfinite(caster.pivot_x) || !std::isfinite(caster.pivot_y) ||
      !std::isfinite(caster.wheel_radius) || caster.wheel_radius <= 0.0 ||
      !std::isfinite(caster.caster_offset) || caster.caster_offset <= 0.0)
    {
      throw std::invalid_argument("Caster geometry must be finite with positive radii");
    }
  }
}

ForwardKinematicsResult ForwardKinematics::solve(
  const std::array<CasterJointState, kCasterCount> & joint_states) const
{
  Matrix matrix{};
  Vector right_hand_side{};

  for (std::size_t i = 0; i < kCasterCount; ++i) {
    const auto & caster = geometry_[i];
    const auto & state = joint_states[i];
    if (!std::isfinite(state.steering_angle) ||
      !std::isfinite(state.steering_speed) || !std::isfinite(state.wheel_angle) ||
      !std::isfinite(state.wheel_speed))
    {
      throw std::invalid_argument("Caster joint state must be finite");
    }

    const double cosine = std::cos(state.steering_angle);
    const double sine = std::sin(state.steering_angle);
    const double g = caster.pivot_y - caster.caster_offset * sine;
    const double h = caster.pivot_x - caster.caster_offset * cosine;
    const double a =
      caster.wheel_radius * state.wheel_speed * cosine -
      caster.caster_offset * state.steering_speed * sine;
    const double c =
      caster.wheel_radius * state.wheel_speed * sine +
      caster.caster_offset * state.steering_speed * cosine;

    matrix[2 * i] = {1.0, 0.0, -g};
    matrix[2 * i + 1] = {0.0, 1.0, h};
    right_hand_side[2 * i] = a;
    right_hand_side[2 * i + 1] = c;
  }

  ForwardKinematicsResult result;
  result.twist = solve_least_squares_qr(matrix, right_hand_side);

  double residual_squared_sum = 0.0;
  for (std::size_t row = 0; row < kEquationCount; ++row) {
    const double prediction =
      matrix[row][0] * result.twist.linear_x +
      matrix[row][1] * result.twist.linear_y +
      matrix[row][2] * result.twist.angular_z;
    const double residual = prediction - right_hand_side[row];
    residual_squared_sum += residual * residual;
  }
  result.residual_rms = std::sqrt(residual_squared_sum / kEquationCount);

  return result;
}

}  // namespace offset_caster_mujoco_control
