#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace obs_avoid::spatial_guard
{

enum class DirectionState
{
  Clear,
  Warning,
  Danger,
  Unknown
};

enum class Direction : std::size_t
{
  Front = 0,
  Rear = 1,
  Left = 2,
  Right = 3,
  Top = 4,
  Bottom = 5
};

struct BodyCommand
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw_rate{0.0};
};

struct GuardResult
{
  BodyCommand command;
  std::array<bool, 6> blocked{};
  std::array<bool, 6> warning_limited{};
};

inline std::size_t index(Direction direction)
{
  return static_cast<std::size_t>(direction);
}

inline bool blocksMotion(DirectionState state, bool unknown_blocks_motion)
{
  return state == DirectionState::Danger ||
         (unknown_blocks_motion && state == DirectionState::Unknown);
}

inline void gateAxis(
  double & value,
  Direction positive_direction,
  Direction negative_direction,
  const std::array<DirectionState, 6> & states,
  double warning_scale,
  bool unknown_blocks_motion,
  std::array<bool, 6> & blocked,
  std::array<bool, 6> & warning_limited)
{
  if (std::abs(value) <= 1e-9) {
    value = 0.0;
    return;
  }

  const Direction direction = value > 0.0 ? positive_direction : negative_direction;
  const auto direction_index = index(direction);
  const auto state = states[direction_index];
  if (blocksMotion(state, unknown_blocks_motion)) {
    value = 0.0;
    blocked[direction_index] = true;
  } else if (state == DirectionState::Warning) {
    value *= std::clamp(warning_scale, 0.0, 1.0);
    warning_limited[direction_index] = true;
  }
}

inline GuardResult applyDirectionalGuard(
  const BodyCommand & input,
  const std::array<DirectionState, 6> & states,
  double warning_scale,
  bool unknown_blocks_motion)
{
  GuardResult result;
  result.command = input;
  gateAxis(
    result.command.x, Direction::Front, Direction::Rear, states,
    warning_scale, unknown_blocks_motion, result.blocked, result.warning_limited);
  gateAxis(
    result.command.y, Direction::Left, Direction::Right, states,
    warning_scale, unknown_blocks_motion, result.blocked, result.warning_limited);
  gateAxis(
    result.command.z, Direction::Top, Direction::Bottom, states,
    warning_scale, unknown_blocks_motion, result.blocked, result.warning_limited);
  return result;
}

}  // namespace obs_avoid::spatial_guard
