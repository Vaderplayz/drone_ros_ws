#include <array>

#include "gtest/gtest.h"

#include "obs_avoid/spatial_command_guard.hpp"

namespace guard = obs_avoid::spatial_guard;

TEST(SpatialCommandGuard, ClearDirectionsPreserveCommand)
{
  std::array<guard::DirectionState, 6> states;
  states.fill(guard::DirectionState::Clear);
  const auto result = guard::applyDirectionalGuard({0.2, -0.1, 0.15, 0.2}, states, 0.25, true);
  EXPECT_DOUBLE_EQ(result.command.x, 0.2);
  EXPECT_DOUBLE_EQ(result.command.y, -0.1);
  EXPECT_DOUBLE_EQ(result.command.z, 0.15);
  EXPECT_DOUBLE_EQ(result.command.yaw_rate, 0.2);
}

TEST(SpatialCommandGuard, DangerBlocksOnlyMotionTowardObstacle)
{
  std::array<guard::DirectionState, 6> states;
  states.fill(guard::DirectionState::Clear);
  states[guard::index(guard::Direction::Front)] = guard::DirectionState::Danger;
  states[guard::index(guard::Direction::Right)] = guard::DirectionState::Danger;
  states[guard::index(guard::Direction::Bottom)] = guard::DirectionState::Danger;
  const auto result = guard::applyDirectionalGuard({0.2, -0.1, -0.15, 0.2}, states, 0.25, true);
  EXPECT_DOUBLE_EQ(result.command.x, 0.0);
  EXPECT_DOUBLE_EQ(result.command.y, 0.0);
  EXPECT_DOUBLE_EQ(result.command.z, 0.0);
  EXPECT_DOUBLE_EQ(result.command.yaw_rate, 0.2);
}

TEST(SpatialCommandGuard, WarningScalesAffectedAxis)
{
  std::array<guard::DirectionState, 6> states;
  states.fill(guard::DirectionState::Clear);
  states[guard::index(guard::Direction::Rear)] = guard::DirectionState::Warning;
  states[guard::index(guard::Direction::Top)] = guard::DirectionState::Warning;
  const auto result = guard::applyDirectionalGuard({-0.2, 0.1, 0.16, 0.0}, states, 0.25, true);
  EXPECT_DOUBLE_EQ(result.command.x, -0.05);
  EXPECT_DOUBLE_EQ(result.command.y, 0.1);
  EXPECT_DOUBLE_EQ(result.command.z, 0.04);
}

TEST(SpatialCommandGuard, UnknownIsFailClosedByDefault)
{
  std::array<guard::DirectionState, 6> states;
  states.fill(guard::DirectionState::Unknown);
  const auto result = guard::applyDirectionalGuard({0.2, 0.1, 0.15, 0.1}, states, 0.25, true);
  EXPECT_DOUBLE_EQ(result.command.x, 0.0);
  EXPECT_DOUBLE_EQ(result.command.y, 0.0);
  EXPECT_DOUBLE_EQ(result.command.z, 0.0);
  EXPECT_DOUBLE_EQ(result.command.yaw_rate, 0.1);
}
