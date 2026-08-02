#include <gtest/gtest.h>

#include "vertical_lidar_mapper/spatial_awareness_geometry.hpp"

namespace awareness = vertical_lidar_mapper::spatial_awareness;

TEST(SpatialAwarenessGeometry, PropellerTipsDefineHorizontalCollisionSpan)
{
  awareness::CollisionGeometry geometry;
  geometry.body_length_m = 0.40;
  geometry.body_width_m = 0.30;
  geometry.propeller_tip_to_tip_length_m = 0.80;
  geometry.propeller_tip_to_tip_width_m = 0.70;

  const auto bounds = awareness::physicalBounds(geometry);
  EXPECT_DOUBLE_EQ(bounds.min_x, -0.40);
  EXPECT_DOUBLE_EQ(bounds.max_x, 0.40);
  EXPECT_DOUBLE_EQ(bounds.min_y, -0.35);
  EXPECT_DOUBLE_EQ(bounds.max_y, 0.35);
}

TEST(SpatialAwarenessGeometry, SelfFilterIncludesPadding)
{
  awareness::CollisionGeometry geometry;
  geometry.propeller_tip_to_tip_length_m = 0.80;
  geometry.propeller_tip_to_tip_width_m = 0.70;
  geometry.body_height_m = 0.20;
  geometry.self_filter_padding_m = 0.05;

  EXPECT_TRUE(awareness::isSelfPoint({0.44, 0.0, 0.0}, geometry));
  EXPECT_FALSE(awareness::isSelfPoint({0.46, 0.0, 0.0}, geometry));
}

TEST(SpatialAwarenessGeometry, ClearanceIsMeasuredFromCollisionSurface)
{
  awareness::CollisionGeometry geometry;
  geometry.propeller_tip_to_tip_length_m = 0.80;
  geometry.horizontal_safety_margin_m = 0.50;

  const auto clearance = awareness::directionalClearance(
    {1.00, 0.0, 0.0}, awareness::Direction::Front, geometry);
  ASSERT_TRUE(clearance.has_value());
  EXPECT_NEAR(clearance.value(), 0.60, 1e-9);
}

TEST(SpatialAwarenessGeometry, DirectionalGateRejectsNonOverlappingObstacle)
{
  awareness::CollisionGeometry geometry;
  geometry.propeller_tip_to_tip_width_m = 0.70;
  geometry.horizontal_safety_margin_m = 0.40;

  EXPECT_FALSE(awareness::directionalClearance(
    {0.80, 1.0, 0.0}, awareness::Direction::Front, geometry).has_value());
}

TEST(SpatialAwarenessGeometry, ClassifiesDangerWarningClearAndUnknown)
{
  awareness::CollisionGeometry geometry;
  geometry.horizontal_danger_margin_m = 0.15;
  geometry.horizontal_safety_margin_m = 0.40;

  EXPECT_EQ(
    awareness::classifyClearance(0.10, awareness::Direction::Front, true, geometry),
    awareness::ClearanceState::Danger);
  EXPECT_EQ(
    awareness::classifyClearance(0.25, awareness::Direction::Front, true, geometry),
    awareness::ClearanceState::Warning);
  EXPECT_EQ(
    awareness::classifyClearance(0.60, awareness::Direction::Front, true, geometry),
    awareness::ClearanceState::Clear);
  EXPECT_EQ(
    awareness::classifyClearance(std::nullopt, awareness::Direction::Front, true, geometry),
    awareness::ClearanceState::Clear);
  EXPECT_EQ(
    awareness::classifyClearance(0.05, awareness::Direction::Front, false, geometry),
    awareness::ClearanceState::Unknown);
}

TEST(SpatialAwarenessGeometry, BottomClearanceTracksDynamicDroneHeight)
{
  awareness::CollisionGeometry geometry;
  geometry.center.z = 1.0;
  geometry.body_height_m = 0.20;

  const auto clearance = awareness::directionalClearance(
    {0.0, 0.0, 0.0}, awareness::Direction::Bottom, geometry);
  ASSERT_TRUE(clearance.has_value());
  EXPECT_NEAR(clearance.value(), 0.90, 1e-9);
}
