#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "submap_slam_2d/mapping_core.hpp"
#include "submap_slam_2d/descriptor.hpp"
#include "submap_slam_2d/pose_graph.hpp"
#include "submap_slam_2d/scan_matcher.hpp"

using submap_slam_2d::DistanceField2D;
using submap_slam_2d::GridConfig;
using submap_slam_2d::OccupancyGrid2D;
using submap_slam_2d::Point2;
using submap_slam_2d::Pose2;
using submap_slam_2d::between;
using submap_slam_2d::wrap_angle;
using submap_slam_2d::DistanceFieldMatcher2D;
using submap_slam_2d::MatcherConfig;
using submap_slam_2d::EdgeType;
using submap_slam_2d::GraphEdge;
using submap_slam_2d::PoseGraph2D;
using submap_slam_2d::DescriptorConfig;
using submap_slam_2d::PolarDescriptor;
using submap_slam_2d::hasNonDegenerateGeometry;
using submap_slam_2d::Submap2D;
using submap_slam_2d::renderGlobalMap;

TEST(Se2, CompositionAndInverse)
{
  const Pose2 a{1.0, -2.0, 0.4};
  const Pose2 b{-0.3, 0.7, -0.2};
  const Pose2 identity = a * a.inverse();
  EXPECT_NEAR(identity.x, 0.0, 1e-12);
  EXPECT_NEAR(identity.y, 0.0, 1e-12);
  EXPECT_NEAR(identity.yaw, 0.0, 1e-12);
  const Pose2 recovered = a * between(a, b);
  EXPECT_NEAR(recovered.x, b.x, 1e-12);
  EXPECT_NEAR(recovered.y, b.y, 1e-12);
  EXPECT_NEAR(recovered.yaw, b.yaw, 1e-12);
}

TEST(Se2, AngleWrapping)
{
  EXPECT_NEAR(wrap_angle(3.0 * M_PI), M_PI, 1e-12);
  EXPECT_NEAR(wrap_angle(-3.0 * M_PI), -M_PI, 1e-12);
  EXPECT_NEAR(wrap_angle(0.25), 0.25, 1e-12);
}

TEST(Occupancy, RayTracingMarksFreeAndOccupied)
{
  GridConfig config;
  config.resolution = 0.1;
  config.size_x_m = 4.0;
  config.size_y_m = 4.0;
  OccupancyGrid2D grid(config);
  for (int i = 0; i < 5; ++i) {
    grid.insertScan({0.0, 0.0}, {{1.0, 0.0}});
  }
  int free_x = 0;
  int free_y = 0;
  int hit_x = 0;
  int hit_y = 0;
  ASSERT_TRUE(grid.worldToCell({0.5, 0.0}, free_x, free_y));
  ASSERT_TRUE(grid.worldToCell({1.0, 0.0}, hit_x, hit_y));
  EXPECT_TRUE(grid.observed(free_x, free_y));
  EXPECT_LT(grid.occupancy(free_x, free_y), 50);
  EXPECT_TRUE(grid.occupied(hit_x, hit_y));
}

TEST(DistanceField, BilinearLookupAndGradient)
{
  GridConfig config;
  config.resolution = 0.05;
  config.size_x_m = 4.0;
  config.size_y_m = 4.0;
  OccupancyGrid2D grid(config);
  std::vector<Point2> wall;
  for (int i = -20; i <= 20; ++i) {
    wall.push_back({1.0, 0.05 * i});
  }
  for (int scan = 0; scan < 5; ++scan) {
    grid.insertScan({0.0, 0.0}, wall);
  }
  DistanceField2D field;
  field.build(grid);
  const auto sample = field.sample({0.5, 0.0});
  ASSERT_TRUE(sample.valid);
  EXPECT_NEAR(sample.distance, 0.5, 0.08);
  EXPECT_LT(sample.gradient_x, -0.5);
}

TEST(LocalMatcher, RecoversSyntheticTranslation)
{
  GridConfig config;
  config.resolution = 0.05;
  config.size_x_m = 6.0;
  config.size_y_m = 6.0;
  OccupancyGrid2D grid(config);
  std::vector<Point2> reference;
  for (int i = -30; i <= 30; ++i) {
    reference.push_back({1.5, 0.05 * i});
    reference.push_back({0.05 * i, 1.5});
  }
  for (int scan = 0; scan < 8; ++scan) {
    grid.insertScan({0.0, 0.0}, reference);
  }
  DistanceField2D field;
  field.build(grid);

  std::vector<Point2> shifted;
  shifted.reserve(reference.size());
  for (const Point2 & point : reference) {
    shifted.push_back({point.x - 0.10, point.y + 0.05});
  }
  MatcherConfig matcher_config;
  matcher_config.min_points = 30;
  matcher_config.point_stride = 1;
  matcher_config.coarse_xy_window_m = 0.2;
  matcher_config.max_translation_correction_m = 0.3;
  matcher_config.odom_translation_weight = 0.1;
  matcher_config.odom_yaw_weight = 0.1;
  DistanceFieldMatcher2D matcher(matcher_config);
  const auto result = matcher.match(field, shifted, {0.0, 0.0, 0.0});
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_NEAR(result.pose.x, 0.10, 0.05);
  EXPECT_NEAR(result.pose.y, -0.05, 0.05);
}

TEST(PoseGraph, ClosesSyntheticLoop)
{
  PoseGraph2D graph;
  graph.addVertex({0.0, 0.0, 0.0});
  graph.addVertex({1.05, 0.0, 0.02});
  graph.addVertex({1.05, 1.05, 1.58});
  graph.addVertex({0.05, 1.05, 3.13});
  graph.addVertex({0.12, 0.08, -1.55});
  graph.addEdge({0, 1, {1.0, 0.0, 0.0}, 0.1, 0.1, EdgeType::Odometry});
  graph.addEdge({1, 2, {0.0, 1.0, M_PI_2}, 0.1, 0.1, EdgeType::Odometry});
  graph.addEdge({2, 3, {0.0, 1.0, M_PI_2}, 0.1, 0.1, EdgeType::Odometry});
  graph.addEdge({3, 4, {0.0, 1.0, M_PI_2}, 0.1, 0.1, EdgeType::Odometry});
  graph.addEdge({0, 4, {0.0, 0.0, -M_PI_2}, 0.03, 0.03, EdgeType::Loop});
  const auto summary = graph.optimize();
  ASSERT_TRUE(summary.success) << summary.message;
  EXPECT_LT(std::hypot(graph.poses().back().x, graph.poses().back().y), 0.08);
}

TEST(Descriptor, FindsCircularYawShift)
{
  DescriptorConfig config;
  config.angular_bins = 12;
  config.radial_bins = 3;
  PolarDescriptor descriptor(config);
  std::vector<float> reference(36, 0.0F);
  std::vector<float> query(36, 0.0F);
  reference[1 * 3 + 1] = 1.0F;
  reference[5 * 3 + 2] = 0.5F;
  query[4 * 3 + 1] = 1.0F;
  query[8 * 3 + 2] = 0.5F;
  const auto match = descriptor.compare(reference, query);
  EXPECT_EQ(match.yaw_shift_bins, 3);
  EXPECT_GT(match.similarity, 1.0);
}

TEST(LoopGeometry, RejectsSingleWall)
{
  std::vector<Point2> wall;
  for (int i = 0; i < 200; ++i) {
    wall.push_back({0.02 * i, 0.0});
  }
  EXPECT_FALSE(hasNonDegenerateGeometry(wall, 150));
  for (int i = 0; i < 200; ++i) {
    wall.push_back({0.0, 0.02 * i});
  }
  EXPECT_TRUE(hasNonDegenerateGeometry(wall, 150));
}

TEST(SubmapRegistration, RecoversRotationAndRejectsNoOverlap)
{
  GridConfig grid_config;
  grid_config.resolution = 0.05;
  grid_config.size_x_m = 8.0;
  grid_config.size_y_m = 8.0;
  OccupancyGrid2D grid(grid_config);
  std::vector<Point2> reference;
  for (int i = -35; i <= 35; ++i) {
    reference.push_back({1.5, i * 0.05});
    reference.push_back({i * 0.05, 1.5});
  }
  for (int scan = 0; scan < 8; ++scan) {
    grid.insertScan({0.0, 0.0}, reference);
  }
  DistanceField2D field;
  field.build(grid);
  const Pose2 truth{0.12, -0.08, 0.10};
  const Pose2 inverse = truth.inverse();
  std::vector<Point2> query;
  for (const Point2 & point : reference) {
    query.push_back(inverse.transform(point));
  }
  MatcherConfig config;
  config.min_points = 50;
  config.point_stride = 1;
  config.coarse_xy_window_m = 0.25;
  config.coarse_yaw_window_rad = 0.2;
  config.coarse_yaw_step_rad = 0.025;
  config.max_translation_correction_m = 0.4;
  config.max_yaw_correction_rad = 0.3;
  config.odom_translation_weight = 0.1;
  config.odom_yaw_weight = 0.1;
  DistanceFieldMatcher2D matcher(config);
  const auto result = matcher.match(field, query, {});
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_NEAR(result.pose.x, truth.x, 0.06);
  EXPECT_NEAR(result.pose.y, truth.y, 0.06);
  EXPECT_NEAR(result.pose.yaw, truth.yaw, 0.05);

  std::vector<Point2> outside(100, {20.0, 20.0});
  const auto rejected = matcher.match(field, outside, {});
  EXPECT_FALSE(rejected.accepted);
  EXPECT_TRUE(rejected.reason == "coarse_match_failed" || rejected.reason == "insufficient_overlap");
}

TEST(GlobalRendering, MergesShiftedSubmapsAndPreservesOccupiedEvidence)
{
  GridConfig config;
  config.resolution = 0.1;
  config.size_x_m = 4.0;
  config.size_y_m = 4.0;
  Submap2D first(0, config, {}, 0.0);
  Submap2D second(1, config, {1.0, 0.0, 0.0}, 1.0);
  for (int scan = 0; scan < 5; ++scan) {
    first.grid.insertScan({0.0, 0.0}, {{1.0, 0.0}});
    second.grid.insertScan({0.0, 0.0}, {{1.0, 0.0}});
  }
  second.optimized_pose = {1.0, 0.0, 0.0};
  const std::vector<const Submap2D *> submaps{&first, &second};
  const auto rendered = renderGlobalMap(submaps, 0.1);
  ASSERT_GT(rendered.width, first.grid.width());
  EXPECT_EQ(rendered.data.size(), static_cast<std::size_t>(rendered.width * rendered.height));
  EXPECT_TRUE(std::any_of(rendered.data.begin(), rendered.data.end(),
    [](std::int8_t value) {return value >= 65;}));
}
