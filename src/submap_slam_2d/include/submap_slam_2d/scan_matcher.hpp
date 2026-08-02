#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "submap_slam_2d/mapping_core.hpp"

namespace submap_slam_2d
{

struct MatcherConfig
{
  std::size_t min_points{60};
  std::size_t point_stride{3};
  double coarse_xy_window_m{0.15};
  double coarse_xy_step_m{0.05};
  double coarse_yaw_window_rad{0.12};
  double coarse_yaw_step_rad{0.03};
  double distance_sigma_m{0.10};
  double min_score{0.30};
  double max_rmse_m{0.25};
  double max_translation_correction_m{0.35};
  double max_yaw_correction_rad{0.25};
  double huber_scale_m{0.10};
  double odom_translation_weight{2.0};
  double odom_yaw_weight{2.0};
  int max_iterations{20};
};

struct MatchResult
{
  bool accepted{false};
  Pose2 pose;
  double score{0.0};
  double rmse_m{0.0};
  double inlier_ratio{0.0};
  double correction_translation_m{0.0};
  double correction_yaw_rad{0.0};
  double duration_ms{0.0};
  std::size_t points_used{0};
  bool covariance_valid{false};
  std::string reason{"not_run"};
};

class DistanceFieldMatcher2D
{
public:
  explicit DistanceFieldMatcher2D(MatcherConfig config = {});
  MatchResult match(
    const DistanceField2D & field, const std::vector<Point2> & base_points,
    const Pose2 & prediction) const;

private:
  struct Score
  {
    double value{0.0};
    double rmse{0.0};
    double inlier_ratio{0.0};
    std::size_t valid{0};
  };

  Score score(const DistanceField2D & field, const std::vector<Point2> & points,
    const Pose2 & pose) const;
  std::vector<Point2> subsample(const std::vector<Point2> & points) const;

  MatcherConfig config_;
};

}  // namespace submap_slam_2d
