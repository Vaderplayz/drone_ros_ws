#include "submap_slam_2d/scan_matcher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <ceres/ceres.h>

namespace submap_slam_2d
{
namespace
{

struct DistanceResidual
{
  DistanceResidual(const DistanceField2D * field, Point2 point)
  : field(field), point(point) {}

  bool operator()(const double * const pose, double * residual) const
  {
    const double c = std::cos(pose[2]);
    const double s = std::sin(pose[2]);
    const Point2 transformed{
      pose[0] + c * point.x - s * point.y,
      pose[1] + s * point.x + c * point.y};
    const DistanceSample sample = field->sample(transformed);
    residual[0] = sample.valid ? sample.distance : 1.0;
    return std::isfinite(residual[0]);
  }

  const DistanceField2D * field;
  Point2 point;
};

struct OdomPriorResidual
{
  OdomPriorResidual(Pose2 prediction, double translation_weight, double yaw_weight)
  : prediction(prediction), translation_weight(translation_weight), yaw_weight(yaw_weight) {}

  template<typename T>
  bool operator()(const T * const pose, T * residual) const
  {
    residual[0] = T(translation_weight) * (pose[0] - T(prediction.x));
    residual[1] = T(translation_weight) * (pose[1] - T(prediction.y));
    residual[2] = T(yaw_weight) *
      atan2(sin(pose[2] - T(prediction.yaw)), cos(pose[2] - T(prediction.yaw)));
    return true;
  }

  Pose2 prediction;
  double translation_weight;
  double yaw_weight;
};

}  // namespace

DistanceFieldMatcher2D::DistanceFieldMatcher2D(MatcherConfig config)
: config_(config)
{
  if (config_.min_points < 3U || config_.point_stride == 0U ||
    config_.coarse_xy_step_m <= 0.0 || config_.coarse_yaw_step_rad <= 0.0 ||
    config_.distance_sigma_m <= 0.0 || config_.max_iterations <= 0)
  {
    throw std::invalid_argument("invalid distance-field matcher configuration");
  }
}

std::vector<Point2> DistanceFieldMatcher2D::subsample(const std::vector<Point2> & points) const
{
  std::vector<Point2> sampled;
  sampled.reserve((points.size() + config_.point_stride - 1U) / config_.point_stride);
  for (std::size_t index = 0; index < points.size(); index += config_.point_stride) {
    sampled.push_back(points[index]);
  }
  return sampled;
}

DistanceFieldMatcher2D::Score DistanceFieldMatcher2D::score(
  const DistanceField2D & field, const std::vector<Point2> & points, const Pose2 & pose) const
{
  Score result;
  double squared_sum = 0.0;
  std::size_t inliers = 0U;
  for (const Point2 & point : points) {
    const DistanceSample sample = field.sample(pose.transform(point));
    if (!sample.valid || !std::isfinite(sample.distance)) {
      continue;
    }
    ++result.valid;
    squared_sum += sample.distance * sample.distance;
    result.value += std::exp(
      -0.5 * sample.distance * sample.distance /
      (config_.distance_sigma_m * config_.distance_sigma_m));
    if (sample.distance <= 2.0 * config_.distance_sigma_m) {
      ++inliers;
    }
  }
  if (result.valid > 0U) {
    result.value /= static_cast<double>(result.valid);
    result.rmse = std::sqrt(squared_sum / static_cast<double>(result.valid));
    result.inlier_ratio = static_cast<double>(inliers) / static_cast<double>(result.valid);
  }
  return result;
}

MatchResult DistanceFieldMatcher2D::match(
  const DistanceField2D & field, const std::vector<Point2> & base_points,
  const Pose2 & prediction) const
{
  const auto start = std::chrono::steady_clock::now();
  MatchResult result;
  result.pose = prediction;
  const std::vector<Point2> points = subsample(base_points);
  result.points_used = points.size();
  if (field.empty() || points.size() < config_.min_points) {
    result.reason = "insufficient_geometry";
    return result;
  }

  Pose2 coarse = prediction;
  Score best = score(field, points, coarse);
  for (double dx = -config_.coarse_xy_window_m;
    dx <= config_.coarse_xy_window_m + 1e-9; dx += config_.coarse_xy_step_m)
  {
    for (double dy = -config_.coarse_xy_window_m;
      dy <= config_.coarse_xy_window_m + 1e-9; dy += config_.coarse_xy_step_m)
    {
      for (double dyaw = -config_.coarse_yaw_window_rad;
        dyaw <= config_.coarse_yaw_window_rad + 1e-9; dyaw += config_.coarse_yaw_step_rad)
      {
        const Pose2 candidate{
          prediction.x + dx, prediction.y + dy, wrap_angle(prediction.yaw + dyaw)};
        const Score candidate_score = score(field, points, candidate);
        if (candidate_score.valid >= config_.min_points && candidate_score.value > best.value) {
          coarse = candidate;
          best = candidate_score;
        }
      }
    }
  }
  if (best.valid < config_.min_points || best.value < 0.5 * config_.min_score) {
    result.reason = "coarse_match_failed";
    result.score = best.value;
    result.rmse_m = best.rmse;
    return result;
  }

  double parameters[3] = {coarse.x, coarse.y, coarse.yaw};
  ceres::Problem problem;
  for (const Point2 & point : points) {
    auto * cost = new ceres::NumericDiffCostFunction<DistanceResidual, ceres::CENTRAL, 1, 3>(
      new DistanceResidual(&field, point));
    problem.AddResidualBlock(cost, new ceres::HuberLoss(config_.huber_scale_m), parameters);
  }
  auto * odom_cost = new ceres::AutoDiffCostFunction<OdomPriorResidual, 3, 3>(
    new OdomPriorResidual(
      prediction, config_.odom_translation_weight, config_.odom_yaw_weight));
  problem.AddResidualBlock(odom_cost, nullptr, parameters);

  ceres::Solver::Options options;
  options.max_num_iterations = config_.max_iterations;
  options.num_threads = 1;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  ceres::Covariance::Options covariance_options;
  covariance_options.num_threads = 1;
  ceres::Covariance covariance(covariance_options);
  const std::vector<std::pair<const double *, const double *>> covariance_blocks = {
    {parameters, parameters}};
  if (summary.IsSolutionUsable() && covariance.Compute(covariance_blocks, &problem)) {
    double block[9] = {};
    covariance.GetCovarianceBlock(parameters, parameters, block);
    result.covariance_valid = true;
    for (int index = 0; index < 9; ++index) {
      result.covariance_valid = result.covariance_valid && std::isfinite(block[index]);
    }
    result.covariance_valid = result.covariance_valid && block[0] > 0.0 && block[4] > 0.0 &&
      block[8] > 0.0;
  }
  result.pose = {parameters[0], parameters[1], wrap_angle(parameters[2])};
  const Score refined = score(field, points, result.pose);
  result.score = refined.value;
  result.rmse_m = refined.rmse;
  result.inlier_ratio = refined.inlier_ratio;
  const Pose2 correction = between(prediction, result.pose);
  result.correction_translation_m = std::hypot(correction.x, correction.y);
  result.correction_yaw_rad = std::abs(correction.yaw);
  result.duration_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();

  if (!summary.IsSolutionUsable() || !std::isfinite(
      result.pose.x + result.pose.y + result.pose.yaw + result.score + result.rmse_m))
  {
    result.reason = "optimizer_failed";
  } else if (refined.valid < config_.min_points) {
    result.reason = "insufficient_overlap";
  } else if (result.score < config_.min_score) {
    result.reason = "low_match_score";
  } else if (result.rmse_m > config_.max_rmse_m) {
    result.reason = "high_match_rmse";
  } else if (result.correction_translation_m > config_.max_translation_correction_m) {
    result.reason = "excess_translation_correction";
  } else if (result.correction_yaw_rad > config_.max_yaw_correction_rad) {
    result.reason = "excess_yaw_correction";
  } else {
    result.accepted = true;
    result.reason = "accepted";
  }
  return result;
}

}  // namespace submap_slam_2d
