#include "submap_slam_2d/descriptor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace submap_slam_2d
{

PolarDescriptor::PolarDescriptor(DescriptorConfig config)
: config_(config)
{
  if (config_.angular_bins < 4 || config_.radial_bins < 2 || config_.max_radius_m <= 0.0) {
    throw std::invalid_argument("invalid polar descriptor configuration");
  }
}

std::vector<float> PolarDescriptor::compute(const OccupancyGrid2D & grid) const
{
  std::vector<float> descriptor(
    static_cast<std::size_t>(config_.angular_bins * config_.radial_bins), 0.0F);
  for (const Point2 & point : grid.surfacePoints()) {
    const double radius = std::hypot(point.x, point.y);
    if (radius >= config_.max_radius_m) {
      continue;
    }
    double angle = std::atan2(point.y, point.x);
    if (angle < 0.0) {
      angle += 2.0 * M_PI;
    }
    const int angular = std::min(
      config_.angular_bins - 1,
      static_cast<int>(std::floor(angle * config_.angular_bins / (2.0 * M_PI))));
    const int radial = std::min(
      config_.radial_bins - 1,
      static_cast<int>(std::floor(radius * config_.radial_bins / config_.max_radius_m)));
    descriptor[static_cast<std::size_t>(angular * config_.radial_bins + radial)] += 1.0F;
  }
  double norm = 0.0;
  for (float value : descriptor) {
    norm += static_cast<double>(value) * value;
  }
  norm = std::sqrt(norm);
  if (norm > 0.0) {
    for (float & value : descriptor) {
      value = static_cast<float>(value / norm);
    }
  }
  return descriptor;
}

DescriptorMatch PolarDescriptor::compare(
  const std::vector<float> & reference, const std::vector<float> & query) const
{
  DescriptorMatch best;
  const std::size_t expected = static_cast<std::size_t>(
    config_.angular_bins * config_.radial_bins);
  if (reference.size() != expected || query.size() != expected) {
    return best;
  }
  for (int shift = 0; shift < config_.angular_bins; ++shift) {
    double dot = 0.0;
    for (int angular = 0; angular < config_.angular_bins; ++angular) {
      const int shifted = (angular + shift) % config_.angular_bins;
      for (int radial = 0; radial < config_.radial_bins; ++radial) {
        dot += static_cast<double>(reference[static_cast<std::size_t>(
          angular * config_.radial_bins + radial)]) *
          query[static_cast<std::size_t>(shifted * config_.radial_bins + radial)];
      }
    }
    if (dot > best.similarity) {
      best.similarity = dot;
      best.yaw_shift_bins = shift;
    }
  }
  int signed_shift = best.yaw_shift_bins;
  if (signed_shift > config_.angular_bins / 2) {
    signed_shift -= config_.angular_bins;
  }
  best.yaw_shift_rad = 2.0 * M_PI * static_cast<double>(signed_shift) /
    static_cast<double>(config_.angular_bins);
  return best;
}

bool hasNonDegenerateGeometry(
  const std::vector<Point2> & points, std::size_t min_points, double min_eigen_ratio)
{
  if (points.size() < min_points) {
    return false;
  }
  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const Point2 & point : points) {
    mean_x += point.x;
    mean_y += point.y;
  }
  mean_x /= static_cast<double>(points.size());
  mean_y /= static_cast<double>(points.size());
  double xx = 0.0;
  double xy = 0.0;
  double yy = 0.0;
  for (const Point2 & point : points) {
    const double dx = point.x - mean_x;
    const double dy = point.y - mean_y;
    xx += dx * dx;
    xy += dx * dy;
    yy += dy * dy;
  }
  const double trace = xx + yy;
  const double discriminant = std::sqrt(std::max(0.0, (xx - yy) * (xx - yy) + 4.0 * xy * xy));
  const double largest = 0.5 * (trace + discriminant);
  const double smallest = 0.5 * (trace - discriminant);
  return largest > 1e-9 && smallest / largest >= min_eigen_ratio;
}

}  // namespace submap_slam_2d
