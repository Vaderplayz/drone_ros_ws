#pragma once

#include <cstddef>
#include <vector>

#include "submap_slam_2d/mapping_core.hpp"

namespace submap_slam_2d
{

struct DescriptorConfig
{
  int angular_bins{60};
  int radial_bins{20};
  double max_radius_m{8.0};
};

struct DescriptorMatch
{
  double similarity{0.0};
  int yaw_shift_bins{0};
  double yaw_shift_rad{0.0};
};

class PolarDescriptor
{
public:
  explicit PolarDescriptor(DescriptorConfig config = {});
  std::vector<float> compute(const OccupancyGrid2D & grid) const;
  DescriptorMatch compare(const std::vector<float> & reference,
    const std::vector<float> & query) const;

private:
  DescriptorConfig config_;
};

bool hasNonDegenerateGeometry(
  const std::vector<Point2> & points, std::size_t min_points, double min_eigen_ratio = 0.04);

}  // namespace submap_slam_2d
