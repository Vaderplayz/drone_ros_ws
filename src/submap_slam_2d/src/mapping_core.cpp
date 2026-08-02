#include "submap_slam_2d/mapping_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace submap_slam_2d
{

OccupancyGrid2D::OccupancyGrid2D(GridConfig config)
: config_(config)
{
  if (!(config_.resolution > 0.0) || !(config_.size_x_m > 0.0) || !(config_.size_y_m > 0.0)) {
    throw std::invalid_argument("grid dimensions and resolution must be positive");
  }
  width_ = std::max(1, static_cast<int>(std::ceil(config_.size_x_m / config_.resolution)));
  height_ = std::max(1, static_cast<int>(std::ceil(config_.size_y_m / config_.resolution)));
  origin_x_ = -0.5 * static_cast<double>(width_) * config_.resolution;
  origin_y_ = -0.5 * static_cast<double>(height_) * config_.resolution;
  log_odds_.assign(static_cast<std::size_t>(width_ * height_), 0.0F);
  observed_.assign(log_odds_.size(), 0U);
}

bool OccupancyGrid2D::worldToCell(const Point2 & point, int & x, int & y) const
{
  x = static_cast<int>(std::floor((point.x - origin_x_) / config_.resolution));
  y = static_cast<int>(std::floor((point.y - origin_y_) / config_.resolution));
  return x >= 0 && x < width_ && y >= 0 && y < height_;
}

Point2 OccupancyGrid2D::cellCenter(int x, int y) const
{
  return {origin_x_ + (static_cast<double>(x) + 0.5) * config_.resolution,
    origin_y_ + (static_cast<double>(y) + 0.5) * config_.resolution};
}

void OccupancyGrid2D::updateCell(int x, int y, float increment)
{
  if (x < 0 || x >= width_ || y < 0 || y >= height_) {
    return;
  }
  const int cell = index(x, y);
  observed_[cell] = 1U;
  log_odds_[cell] = std::clamp(
    log_odds_[cell] + increment, config_.min_log_odds, config_.max_log_odds);
}

void OccupancyGrid2D::traceFree(int x0, int y0, int x1, int y1)
{
  int dx = std::abs(x1 - x0);
  int sx = x0 < x1 ? 1 : -1;
  int dy = -std::abs(y1 - y0);
  int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;
  while (x0 != x1 || y0 != y1) {
    updateCell(x0, y0, config_.free_log_odds);
    const int doubled = 2 * error;
    if (doubled >= dy) {
      error += dy;
      x0 += sx;
    }
    if (doubled <= dx) {
      error += dx;
      y0 += sy;
    }
  }
}

void OccupancyGrid2D::insertScan(const Point2 & origin, const std::vector<Point2> & endpoints)
{
  int origin_x = 0;
  int origin_y = 0;
  if (!worldToCell(origin, origin_x, origin_y)) {
    return;
  }
  for (const Point2 & endpoint : endpoints) {
    if (!std::isfinite(endpoint.x) || !std::isfinite(endpoint.y)) {
      continue;
    }
    int end_x = 0;
    int end_y = 0;
    if (!worldToCell(endpoint, end_x, end_y)) {
      continue;
    }
    traceFree(origin_x, origin_y, end_x, end_y);
    updateCell(end_x, end_y, config_.occupied_log_odds);
  }
}

bool OccupancyGrid2D::observed(int x, int y) const
{
  return x >= 0 && x < width_ && y >= 0 && y < height_ && observed_[index(x, y)] != 0U;
}

bool OccupancyGrid2D::occupied(int x, int y) const
{
  return observed(x, y) && log_odds_[index(x, y)] >= config_.occupied_threshold;
}

std::int8_t OccupancyGrid2D::occupancy(int x, int y) const
{
  if (!observed(x, y)) {
    return -1;
  }
  const double probability = 1.0 - 1.0 / (1.0 + std::exp(static_cast<double>(log_odds_[index(x, y)])));
  return static_cast<std::int8_t>(std::clamp(std::lround(100.0 * probability), 0L, 100L));
}

std::vector<Point2> OccupancyGrid2D::surfacePoints(std::size_t stride) const
{
  std::vector<Point2> points;
  stride = std::max<std::size_t>(1, stride);
  std::size_t accepted = 0;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (occupied(x, y) && accepted++ % stride == 0) {
        points.push_back(cellCenter(x, y));
      }
    }
  }
  return points;
}

void DistanceField2D::build(const OccupancyGrid2D & grid)
{
  width_ = grid.width();
  height_ = grid.height();
  resolution_ = grid.config().resolution;
  origin_x_ = grid.originX();
  origin_y_ = grid.originY();
  cv::Mat binary(height_, width_, CV_8UC1, cv::Scalar(255));
  valid_.assign(static_cast<std::size_t>(width_ * height_), 0U);
  bool have_occupied = false;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const int cell = y * width_ + x;
      valid_[cell] = grid.observed(x, y) ? 1U : 0U;
      if (grid.occupied(x, y)) {
        binary.at<std::uint8_t>(y, x) = 0U;
        have_occupied = true;
      }
    }
  }
  distances_.assign(static_cast<std::size_t>(width_ * height_),
    std::numeric_limits<float>::infinity());
  if (!have_occupied) {
    return;
  }
  cv::Mat distance_pixels;
  cv::distanceTransform(binary, distance_pixels, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      distances_[static_cast<std::size_t>(y * width_ + x)] =
        distance_pixels.at<float>(y, x) * static_cast<float>(resolution_);
    }
  }
}

double DistanceField2D::valueAt(int x, int y) const
{
  if (x < 0 || x >= width_ || y < 0 || y >= height_) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(distances_[static_cast<std::size_t>(y * width_ + x)]);
}

DistanceSample DistanceField2D::sample(const Point2 & point) const
{
  DistanceSample result;
  if (distances_.empty()) {
    return result;
  }
  const double gx = (point.x - origin_x_) / resolution_ - 0.5;
  const double gy = (point.y - origin_y_) / resolution_ - 0.5;
  const int x0 = static_cast<int>(std::floor(gx));
  const int y0 = static_cast<int>(std::floor(gy));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;
  if (x0 < 0 || y0 < 0 || x1 >= width_ || y1 >= height_) {
    return result;
  }
  const int indices[4] = {y0 * width_ + x0, y0 * width_ + x1,
    y1 * width_ + x0, y1 * width_ + x1};
  for (const int cell : indices) {
    if (valid_[static_cast<std::size_t>(cell)] == 0U) {
      return result;
    }
  }
  const double dx = gx - static_cast<double>(x0);
  const double dy = gy - static_cast<double>(y0);
  const double d00 = valueAt(x0, y0);
  const double d10 = valueAt(x1, y0);
  const double d01 = valueAt(x0, y1);
  const double d11 = valueAt(x1, y1);
  if (!std::isfinite(d00 + d10 + d01 + d11)) {
    return result;
  }
  result.distance = (1.0 - dy) * ((1.0 - dx) * d00 + dx * d10) +
    dy * ((1.0 - dx) * d01 + dx * d11);
  result.gradient_x = ((1.0 - dy) * (d10 - d00) + dy * (d11 - d01)) / resolution_;
  result.gradient_y = ((1.0 - dx) * (d01 - d00) + dx * (d11 - d10)) / resolution_;
  result.valid = std::isfinite(result.distance) && std::isfinite(result.gradient_x) &&
    std::isfinite(result.gradient_y);
  return result;
}

Submap2D::Submap2D(int submap_id, const GridConfig & config, const Pose2 & pose, double stamp)
: id(submap_id), grid(config), odom_pose(pose), optimized_pose(pose), start_time(stamp), end_time(stamp)
{
}

void Submap2D::insert(
  const Pose2 & local_robot_pose, const std::vector<Point2> & base_points, double stamp)
{
  std::vector<Point2> local_points;
  local_points.reserve(base_points.size());
  for (const Point2 & point : base_points) {
    local_points.push_back(local_robot_pose.transform(point));
  }
  grid.insertScan({local_robot_pose.x, local_robot_pose.y}, local_points);
  scan_poses.push_back(local_robot_pose);
  scan_timestamps.push_back(stamp);
  ++scan_count;
  end_time = stamp;
}

void Submap2D::finish()
{
  distance_field.build(grid);
  completed = true;
}

RenderedGrid renderGlobalMap(const std::vector<RenderSubmapView> & submaps, double resolution)
{
  if (!(resolution > 0.0)) {
    throw std::invalid_argument("global map resolution must be positive");
  }
  RenderedGrid result;
  result.resolution = resolution;
  if (submaps.empty()) {
    return result;
  }
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const RenderSubmapView & submap : submaps) {
    if (submap.grid == nullptr) {
      continue;
    }
    const auto & grid = *submap.grid;
    const Point2 corners[4] = {
      {grid.originX(), grid.originY()},
      {grid.originX() + grid.width() * grid.config().resolution, grid.originY()},
      {grid.originX(), grid.originY() + grid.height() * grid.config().resolution},
      {grid.originX() + grid.width() * grid.config().resolution,
        grid.originY() + grid.height() * grid.config().resolution}};
    for (const Point2 & corner : corners) {
      const Point2 global = submap.pose.transform(corner);
      min_x = std::min(min_x, global.x);
      min_y = std::min(min_y, global.y);
      max_x = std::max(max_x, global.x);
      max_y = std::max(max_y, global.y);
    }
  }
  result.origin_x = min_x;
  result.origin_y = min_y;
  result.width = std::max(1, static_cast<int>(std::ceil((max_x - min_x) / resolution)));
  result.height = std::max(1, static_cast<int>(std::ceil((max_y - min_y) / resolution)));
  std::vector<float> evidence(static_cast<std::size_t>(result.width * result.height), 0.0F);
  std::vector<std::uint16_t> observations(evidence.size(), 0U);
  for (const RenderSubmapView & submap : submaps) {
    if (submap.grid == nullptr) {
      continue;
    }
    const auto & grid = *submap.grid;
    for (int y = 0; y < grid.height(); ++y) {
      for (int x = 0; x < grid.width(); ++x) {
        if (!grid.observed(x, y)) {
          continue;
        }
        const Point2 global = submap.pose.transform(grid.cellCenter(x, y));
        const int gx = static_cast<int>(std::floor((global.x - min_x) / resolution));
        const int gy = static_cast<int>(std::floor((global.y - min_y) / resolution));
        if (gx < 0 || gx >= result.width || gy < 0 || gy >= result.height) {
          continue;
        }
        const std::size_t target = static_cast<std::size_t>(gy * result.width + gx);
        const std::size_t source = static_cast<std::size_t>(y * grid.width() + x);
        const float source_evidence = grid.logOdds()[source];
        if (source_evidence > 0.0F) {
          evidence[target] = std::max(evidence[target], source_evidence);
        } else if (evidence[target] <= 0.0F) {
          evidence[target] = std::max(-4.0F, evidence[target] + 0.25F * source_evidence);
        }
        observations[target] = static_cast<std::uint16_t>(
          std::min<unsigned int>(65535U, observations[target] + 1U));
      }
    }
  }
  result.data.assign(evidence.size(), -1);
  for (std::size_t index = 0; index < evidence.size(); ++index) {
    if (observations[index] == 0U) {
      continue;
    }
    const double probability = 1.0 - 1.0 / (1.0 + std::exp(static_cast<double>(evidence[index])));
    result.data[index] = static_cast<std::int8_t>(
      std::clamp(std::lround(100.0 * probability), 0L, 100L));
  }
  return result;
}

RenderedGrid renderGlobalMap(const std::vector<const Submap2D *> & submaps, double resolution)
{
  std::vector<RenderSubmapView> views;
  views.reserve(submaps.size());
  for (const Submap2D * submap : submaps) {
    if (submap != nullptr) {
      views.push_back({&submap->grid, submap->optimized_pose});
    }
  }
  return renderGlobalMap(views, resolution);
}

}  // namespace submap_slam_2d
