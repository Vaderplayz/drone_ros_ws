#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "submap_slam_2d/se2.hpp"

namespace submap_slam_2d
{

struct GridConfig
{
  double resolution{0.05};
  double size_x_m{20.0};
  double size_y_m{20.0};
  float occupied_log_odds{0.85F};
  float free_log_odds{-0.40F};
  float min_log_odds{-4.0F};
  float max_log_odds{4.0F};
  float occupied_threshold{0.5F};
};

struct DistanceSample
{
  double distance{0.0};
  double gradient_x{0.0};
  double gradient_y{0.0};
  bool valid{false};
};

class OccupancyGrid2D
{
public:
  explicit OccupancyGrid2D(GridConfig config = {});

  void insertScan(const Point2 & origin, const std::vector<Point2> & endpoints);
  bool worldToCell(const Point2 & point, int & x, int & y) const;
  Point2 cellCenter(int x, int y) const;
  std::int8_t occupancy(int x, int y) const;
  bool observed(int x, int y) const;
  bool occupied(int x, int y) const;
  std::vector<Point2> surfacePoints(std::size_t stride = 1) const;

  const GridConfig & config() const {return config_;}
  int width() const {return width_;}
  int height() const {return height_;}
  double originX() const {return origin_x_;}
  double originY() const {return origin_y_;}
  const std::vector<float> & logOdds() const {return log_odds_;}
  const std::vector<std::uint8_t> & observedMask() const {return observed_;}

private:
  int index(int x, int y) const {return y * width_ + x;}
  void updateCell(int x, int y, float increment);
  void traceFree(int x0, int y0, int x1, int y1);

  GridConfig config_;
  int width_{0};
  int height_{0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::vector<float> log_odds_;
  std::vector<std::uint8_t> observed_;
};

class DistanceField2D
{
public:
  void build(const OccupancyGrid2D & grid);
  DistanceSample sample(const Point2 & point) const;
  bool empty() const {return distances_.empty();}
  int width() const {return width_;}
  int height() const {return height_;}

private:
  double valueAt(int x, int y) const;

  int width_{0};
  int height_{0};
  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::vector<float> distances_;
  std::vector<std::uint8_t> valid_;
};

struct Submap2D
{
  int id{0};
  OccupancyGrid2D grid;
  DistanceField2D distance_field;
  Pose2 odom_pose;
  Pose2 optimized_pose;
  std::vector<Pose2> scan_poses;
  std::vector<double> scan_timestamps;
  std::size_t scan_count{0};
  double start_time{0.0};
  double end_time{0.0};
  bool completed{false};

  Submap2D(int submap_id, const GridConfig & config, const Pose2 & pose, double stamp);
  void insert(const Pose2 & local_robot_pose, const std::vector<Point2> & base_points, double stamp);
  void finish();
};

struct RenderedGrid
{
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  int width{0};
  int height{0};
  std::vector<std::int8_t> data;
};

struct RenderSubmapView
{
  const OccupancyGrid2D * grid{nullptr};
  Pose2 pose;
};

RenderedGrid renderGlobalMap(const std::vector<RenderSubmapView> & submaps, double resolution);
RenderedGrid renderGlobalMap(const std::vector<const Submap2D *> & submaps, double resolution);

}  // namespace submap_slam_2d
