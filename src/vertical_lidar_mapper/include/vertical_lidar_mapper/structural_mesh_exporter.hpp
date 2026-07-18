#ifndef VERTICAL_LIDAR_MAPPER__STRUCTURAL_MESH_EXPORTER_HPP_
#define VERTICAL_LIDAR_MAPPER__STRUCTURAL_MESH_EXPORTER_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace vertical_lidar_mapper
{

struct StructuralMeshConfig
{
  int occupied_threshold{65};
  int free_threshold{19};
  bool auto_height{true};
  bool include_ceiling{false};
  bool use_obstacle_heights{false};
  double default_floor_z{0.0};
  double default_ceiling_z{2.5};
  double floor_quantile{0.02};
  double ceiling_quantile{0.98};
  double min_room_height{1.8};
  double max_room_height{4.0};
  double min_obstacle_height{0.25};
  double obstacle_height_padding{0.05};
  double height_quantization{0.10};
  int height_search_radius_cells{1};
  std::size_t max_grid_cells{2000000U};
  std::size_t max_height_samples{200000U};
  std::size_t max_quads{250000U};
};

struct StructuralMeshStats
{
  std::size_t floor_rectangles{0};
  std::size_t wall_runs{0};
  std::size_t vertices{0};
  std::size_t triangles{0};
  std::size_t binary_bytes{0};
  double floor_z{0.0};
  double ceiling_z{0.0};
  double duration_ms{0.0};
};

bool exportStructuralMeshGlb(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const nav_msgs::msg::OccupancyGrid & map,
  const StructuralMeshConfig & config,
  const std::filesystem::path & output_file,
  const std::string & source_frame,
  StructuralMeshStats & stats,
  std::string & error_message);

}  // namespace vertical_lidar_mapper

#endif  // VERTICAL_LIDAR_MAPPER__STRUCTURAL_MESH_EXPORTER_HPP_
