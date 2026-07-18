#include "vertical_lidar_mapper/structural_mesh_exporter.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace vertical_lidar_mapper
{

namespace
{

constexpr std::uint32_t kGlbMagic = 0x46546C67U;
constexpr std::uint32_t kGlbVersion = 2U;
constexpr std::uint32_t kJsonChunkType = 0x4E4F534AU;
constexpr std::uint32_t kBinChunkType = 0x004E4942U;

struct Vec3
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

static_assert(sizeof(Vec3) == 3U * sizeof(float), "Vec3 must be tightly packed for glTF");

struct MeshPrimitive
{
  std::string name;
  int material{0};
  std::vector<Vec3> positions;
  std::vector<Vec3> normals;
  std::vector<std::uint32_t> indices;
};

struct BufferView
{
  std::size_t offset{0};
  std::size_t length{0};
  int target{0};
};

struct Accessor
{
  std::size_t buffer_view{0};
  int component_type{0};
  std::size_t count{0};
  std::string type;
  bool has_bounds{false};
  Vec3 minimum;
  Vec3 maximum;
};

struct PrimitiveAccessors
{
  std::size_t position{0};
  std::size_t normal{0};
  std::size_t index{0};
  int material{0};
};

std::string jsonEscape(const std::string & value)
{
  std::ostringstream out;
  for (const unsigned char ch : value) {
    switch (ch) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (ch < 0x20U) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(ch) << std::dec;
        } else {
          out << static_cast<char>(ch);
        }
    }
  }
  return out.str();
}

Vec3 rosToGltf(const Vec3 & value)
{
  // ROS is x-forward, y-left, z-up. glTF convention is x-right, y-up.
  return Vec3{value.x, value.z, -value.y};
}

void addQuad(
  MeshPrimitive & primitive,
  const Vec3 & p0,
  const Vec3 & p1,
  const Vec3 & p2,
  const Vec3 & p3,
  const Vec3 & normal)
{
  const std::uint32_t base = static_cast<std::uint32_t>(primitive.positions.size());
  const Vec3 gltf_normal = rosToGltf(normal);
  primitive.positions.push_back(rosToGltf(p0));
  primitive.positions.push_back(rosToGltf(p1));
  primitive.positions.push_back(rosToGltf(p2));
  primitive.positions.push_back(rosToGltf(p3));
  primitive.normals.insert(primitive.normals.end(), 4U, gltf_normal);
  primitive.indices.insert(
    primitive.indices.end(),
    {base, base + 1U, base + 2U, base, base + 2U, base + 3U});
}

void alignFour(std::vector<std::uint8_t> & bytes)
{
  while ((bytes.size() % 4U) != 0U) {
    bytes.push_back(0U);
  }
}

template<typename T>
BufferView appendBufferView(
  std::vector<std::uint8_t> & bytes,
  const std::vector<T> & values,
  int target)
{
  alignFour(bytes);
  const std::size_t offset = bytes.size();
  const std::size_t byte_count = values.size() * sizeof(T);
  if (byte_count > 0U) {
    const auto * begin = reinterpret_cast<const std::uint8_t *>(values.data());
    bytes.insert(bytes.end(), begin, begin + byte_count);
  }
  return BufferView{offset, byte_count, target};
}

void writeU32(std::ofstream & output, std::uint32_t value)
{
  const std::array<char, 4> bytes{
    static_cast<char>(value & 0xFFU),
    static_cast<char>((value >> 8U) & 0xFFU),
    static_cast<char>((value >> 16U) & 0xFFU),
    static_cast<char>((value >> 24U) & 0xFFU)};
  output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

double mapYaw(const nav_msgs::msg::OccupancyGrid & map)
{
  const auto & orientation = map.info.origin.orientation;
  tf2::Quaternion rotation(orientation.x, orientation.y, orientation.z, orientation.w);
  if (rotation.length2() <= 1e-12) {
    return 0.0;
  }
  rotation.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
  return yaw;
}

Vec3 mapPoint(
  const nav_msgs::msg::OccupancyGrid & map,
  double yaw,
  double grid_x,
  double grid_y,
  double z)
{
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const double x = map.info.origin.position.x + cosine * grid_x - sine * grid_y;
  const double y = map.info.origin.position.y + sine * grid_x + cosine * grid_y;
  return Vec3{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
}

Vec3 mapNormal(double yaw, double grid_x, double grid_y)
{
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  return Vec3{
    static_cast<float>(cosine * grid_x - sine * grid_y),
    static_cast<float>(sine * grid_x + cosine * grid_y),
    0.0F};
}

bool worldToCell(
  const nav_msgs::msg::OccupancyGrid & map,
  double yaw,
  double world_x,
  double world_y,
  int & column,
  int & row)
{
  const double dx = world_x - map.info.origin.position.x;
  const double dy = world_y - map.info.origin.position.y;
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const double grid_x = cosine * dx + sine * dy;
  const double grid_y = -sine * dx + cosine * dy;
  const double resolution = static_cast<double>(map.info.resolution);
  column = static_cast<int>(std::floor(grid_x / resolution));
  row = static_cast<int>(std::floor(grid_y / resolution));
  return column >= 0 && row >= 0 &&
         column < static_cast<int>(map.info.width) && row < static_cast<int>(map.info.height);
}

double quantile(const std::vector<float> & sorted_values, double fraction)
{
  if (sorted_values.empty()) {
    return 0.0;
  }
  const double position = std::clamp(fraction, 0.0, 1.0) *
    static_cast<double>(sorted_values.size() - 1U);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  const double alpha = position - static_cast<double>(lower);
  return static_cast<double>(sorted_values[lower]) * (1.0 - alpha) +
         static_cast<double>(sorted_values[upper]) * alpha;
}

void estimateRoomHeight(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const StructuralMeshConfig & config,
  double & floor_z,
  double & ceiling_z)
{
  floor_z = config.default_floor_z;
  ceiling_z = std::max(config.default_ceiling_z, floor_z + config.min_room_height);
  if (!config.auto_height || cloud.empty() || config.max_height_samples == 0U) {
    return;
  }

  const std::size_t stride = std::max<std::size_t>(
    1U, (cloud.size() + config.max_height_samples - 1U) / config.max_height_samples);
  std::vector<float> z_samples;
  z_samples.reserve(std::min(cloud.size(), config.max_height_samples));
  for (std::size_t index = 0; index < cloud.size(); index += stride) {
    const float z = cloud.points[index].z;
    if (std::isfinite(z)) {
      z_samples.push_back(z);
    }
  }
  if (z_samples.size() < 20U) {
    return;
  }

  std::sort(z_samples.begin(), z_samples.end());
  const double candidate_floor = quantile(z_samples, config.floor_quantile);
  const double candidate_ceiling = quantile(z_samples, config.ceiling_quantile);
  const double candidate_height = candidate_ceiling - candidate_floor;
  if (!std::isfinite(candidate_height) || candidate_height < config.min_room_height) {
    return;
  }

  floor_z = candidate_floor;
  ceiling_z = std::min(candidate_ceiling, floor_z + config.max_room_height);
  ceiling_z = std::max(ceiling_z, floor_z + config.min_room_height);
}

bool writeGlb(
  const std::vector<MeshPrimitive> & primitives,
  const std::filesystem::path & output_file,
  const std::string & source_frame,
  double floor_z,
  double ceiling_z,
  std::size_t & output_bytes,
  std::string & error_message)
{
  std::vector<std::uint8_t> binary;
  std::vector<BufferView> views;
  std::vector<Accessor> accessors;
  std::vector<PrimitiveAccessors> primitive_accessors;

  for (const auto & primitive : primitives) {
    if (primitive.positions.empty() || primitive.indices.empty()) {
      continue;
    }

    const std::size_t position_view = views.size();
    views.push_back(appendBufferView(binary, primitive.positions, 34962));
    Vec3 minimum{
      std::numeric_limits<float>::infinity(),
      std::numeric_limits<float>::infinity(),
      std::numeric_limits<float>::infinity()};
    Vec3 maximum{
      -std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity()};
    for (const auto & point : primitive.positions) {
      minimum.x = std::min(minimum.x, point.x);
      minimum.y = std::min(minimum.y, point.y);
      minimum.z = std::min(minimum.z, point.z);
      maximum.x = std::max(maximum.x, point.x);
      maximum.y = std::max(maximum.y, point.y);
      maximum.z = std::max(maximum.z, point.z);
    }
    const std::size_t position_accessor = accessors.size();
    accessors.push_back(Accessor{
      position_view, 5126, primitive.positions.size(), "VEC3", true, minimum, maximum});

    const std::size_t normal_view = views.size();
    views.push_back(appendBufferView(binary, primitive.normals, 34962));
    const std::size_t normal_accessor = accessors.size();
    accessors.push_back(Accessor{
      normal_view, 5126, primitive.normals.size(), "VEC3", false, {}, {}});

    const std::size_t index_view = views.size();
    views.push_back(appendBufferView(binary, primitive.indices, 34963));
    const std::size_t index_accessor = accessors.size();
    accessors.push_back(Accessor{
      index_view, 5125, primitive.indices.size(), "SCALAR", false, {}, {}});
    primitive_accessors.push_back(PrimitiveAccessors{
      position_accessor, normal_accessor, index_accessor, primitive.material});
  }

  if (primitive_accessors.empty()) {
    error_message = "Structural mesh contains no triangles.";
    return false;
  }
  alignFour(binary);

  std::ostringstream json;
  json << std::setprecision(9);
  json << "{\"asset\":{\"version\":\"2.0\",\"generator\":\"vertical_lidar_mapper\","
       << "\"extras\":{\"sourceFrame\":\"" << jsonEscape(source_frame) << "\","
       << "\"floorZ\":" << floor_z << ",\"ceilingZ\":" << ceiling_z << "}},"
       << "\"scene\":0,\"scenes\":[{\"nodes\":[0]}],"
       << "\"nodes\":[{\"mesh\":0,\"name\":\"Structural Environment\"}],"
       << "\"meshes\":[{\"name\":\"SLAM Structural Mesh\",\"primitives\":[";
  for (std::size_t index = 0; index < primitive_accessors.size(); ++index) {
    if (index > 0U) {
      json << ',';
    }
    const auto & primitive = primitive_accessors[index];
    json << "{\"attributes\":{\"POSITION\":" << primitive.position
         << ",\"NORMAL\":" << primitive.normal << "},\"indices\":" << primitive.index
         << ",\"material\":" << primitive.material << '}';
  }
  json << "]}],"
       << "\"materials\":["
       << "{\"name\":\"Floor\",\"doubleSided\":true,\"pbrMetallicRoughness\":{"
       << "\"baseColorFactor\":[0.32,0.36,0.38,1],\"metallicFactor\":0,\"roughnessFactor\":0.92}},"
       << "{\"name\":\"Walls\",\"doubleSided\":true,\"pbrMetallicRoughness\":{"
       << "\"baseColorFactor\":[0.68,0.72,0.74,1],\"metallicFactor\":0,\"roughnessFactor\":0.82}},"
       << "{\"name\":\"Ceiling\",\"doubleSided\":true,\"pbrMetallicRoughness\":{"
       << "\"baseColorFactor\":[0.82,0.84,0.80,1],\"metallicFactor\":0,\"roughnessFactor\":0.95}}],"
       << "\"buffers\":[{\"byteLength\":" << binary.size() << "}],\"bufferViews\":[";
  for (std::size_t index = 0; index < views.size(); ++index) {
    if (index > 0U) {
      json << ',';
    }
    const auto & view = views[index];
    json << "{\"buffer\":0,\"byteOffset\":" << view.offset
         << ",\"byteLength\":" << view.length << ",\"target\":" << view.target << '}';
  }
  json << "],\"accessors\":[";
  for (std::size_t index = 0; index < accessors.size(); ++index) {
    if (index > 0U) {
      json << ',';
    }
    const auto & accessor = accessors[index];
    json << "{\"bufferView\":" << accessor.buffer_view << ",\"componentType\":"
         << accessor.component_type << ",\"count\":" << accessor.count
         << ",\"type\":\"" << accessor.type << '"';
    if (accessor.has_bounds) {
      json << ",\"min\":[" << accessor.minimum.x << ',' << accessor.minimum.y << ','
           << accessor.minimum.z << "],\"max\":[" << accessor.maximum.x << ','
           << accessor.maximum.y << ',' << accessor.maximum.z << ']';
    }
    json << '}';
  }
  json << "]}";

  std::string json_text = json.str();
  while ((json_text.size() % 4U) != 0U) {
    json_text.push_back(' ');
  }

  const std::uint64_t total_length_64 = 12ULL + 8ULL + json_text.size() + 8ULL + binary.size();
  if (total_length_64 > std::numeric_limits<std::uint32_t>::max()) {
    error_message = "Structural GLB exceeds the 4 GiB format limit.";
    return false;
  }

  std::ofstream output(output_file, std::ios::binary);
  if (!output.good()) {
    error_message = "Failed to open structural GLB output: " + output_file.string();
    return false;
  }
  writeU32(output, kGlbMagic);
  writeU32(output, kGlbVersion);
  writeU32(output, static_cast<std::uint32_t>(total_length_64));
  writeU32(output, static_cast<std::uint32_t>(json_text.size()));
  writeU32(output, kJsonChunkType);
  output.write(json_text.data(), static_cast<std::streamsize>(json_text.size()));
  writeU32(output, static_cast<std::uint32_t>(binary.size()));
  writeU32(output, kBinChunkType);
  if (!binary.empty()) {
    output.write(
      reinterpret_cast<const char *>(binary.data()),
      static_cast<std::streamsize>(binary.size()));
  }
  if (!output.good()) {
    error_message = "Failed while writing structural GLB: " + output_file.string();
    return false;
  }

  output_bytes = static_cast<std::size_t>(total_length_64);
  return true;
}

}  // namespace

bool exportStructuralMeshGlb(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const nav_msgs::msg::OccupancyGrid & map,
  const StructuralMeshConfig & config,
  const std::filesystem::path & output_file,
  const std::string & source_frame,
  StructuralMeshStats & stats,
  std::string & error_message)
{
  const auto started = std::chrono::steady_clock::now();
  stats = StructuralMeshStats{};
  error_message.clear();

  const std::size_t width = static_cast<std::size_t>(map.info.width);
  const std::size_t height = static_cast<std::size_t>(map.info.height);
  if (width == 0U || height == 0U || map.info.resolution <= 0.0F) {
    error_message = "SLAM map has invalid dimensions or resolution.";
    return false;
  }
  if (height > std::numeric_limits<std::size_t>::max() / width) {
    error_message = "SLAM map dimensions overflow addressable memory.";
    return false;
  }
  const std::size_t cell_count = width * height;
  if (cell_count > config.max_grid_cells) {
    error_message = "SLAM map has " + std::to_string(cell_count) +
      " cells, above structural_mesh_max_grid_cells=" + std::to_string(config.max_grid_cells) + ".";
    return false;
  }
  if (map.data.size() != cell_count) {
    error_message = "SLAM map data size does not match its dimensions.";
    return false;
  }
  if (config.free_threshold >= config.occupied_threshold) {
    error_message = "Structural free threshold must be lower than occupied threshold.";
    return false;
  }

  double floor_z = config.default_floor_z;
  double ceiling_z = config.default_ceiling_z;
  estimateRoomHeight(cloud, config, floor_z, ceiling_z);
  stats.floor_z = floor_z;
  stats.ceiling_z = ceiling_z;

  const double yaw = mapYaw(map);
  const double resolution = static_cast<double>(map.info.resolution);
  const auto cellIndex = [width](std::size_t column, std::size_t row) {
      return row * width + column;
    };
  const auto isFree = [&](int column, int row) {
      if (column < 0 || row < 0 || column >= static_cast<int>(width) || row >= static_cast<int>(height)) {
        return false;
      }
      const int value = static_cast<int>(map.data[cellIndex(
            static_cast<std::size_t>(column), static_cast<std::size_t>(row))]);
      return value >= 0 && value <= config.free_threshold;
    };
  const auto isOccupied = [&](int column, int row) {
      if (column < 0 || row < 0 || column >= static_cast<int>(width) || row >= static_cast<int>(height)) {
        return false;
      }
      return static_cast<int>(map.data[cellIndex(
          static_cast<std::size_t>(column), static_cast<std::size_t>(row))]) >= config.occupied_threshold;
    };

  std::vector<float> cell_max_z;
  if (config.use_obstacle_heights) {
    cell_max_z.assign(cell_count, -std::numeric_limits<float>::infinity());
    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      int column = 0;
      int row = 0;
      if (worldToCell(map, yaw, point.x, point.y, column, row)) {
        const std::size_t index = cellIndex(
          static_cast<std::size_t>(column), static_cast<std::size_t>(row));
        cell_max_z[index] = std::max(cell_max_z[index], point.z);
      }
    }
  }

  std::vector<float> occupied_top(cell_count, std::numeric_limits<float>::quiet_NaN());
  const int search_radius = std::max(0, config.height_search_radius_cells);
  const double quantization = std::max(0.01, config.height_quantization);
  for (std::size_t row = 0; row < height; ++row) {
    for (std::size_t column = 0; column < width; ++column) {
      if (!isOccupied(static_cast<int>(column), static_cast<int>(row))) {
        continue;
      }
      float observed_top = -std::numeric_limits<float>::infinity();
      if (config.use_obstacle_heights) {
        for (int dy = -search_radius; dy <= search_radius; ++dy) {
          for (int dx = -search_radius; dx <= search_radius; ++dx) {
            const int sample_column = static_cast<int>(column) + dx;
            const int sample_row = static_cast<int>(row) + dy;
            if (sample_column < 0 || sample_row < 0 ||
              sample_column >= static_cast<int>(width) || sample_row >= static_cast<int>(height))
            {
              continue;
            }
            observed_top = std::max(
              observed_top,
              cell_max_z[cellIndex(
                static_cast<std::size_t>(sample_column), static_cast<std::size_t>(sample_row))]);
          }
        }
      }

      double top = ceiling_z;
      if (config.use_obstacle_heights && std::isfinite(observed_top) &&
        static_cast<double>(observed_top) >= floor_z + config.min_obstacle_height)
      {
        top = static_cast<double>(observed_top) + config.obstacle_height_padding;
      }
      top = std::clamp(top, floor_z + config.min_obstacle_height, ceiling_z);
      top = floor_z + std::round((top - floor_z) / quantization) * quantization;
      top = std::clamp(top, floor_z + config.min_obstacle_height, ceiling_z);
      occupied_top[cellIndex(column, row)] = static_cast<float>(top);
    }
  }

  MeshPrimitive floor_mesh{"Floor", 0, {}, {}, {}};
  MeshPrimitive wall_mesh{"Walls", 1, {}, {}, {}};
  MeshPrimitive ceiling_mesh{"Ceiling", 2, {}, {}, {}};
  std::size_t quad_count = 0U;

  std::vector<std::uint8_t> visited(cell_count, 0U);
  for (std::size_t row = 0; row < height; ++row) {
    for (std::size_t column = 0; column < width; ++column) {
      const std::size_t start_index = cellIndex(column, row);
      if (visited[start_index] != 0U || !isFree(static_cast<int>(column), static_cast<int>(row))) {
        continue;
      }

      std::size_t rectangle_width = 1U;
      while (column + rectangle_width < width) {
        const std::size_t index = cellIndex(column + rectangle_width, row);
        if (visited[index] != 0U ||
          !isFree(static_cast<int>(column + rectangle_width), static_cast<int>(row)))
        {
          break;
        }
        ++rectangle_width;
      }

      std::size_t rectangle_height = 1U;
      bool can_expand = true;
      while (row + rectangle_height < height && can_expand) {
        for (std::size_t dx = 0; dx < rectangle_width; ++dx) {
          const std::size_t index = cellIndex(column + dx, row + rectangle_height);
          if (visited[index] != 0U ||
            !isFree(static_cast<int>(column + dx), static_cast<int>(row + rectangle_height)))
          {
            can_expand = false;
            break;
          }
        }
        if (can_expand) {
          ++rectangle_height;
        }
      }

      for (std::size_t dy = 0; dy < rectangle_height; ++dy) {
        for (std::size_t dx = 0; dx < rectangle_width; ++dx) {
          visited[cellIndex(column + dx, row + dy)] = 1U;
        }
      }

      const double x0 = static_cast<double>(column) * resolution;
      const double y0 = static_cast<double>(row) * resolution;
      const double x1 = static_cast<double>(column + rectangle_width) * resolution;
      const double y1 = static_cast<double>(row + rectangle_height) * resolution;
      const std::size_t required_quads = config.include_ceiling ? 2U : 1U;
      if (quad_count + required_quads > config.max_quads) {
        error_message = "Structural mesh exceeds structural_mesh_max_quads=" +
          std::to_string(config.max_quads) + ".";
        return false;
      }
      addQuad(
        floor_mesh,
        mapPoint(map, yaw, x0, y0, floor_z),
        mapPoint(map, yaw, x1, y0, floor_z),
        mapPoint(map, yaw, x1, y1, floor_z),
        mapPoint(map, yaw, x0, y1, floor_z),
        Vec3{0.0F, 0.0F, 1.0F});
      if (config.include_ceiling) {
        addQuad(
          ceiling_mesh,
          mapPoint(map, yaw, x0, y1, ceiling_z),
          mapPoint(map, yaw, x1, y1, ceiling_z),
          mapPoint(map, yaw, x1, y0, ceiling_z),
          mapPoint(map, yaw, x0, y0, ceiling_z),
          Vec3{0.0F, 0.0F, -1.0F});
      }
      quad_count += required_quads;
      ++stats.floor_rectangles;
    }
  }

  const auto topAt = [&](std::size_t column, std::size_t row) {
      return occupied_top[cellIndex(column, row)];
    };
  const auto sameTop = [](float left, float right) {
      return std::fabs(static_cast<double>(left) - static_cast<double>(right)) < 1e-4;
    };

  // South and north faces merge along grid rows.
  for (std::size_t row = 0; row < height; ++row) {
    for (int side = 0; side < 2; ++side) {
      std::size_t column = 0U;
      while (column < width) {
        const int neighbor_row = side == 0 ? static_cast<int>(row) - 1 : static_cast<int>(row) + 1;
        const bool boundary = isOccupied(static_cast<int>(column), static_cast<int>(row)) &&
          isFree(static_cast<int>(column), neighbor_row);
        if (!boundary) {
          ++column;
          continue;
        }
        const float top = topAt(column, row);
        const std::size_t run_start = column;
        ++column;
        while (column < width &&
          isOccupied(static_cast<int>(column), static_cast<int>(row)) &&
          isFree(static_cast<int>(column), neighbor_row) && sameTop(topAt(column, row), top))
        {
          ++column;
        }
        const double x0 = static_cast<double>(run_start) * resolution;
        const double x1 = static_cast<double>(column) * resolution;
        const double y = static_cast<double>(side == 0 ? row : row + 1U) * resolution;
        if (quad_count >= config.max_quads) {
          error_message = "Structural mesh exceeds structural_mesh_max_quads=" +
            std::to_string(config.max_quads) + ".";
          return false;
        }
        if (side == 0) {
          addQuad(
            wall_mesh,
            mapPoint(map, yaw, x0, y, floor_z), mapPoint(map, yaw, x1, y, floor_z),
            mapPoint(map, yaw, x1, y, top), mapPoint(map, yaw, x0, y, top),
            mapNormal(yaw, 0.0, -1.0));
        } else {
          addQuad(
            wall_mesh,
            mapPoint(map, yaw, x1, y, floor_z), mapPoint(map, yaw, x0, y, floor_z),
            mapPoint(map, yaw, x0, y, top), mapPoint(map, yaw, x1, y, top),
            mapNormal(yaw, 0.0, 1.0));
        }
        ++quad_count;
        ++stats.wall_runs;
      }
    }
  }

  // West and east faces merge along grid columns.
  for (std::size_t column = 0; column < width; ++column) {
    for (int side = 0; side < 2; ++side) {
      std::size_t row = 0U;
      while (row < height) {
        const int neighbor_column = side == 0 ? static_cast<int>(column) - 1 : static_cast<int>(column) + 1;
        const bool boundary = isOccupied(static_cast<int>(column), static_cast<int>(row)) &&
          isFree(neighbor_column, static_cast<int>(row));
        if (!boundary) {
          ++row;
          continue;
        }
        const float top = topAt(column, row);
        const std::size_t run_start = row;
        ++row;
        while (row < height &&
          isOccupied(static_cast<int>(column), static_cast<int>(row)) &&
          isFree(neighbor_column, static_cast<int>(row)) && sameTop(topAt(column, row), top))
        {
          ++row;
        }
        const double y0 = static_cast<double>(run_start) * resolution;
        const double y1 = static_cast<double>(row) * resolution;
        const double x = static_cast<double>(side == 0 ? column : column + 1U) * resolution;
        if (quad_count >= config.max_quads) {
          error_message = "Structural mesh exceeds structural_mesh_max_quads=" +
            std::to_string(config.max_quads) + ".";
          return false;
        }
        if (side == 0) {
          addQuad(
            wall_mesh,
            mapPoint(map, yaw, x, y1, floor_z), mapPoint(map, yaw, x, y0, floor_z),
            mapPoint(map, yaw, x, y0, top), mapPoint(map, yaw, x, y1, top),
            mapNormal(yaw, -1.0, 0.0));
        } else {
          addQuad(
            wall_mesh,
            mapPoint(map, yaw, x, y0, floor_z), mapPoint(map, yaw, x, y1, floor_z),
            mapPoint(map, yaw, x, y1, top), mapPoint(map, yaw, x, y0, top),
            mapNormal(yaw, 1.0, 0.0));
        }
        ++quad_count;
        ++stats.wall_runs;
      }
    }
  }

  std::vector<MeshPrimitive> primitives;
  primitives.push_back(std::move(floor_mesh));
  primitives.push_back(std::move(wall_mesh));
  if (config.include_ceiling) {
    primitives.push_back(std::move(ceiling_mesh));
  }
  for (const auto & primitive : primitives) {
    stats.vertices += primitive.positions.size();
    stats.triangles += primitive.indices.size() / 3U;
  }
  if (stats.triangles == 0U) {
    error_message = "No free floor cells or occupied/free wall boundaries were available for meshing.";
    return false;
  }

  if (!writeGlb(
      primitives, output_file, source_frame, floor_z, ceiling_z,
      stats.binary_bytes, error_message))
  {
    return false;
  }
  stats.duration_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  return true;
}

}  // namespace vertical_lidar_mapper
