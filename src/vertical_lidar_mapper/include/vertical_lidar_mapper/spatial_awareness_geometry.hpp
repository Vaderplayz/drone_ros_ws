#ifndef VERTICAL_LIDAR_MAPPER__SPATIAL_AWARENESS_GEOMETRY_HPP_
#define VERTICAL_LIDAR_MAPPER__SPATIAL_AWARENESS_GEOMETRY_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>

namespace vertical_lidar_mapper::spatial_awareness
{

enum class Direction : std::size_t
{
  Front = 0,
  Rear,
  Left,
  Right,
  Top,
  Bottom,
  Count
};

enum class ClearanceState
{
  Clear,
  Warning,
  Danger,
  Unknown
};

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Bounds
{
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};
  double min_z{0.0};
  double max_z{0.0};
};

struct CollisionGeometry
{
  Point3 center;
  double body_length_m{0.45};
  double body_width_m{0.35};
  double body_height_m{0.22};
  double propeller_tip_to_tip_length_m{0.75};
  double propeller_tip_to_tip_width_m{0.75};
  double horizontal_safety_margin_m{0.40};
  double vertical_safety_margin_m{0.30};
  double horizontal_danger_margin_m{0.15};
  double vertical_danger_margin_m{0.12};
  double self_filter_padding_m{0.04};
};

struct DirectionResult
{
  Direction direction{Direction::Front};
  ClearanceState state{ClearanceState::Unknown};
  bool sensor_coverage{false};
  std::optional<double> clearance_m;
  std::optional<Point3> nearest_point;
};

inline constexpr std::array<Direction, 6> kDirections{
  Direction::Front, Direction::Rear, Direction::Left,
  Direction::Right, Direction::Top, Direction::Bottom};

inline std::string directionName(Direction direction)
{
  switch (direction) {
    case Direction::Front: return "front";
    case Direction::Rear: return "rear";
    case Direction::Left: return "left";
    case Direction::Right: return "right";
    case Direction::Top: return "top";
    case Direction::Bottom: return "bottom";
    case Direction::Count: break;
  }
  return "unknown";
}

inline std::string stateName(ClearanceState state)
{
  switch (state) {
    case ClearanceState::Clear: return "CLEAR";
    case ClearanceState::Warning: return "WARNING";
    case ClearanceState::Danger: return "DANGER";
    case ClearanceState::Unknown: return "UNKNOWN";
  }
  return "UNKNOWN";
}

inline bool isVertical(Direction direction)
{
  return direction == Direction::Top || direction == Direction::Bottom;
}

inline Bounds physicalBounds(const CollisionGeometry & geometry)
{
  const double length = std::max(
    geometry.body_length_m, geometry.propeller_tip_to_tip_length_m);
  const double width = std::max(
    geometry.body_width_m, geometry.propeller_tip_to_tip_width_m);
  return Bounds{
    geometry.center.x - 0.5 * length,
    geometry.center.x + 0.5 * length,
    geometry.center.y - 0.5 * width,
    geometry.center.y + 0.5 * width,
    geometry.center.z - 0.5 * geometry.body_height_m,
    geometry.center.z + 0.5 * geometry.body_height_m};
}

inline Bounds safetyBounds(const CollisionGeometry & geometry)
{
  Bounds bounds = physicalBounds(geometry);
  bounds.min_x -= geometry.horizontal_safety_margin_m;
  bounds.max_x += geometry.horizontal_safety_margin_m;
  bounds.min_y -= geometry.horizontal_safety_margin_m;
  bounds.max_y += geometry.horizontal_safety_margin_m;
  bounds.min_z -= geometry.vertical_safety_margin_m;
  bounds.max_z += geometry.vertical_safety_margin_m;
  return bounds;
}

inline bool isFinite(const Point3 & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

inline bool pointInsideBounds(const Point3 & point, const Bounds & bounds)
{
  return point.x >= bounds.min_x && point.x <= bounds.max_x &&
         point.y >= bounds.min_y && point.y <= bounds.max_y &&
         point.z >= bounds.min_z && point.z <= bounds.max_z;
}

inline bool isSelfPoint(const Point3 & point, const CollisionGeometry & geometry)
{
  Bounds bounds = physicalBounds(geometry);
  bounds.min_x -= geometry.self_filter_padding_m;
  bounds.max_x += geometry.self_filter_padding_m;
  bounds.min_y -= geometry.self_filter_padding_m;
  bounds.max_y += geometry.self_filter_padding_m;
  bounds.min_z -= geometry.self_filter_padding_m;
  bounds.max_z += geometry.self_filter_padding_m;
  return pointInsideBounds(point, bounds);
}

inline std::optional<double> directionalClearance(
  const Point3 & point,
  Direction direction,
  const CollisionGeometry & geometry)
{
  if (!isFinite(point)) {
    return std::nullopt;
  }

  const Bounds physical = physicalBounds(geometry);
  const Bounds safety = safetyBounds(geometry);
  switch (direction) {
    case Direction::Front:
      if (point.x >= physical.max_x && point.y >= safety.min_y && point.y <= safety.max_y &&
        point.z >= safety.min_z && point.z <= safety.max_z)
      {
        return point.x - physical.max_x;
      }
      break;
    case Direction::Rear:
      if (point.x <= physical.min_x && point.y >= safety.min_y && point.y <= safety.max_y &&
        point.z >= safety.min_z && point.z <= safety.max_z)
      {
        return physical.min_x - point.x;
      }
      break;
    case Direction::Left:
      if (point.y >= physical.max_y && point.x >= safety.min_x && point.x <= safety.max_x &&
        point.z >= safety.min_z && point.z <= safety.max_z)
      {
        return point.y - physical.max_y;
      }
      break;
    case Direction::Right:
      if (point.y <= physical.min_y && point.x >= safety.min_x && point.x <= safety.max_x &&
        point.z >= safety.min_z && point.z <= safety.max_z)
      {
        return physical.min_y - point.y;
      }
      break;
    case Direction::Top:
      if (point.z >= physical.max_z && point.x >= safety.min_x && point.x <= safety.max_x &&
        point.y >= safety.min_y && point.y <= safety.max_y)
      {
        return point.z - physical.max_z;
      }
      break;
    case Direction::Bottom:
      if (point.z <= physical.min_z && point.x >= safety.min_x && point.x <= safety.max_x &&
        point.y >= safety.min_y && point.y <= safety.max_y)
      {
        return physical.min_z - point.z;
      }
      break;
    case Direction::Count:
      break;
  }
  return std::nullopt;
}

inline ClearanceState classifyClearance(
  const std::optional<double> & clearance_m,
  Direction direction,
  bool sensor_coverage,
  const CollisionGeometry & geometry)
{
  if (!sensor_coverage) {
    return ClearanceState::Unknown;
  }
  if (!clearance_m.has_value()) {
    return ClearanceState::Clear;
  }

  const double danger_margin = isVertical(direction) ?
    geometry.vertical_danger_margin_m : geometry.horizontal_danger_margin_m;
  const double warning_margin = isVertical(direction) ?
    geometry.vertical_safety_margin_m : geometry.horizontal_safety_margin_m;
  if (clearance_m.value() <= danger_margin) {
    return ClearanceState::Danger;
  }
  if (clearance_m.value() <= warning_margin) {
    return ClearanceState::Warning;
  }
  return ClearanceState::Clear;
}

inline Point3 directionUnitVector(Direction direction)
{
  switch (direction) {
    case Direction::Front: return Point3{1.0, 0.0, 0.0};
    case Direction::Rear: return Point3{-1.0, 0.0, 0.0};
    case Direction::Left: return Point3{0.0, 1.0, 0.0};
    case Direction::Right: return Point3{0.0, -1.0, 0.0};
    case Direction::Top: return Point3{0.0, 0.0, 1.0};
    case Direction::Bottom: return Point3{0.0, 0.0, -1.0};
    case Direction::Count: break;
  }
  return Point3{};
}

inline Point3 collisionSurfacePoint(Direction direction, const CollisionGeometry & geometry)
{
  const Bounds bounds = physicalBounds(geometry);
  Point3 point = geometry.center;
  switch (direction) {
    case Direction::Front: point.x = bounds.max_x; break;
    case Direction::Rear: point.x = bounds.min_x; break;
    case Direction::Left: point.y = bounds.max_y; break;
    case Direction::Right: point.y = bounds.min_y; break;
    case Direction::Top: point.z = bounds.max_z; break;
    case Direction::Bottom: point.z = bounds.min_z; break;
    case Direction::Count: break;
  }
  return point;
}

}  // namespace vertical_lidar_mapper::spatial_awareness

#endif  // VERTICAL_LIDAR_MAPPER__SPATIAL_AWARENESS_GEOMETRY_HPP_
