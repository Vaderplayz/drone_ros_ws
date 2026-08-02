#pragma once

#include <cmath>

namespace submap_slam_2d
{

inline double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  Point2 transform(const Point2 & point) const
  {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {x + c * point.x - s * point.y, y + s * point.x + c * point.y};
  }

  Pose2 inverse() const
  {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {-c * x - s * y, s * x - c * y, wrap_angle(-yaw)};
  }

  Pose2 operator*(const Pose2 & rhs) const
  {
    const Point2 translated = transform({rhs.x, rhs.y});
    return {translated.x, translated.y, wrap_angle(yaw + rhs.yaw)};
  }
};

inline Pose2 between(const Pose2 & from, const Pose2 & to)
{
  return from.inverse() * to;
}

}  // namespace submap_slam_2d
