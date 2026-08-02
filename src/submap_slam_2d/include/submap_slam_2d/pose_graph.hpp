#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "submap_slam_2d/se2.hpp"

namespace submap_slam_2d
{

enum class EdgeType {Odometry, Registration, Loop};

struct GraphEdge
{
  int from{0};
  int to{0};
  Pose2 measurement;
  double translation_stddev{0.2};
  double yaw_stddev{0.15};
  EdgeType type{EdgeType::Odometry};
};

struct OptimizationSummary
{
  bool success{false};
  double duration_ms{0.0};
  double initial_cost{0.0};
  double final_cost{0.0};
  int iterations{0};
  std::string message{"not_run"};
};

struct PoseGraphConfig
{
  int max_iterations{40};
  double loop_huber_scale{1.0};
};

class PoseGraph2D
{
public:
  explicit PoseGraph2D(PoseGraphConfig config = {});
  int addVertex(const Pose2 & pose);
  void addEdge(const GraphEdge & edge);
  OptimizationSummary optimize();

  const std::vector<Pose2> & poses() const {return poses_;}
  const std::vector<GraphEdge> & edges() const {return edges_;}
  std::size_t edgeCount(EdgeType type) const;

private:
  PoseGraphConfig config_;
  std::vector<Pose2> poses_;
  std::vector<GraphEdge> edges_;
};

}  // namespace submap_slam_2d
