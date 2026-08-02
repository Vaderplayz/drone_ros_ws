#include "submap_slam_2d/pose_graph.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <ceres/ceres.h>

namespace submap_slam_2d
{
namespace
{

struct RelativePoseResidual
{
  explicit RelativePoseResidual(const GraphEdge & edge)
  : measurement(edge.measurement),
    translation_weight(1.0 / edge.translation_stddev), yaw_weight(1.0 / edge.yaw_stddev) {}

  template<typename T>
  bool operator()(const T * const from, const T * const to, T * residual) const
  {
    const T dx = to[0] - from[0];
    const T dy = to[1] - from[1];
    const T c = cos(from[2]);
    const T s = sin(from[2]);
    const T relative_x = c * dx + s * dy;
    const T relative_y = -s * dx + c * dy;
    const T relative_yaw = atan2(sin(to[2] - from[2]), cos(to[2] - from[2]));
    residual[0] = T(translation_weight) * (relative_x - T(measurement.x));
    residual[1] = T(translation_weight) * (relative_y - T(measurement.y));
    residual[2] = T(yaw_weight) * atan2(
      sin(relative_yaw - T(measurement.yaw)), cos(relative_yaw - T(measurement.yaw)));
    return true;
  }

  Pose2 measurement;
  double translation_weight;
  double yaw_weight;
};

}  // namespace

PoseGraph2D::PoseGraph2D(PoseGraphConfig config)
: config_(config)
{
  if (config_.max_iterations <= 0 || config_.loop_huber_scale <= 0.0) {
    throw std::invalid_argument("invalid pose graph configuration");
  }
}

int PoseGraph2D::addVertex(const Pose2 & pose)
{
  if (!std::isfinite(pose.x + pose.y + pose.yaw)) {
    throw std::invalid_argument("non-finite pose graph vertex");
  }
  poses_.push_back(pose);
  return static_cast<int>(poses_.size() - 1U);
}

void PoseGraph2D::addEdge(const GraphEdge & edge)
{
  if (edge.from < 0 || edge.to < 0 || edge.from >= static_cast<int>(poses_.size()) ||
    edge.to >= static_cast<int>(poses_.size()) || edge.from == edge.to ||
    edge.translation_stddev <= 0.0 || edge.yaw_stddev <= 0.0 ||
    !std::isfinite(edge.measurement.x + edge.measurement.y + edge.measurement.yaw))
  {
    throw std::invalid_argument("invalid pose graph edge");
  }
  edges_.push_back(edge);
}

std::size_t PoseGraph2D::edgeCount(EdgeType type) const
{
  return static_cast<std::size_t>(std::count_if(
    edges_.begin(), edges_.end(), [type](const GraphEdge & edge) {return edge.type == type;}));
}

OptimizationSummary PoseGraph2D::optimize()
{
  OptimizationSummary result;
  if (poses_.empty()) {
    result.message = "empty_graph";
    return result;
  }
  const auto start = std::chrono::steady_clock::now();
  std::vector<std::array<double, 3>> parameters;
  parameters.reserve(poses_.size());
  for (const Pose2 & pose : poses_) {
    parameters.push_back({pose.x, pose.y, pose.yaw});
  }
  ceres::Problem problem;
  for (auto & pose : parameters) {
    problem.AddParameterBlock(pose.data(), 3);
  }
  for (const GraphEdge & edge : edges_) {
    auto * cost = new ceres::AutoDiffCostFunction<RelativePoseResidual, 3, 3, 3>(
      new RelativePoseResidual(edge));
    ceres::LossFunction * loss = edge.type == EdgeType::Loop ?
      static_cast<ceres::LossFunction *>(new ceres::HuberLoss(config_.loop_huber_scale)) : nullptr;
    problem.AddResidualBlock(
      cost, loss, parameters[static_cast<std::size_t>(edge.from)].data(),
      parameters[static_cast<std::size_t>(edge.to)].data());
  }
  problem.SetParameterBlockConstant(parameters.front().data());
  ceres::Solver::Options options;
  options.max_num_iterations = config_.max_iterations;
  options.num_threads = 1;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  result.duration_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  result.initial_cost = summary.initial_cost;
  result.final_cost = summary.final_cost;
  result.iterations = static_cast<int>(summary.iterations.size());
  result.success = summary.IsSolutionUsable();
  result.message = summary.BriefReport();
  if (!result.success) {
    return result;
  }
  for (std::size_t index = 0; index < poses_.size(); ++index) {
    const auto & optimized = parameters[index];
    if (!std::isfinite(optimized[0] + optimized[1] + optimized[2])) {
      result.success = false;
      result.message = "non_finite_optimized_pose";
      return result;
    }
    poses_[index] = {optimized[0], optimized[1], wrap_angle(optimized[2])};
  }
  return result;
}

}  // namespace submap_slam_2d
