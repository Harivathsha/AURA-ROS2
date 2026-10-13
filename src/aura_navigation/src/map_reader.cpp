// AURA: beginner-friendly eight-direction A* for Nav2 / ROS 2 Jazzy.
// This file is now a planner plugin, so it has no main() or ROS subscribers.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace aura_navigation
{
class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  // Nav2 supplies its node, TF buffer, and global costmap here.
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = parent.lock();
    if (!node_ || !costmap_ros) {
      throw std::runtime_error("AURA A*: missing node or costmap");
    }
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
    const std::string parameter = name + ".max_traversable_cost";
    if (!node_->has_parameter(parameter)) {
      node_->declare_parameter(parameter, 253);
    }
    node_->get_parameter(parameter, max_cost_);
    if (max_cost_ < 1 || max_cost_ > 253) {
      throw std::invalid_argument("max_traversable_cost must be 1..253");
    }
    const std::string penalty_parameter = name + ".cost_penalty";
    if (!node_->has_parameter(penalty_parameter)) {
      node_->declare_parameter(penalty_parameter, 2.0);
    }
    node_->get_parameter(penalty_parameter, cost_penalty_);
    if (!std::isfinite(cost_penalty_) || cost_penalty_ < 0.0) {
      throw std::invalid_argument("cost_penalty must be finite and non-negative");
    }
    RCLCPP_INFO(node_->get_logger(),
      "AURA A* configured: 8 directions, octile heuristic, cost < %d, penalty %.1f",
      max_cost_, cost_penalty_);
  }

  void activate() override {}
  void deactivate() override {}
  void cleanup() override
  {
    costmap_ = nullptr;
    node_.reset();
  }

  // Nav2 calls this for each planning request and receives one complete path.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override
  {
    if (!node_ || !costmap_) {
      throw nav2_core::PlannerException("AURA A*: plugin is not configured");
    }
    if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
      throw nav2_core::PlannerTFError("AURA A*: start and goal must be in the global frame");
    }
    if (!std::isfinite(start.pose.position.x) || !std::isfinite(start.pose.position.y) ||
      !std::isfinite(goal.pose.position.x) || !std::isfinite(goal.pose.position.y))
    {
      throw nav2_core::PlannerException("AURA A*: non-finite position");
    }

    // Prevent costmap updates while this request is reading the grid.
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    const auto began = std::chrono::steady_clock::now();
    unsigned int start_x, start_y, goal_x, goal_y;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
      throw nav2_core::StartOutsideMapBounds("AURA A*: start is outside the costmap");
    }
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
      throw nav2_core::GoalOutsideMapBounds("AURA A*: goal is outside the costmap");
    }
    if (!is_free(start_x, start_y)) {
      throw nav2_core::StartOccupied("AURA A*: start cost=" +
        std::to_string(costmap_->getCost(start_x, start_y)) +
        " is blocked by cutoff " + std::to_string(max_cost_));
    }
    if (!is_free(goal_x, goal_y)) {
      throw nav2_core::GoalOccupied("AURA A*: goal is inside an obstacle/clearance area");
    }

    const int width = static_cast<int>(costmap_->getSizeInCellsX());
    const std::size_t count = static_cast<std::size_t>(width) * costmap_->getSizeInCellsY();
    std::vector<double> best_g(count, std::numeric_limits<double>::infinity());
    std::vector<int> parent(count, -1);
    std::vector<Cell> open_list;
    const int start_index = start_y * width + start_x;
    const int goal_index = goal_y * width + goal_x;
    best_g[start_index] = 0.0;
    open_list.push_back({static_cast<int>(start_x), static_cast<int>(start_y), 0.0,
      heuristic(start_x, start_y, goal_x, goal_y)});

    const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    std::size_t expanded = 0;
    while (!open_list.empty()) {
      if (cancel_checker && cancel_checker()) {
        throw nav2_core::PlannerCancelled("AURA A*: planning cancelled");
      }
      std::size_t best_index = 0;
      for (std::size_t i = 1; i < open_list.size(); ++i) {
        if (open_list[i].f < open_list[best_index].f) {
          best_index = i;
        }
      }
      const Cell current = open_list[best_index];
      open_list.erase(open_list.begin() + best_index);
      const int current_index = current.y * width + current.x;
      // Skip an old queued route if a cheaper route has replaced it.
      if (current.g > best_g[current_index]) {
        continue;
      }
      ++expanded;
      if (current_index == goal_index) {
        std::vector<int> cells;
        for (int index = goal_index; index != -1; index = parent[index]) {
          cells.push_back(index);
        }
        std::reverse(cells.begin(), cells.end());
        auto path = make_path(cells, width, start, goal);
        double length = 0.0;
        for (std::size_t i = 1; i < path.poses.size(); ++i) {
          const auto & a = path.poses[i - 1].pose.position;
          const auto & b = path.poses[i].pose.position;
          length += std::hypot(b.x - a.x, b.y - a.y);
        }
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - began).count();
        RCLCPP_INFO(node_->get_logger(),
          "AURA A*: %zu poses | length=%.3f m | weighted cost=%.3f | expanded=%zu | %.1f ms",
          path.poses.size(), length, current.g * costmap_->getResolution(), expanded, elapsed_ms);
        return path;
      }

      for (int i = 0; i < 8; ++i) {
        const int next_x = current.x + dx[i];
        const int next_y = current.y + dy[i];
        if (!is_free(next_x, next_y)) {
          continue;
        }
        const bool diagonal = dx[i] != 0 && dy[i] != 0;
        // A diagonal needs both adjoining side cells to be traversable.
        if (diagonal && (!is_free(current.x + dx[i], current.y) ||
          !is_free(current.x, current.y + dy[i])))
        {
          continue;
        }
        const int next_index = next_y * width + next_x;
        const double step = diagonal ? std::sqrt(2.0) : 1.0;
        const double average_cost = (costmap_->getCost(current.x, current.y) +
          costmap_->getCost(next_x, next_y)) / (2.0 * 252.0);
        const double next_g = current.g + step * (1.0 + cost_penalty_ * average_cost);
        if (next_g >= best_g[next_index]) {
          continue;
        }
        best_g[next_index] = next_g;
        parent[next_index] = current_index;
        const double next_f = next_g + heuristic(next_x, next_y, goal_x, goal_y);
        open_list.push_back({next_x, next_y, next_g, next_f});
      }
    }
    throw nav2_core::NoValidPathCouldBeFound("AURA A*: no traversable route to this goal");
  }

private:
  struct Cell
  {
    int x, y;
    double g, f;
  };
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;
  std::string global_frame_;
  int max_cost_ = 253;
  double cost_penalty_ = 2.0;

  bool is_free(int x, int y) const
  {
    if (x < 0 || y < 0 || x >= static_cast<int>(costmap_->getSizeInCellsX()) ||
      y >= static_cast<int>(costmap_->getSizeInCellsY()))
    {
      return false;
    }
    // Nav2 costs: 0=free, 253=inscribed, 254=lethal, 255=unknown.
    // Soft inflation costs are traversable but penalized in the search.
    // Inscribed, lethal and unknown cells remain blocked at cutoff 253.
    return costmap_->getCost(x, y) < max_cost_;
  }

  double heuristic(int x, int y, int goal_x, int goal_y) const
  {
    const int distance_x = std::abs(goal_x - x);
    const int distance_y = std::abs(goal_y - y);
    const int diagonal_steps = std::min(distance_x, distance_y);
    const int straight_steps = std::max(distance_x, distance_y) - diagonal_steps;
    return diagonal_steps * std::sqrt(2.0) + straight_steps;
  }

  nav_msgs::msg::Path make_path(
    const std::vector<int> & cells, int width,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = node_->now();
    for (int index : cells) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      costmap_->mapToWorld(index % width, index / width,
        pose.pose.position.x, pose.pose.position.y);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    // Preserve the real endpoints, including a requested final orientation.
    path.poses.front().pose = start.pose;
    if (path.poses.size() == 1) {
      path.poses.push_back(path.poses.front());
    }
    path.poses.back().pose = goal.pose;
    for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
      const auto & a = path.poses[i].pose.position;
      const auto & b = path.poses[i + 1].pose.position;
      const double yaw = std::atan2(b.y - a.y, b.x - a.x);
      path.poses[i].pose.orientation.x = 0.0;
      path.poses[i].pose.orientation.y = 0.0;
      path.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
      path.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
    }
    path.poses.back().pose.orientation = goal.pose.orientation;
    return path;
  }
};
}  // namespace aura_navigation

PLUGINLIB_EXPORT_CLASS(aura_navigation::AStarPlanner, nav2_core::GlobalPlanner)
