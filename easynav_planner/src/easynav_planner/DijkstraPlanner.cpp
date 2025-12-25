// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Implementation of the DijkstraPlanner class.

#include "easynav_planner/DijkstraPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace easynav
{

std::expected<void, std::string> DijkstraPlanner::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<std::string>(plugin_name + ".map_key", map_key_);
  node->declare_parameter<std::string>(plugin_name + ".path_key", path_key_);
  node->declare_parameter<int>(plugin_name + ".obstacle_threshold", obstacle_threshold_);
  node->declare_parameter<bool>(plugin_name + ".allow_unknown", allow_unknown_);
  node->declare_parameter<bool>(plugin_name + ".allow_diagonal", allow_diagonal_);
  node->declare_parameter<double>(plugin_name + ".cost_weight", cost_weight_);

  node->get_parameter(plugin_name + ".map_key", map_key_);
  node->get_parameter(plugin_name + ".path_key", path_key_);
  node->get_parameter(plugin_name + ".obstacle_threshold", obstacle_threshold_);
  node->get_parameter(plugin_name + ".allow_unknown", allow_unknown_);
  node->get_parameter(plugin_name + ".allow_diagonal", allow_diagonal_);
  node->get_parameter(plugin_name + ".cost_weight", cost_weight_);

  path_.poses.clear();

  return {};
}

void DijkstraPlanner::update(NavState & nav_state)
{
  auto node = get_node();

  if (!nav_state.has("robot_pose")) {
    RCLCPP_WARN(node->get_logger(), "[%s] Missing 'robot_pose' in NavState", get_plugin_name().c_str());
    return;
  }
  if (!nav_state.has("goals")) {
    RCLCPP_WARN(node->get_logger(), "[%s] Missing 'goals' in NavState", get_plugin_name().c_str());
    return;
  }
  if (!nav_state.has(map_key_)) {
    RCLCPP_WARN(
      node->get_logger(), "[%s] Missing map key '%s' in NavState",
      get_plugin_name().c_str(), map_key_.c_str());
    return;
  }

  const auto & robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
  const auto & goals = nav_state.get<nav_msgs::msg::Goals>("goals");
  const auto & map = nav_state.get<nav_msgs::msg::OccupancyGrid>(map_key_);

  if (goals.goals.empty()) {
    RCLCPP_DEBUG(node->get_logger(), "[%s] No goals to plan", get_plugin_name().c_str());
    return;
  }

  const auto & goal_pose = goals.goals.front();

  GridIndex start;
  GridIndex goal;
  if (!world_to_grid(map, robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y, start)) {
    RCLCPP_WARN(node->get_logger(), "[%s] Robot pose outside map bounds", get_plugin_name().c_str());
    return;
  }
  if (!world_to_grid(map, goal_pose.pose.position.x, goal_pose.pose.position.y, goal)) {
    RCLCPP_WARN(node->get_logger(), "[%s] Goal pose outside map bounds", get_plugin_name().c_str());
    return;
  }
  if (start.x == goal.x && start.y == goal.y) {
    RCLCPP_WARN(
      node->get_logger(), "[%s] Goal cell matches the robot's current cell", get_plugin_name().c_str());
    return;
  }

  if (!is_cell_traversable(map, start)) {
    RCLCPP_WARN(node->get_logger(), "[%s] Start cell is not traversable", get_plugin_name().c_str());
    return;
  }
  if (!is_cell_traversable(map, goal)) {
    RCLCPP_WARN(node->get_logger(), "[%s] Goal cell is not traversable", get_plugin_name().c_str());
    return;
  }

  auto path_cells = compute_path(map, start, goal);
  if (path_cells.empty()) {
    RCLCPP_WARN(node->get_logger(), "[%s] Failed to compute a path", get_plugin_name().c_str());
    return;
  }

  path_.header.stamp = node->now();
  path_.header.frame_id = map.header.frame_id.empty() ? (get_tf_prefix() + "map") : map.header.frame_id;
  path_.poses.clear();
  path_.poses.reserve(path_cells.size());

  for (const auto & cell : path_cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_.header;
    pose.pose.position = grid_to_world(map, cell);
    pose.pose.orientation.w = 1.0;
    path_.poses.push_back(pose);
  }

  nav_state.set(path_key_, path_);
}

bool DijkstraPlanner::world_to_grid(
  const nav_msgs::msg::OccupancyGrid & map,
  double wx,
  double wy,
  GridIndex & grid) const
{
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;
  if (map.info.resolution <= 0.0) {
    return false;
  }

  const int x = static_cast<int>(std::floor((wx - origin_x) / map.info.resolution));
  const int y = static_cast<int>(std::floor((wy - origin_y) / map.info.resolution));
  if (x < 0 || y < 0 || x >= static_cast<int>(map.info.width) || y >= static_cast<int>(map.info.height)) {
    return false;
  }

  grid.x = x;
  grid.y = y;
  return true;
}

geometry_msgs::msg::Point DijkstraPlanner::grid_to_world(
  const nav_msgs::msg::OccupancyGrid & map,
  const GridIndex & grid) const
{
  geometry_msgs::msg::Point p;
  p.x = map.info.origin.position.x + (static_cast<double>(grid.x) + 0.5) * map.info.resolution;
  p.y = map.info.origin.position.y + (static_cast<double>(grid.y) + 0.5) * map.info.resolution;
  p.z = map.info.origin.position.z;
  return p;
}

bool DijkstraPlanner::is_cell_traversable(
  const nav_msgs::msg::OccupancyGrid & map,
  const GridIndex & grid) const
{
  const auto width = static_cast<int>(map.info.width);
  const auto height = static_cast<int>(map.info.height);
  if (grid.x < 0 || grid.y < 0 || grid.x >= width || grid.y >= height) {
    return false;
  }

  const std::size_t index = static_cast<std::size_t>(grid.y) * map.info.width + grid.x;
  if (index >= map.data.size()) {
    return false;
  }

  const int8_t value = map.data[index];
  if (value < 0) {
    return allow_unknown_;
  }

  return value < obstacle_threshold_;
}

std::vector<DijkstraPlanner::GridIndex> DijkstraPlanner::compute_path(
  const nav_msgs::msg::OccupancyGrid & map,
  const GridIndex & start,
  const GridIndex & goal) const
{
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const int cell_count = width * height;
  if (cell_count <= 0) {
    return {};
  }
  if (map.data.size() < static_cast<std::size_t>(cell_count)) {
    return {};
  }

  const auto to_index = [width](const GridIndex & cell) {
      return cell.y * width + cell.x;
    };
  const auto to_cell = [width](int idx) {
      GridIndex cell;
      cell.x = idx % width;
      cell.y = idx / width;
      return cell;
    };

  std::vector<double> dist(static_cast<std::size_t>(cell_count), std::numeric_limits<double>::infinity());
  std::vector<int> prev(static_cast<std::size_t>(cell_count), -1);

  using QueueEntry = std::pair<double, int>;
  auto cmp = [](const QueueEntry & a, const QueueEntry & b) { return a.first > b.first; };
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, decltype(cmp)> queue(cmp);

  const int start_idx = to_index(start);
  const int goal_idx = to_index(goal);
  dist[start_idx] = 0.0;
  queue.push({0.0, start_idx});

  const std::vector<GridIndex> neighbors = allow_diagonal_ ? std::vector<GridIndex>{
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  } : std::vector<GridIndex>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  while (!queue.empty()) {
    const auto [cost, idx] = queue.top();
    queue.pop();

    if (idx == goal_idx) {
      break;
    }
    if (cost > dist[idx]) {
      continue;
    }

    const auto cell = to_cell(idx);

    for (const auto & offset : neighbors) {
      GridIndex next{cell.x + offset.x, cell.y + offset.y};
      if (!is_cell_traversable(map, next)) {
        continue;
      }

      const int next_idx = to_index(next);
      const bool diagonal = offset.x != 0 && offset.y != 0;
      const double step_cost = diagonal ? std::sqrt(2.0) : 1.0;

      const std::size_t cost_index = static_cast<std::size_t>(next_idx);
      const int8_t cell_value = map.data[cost_index];
      const double cell_cost = cell_value < 0 ? 1.0 : 1.0 + cost_weight_ * (cell_value / 100.0);

      const double new_cost = cost + step_cost * cell_cost;
      if (new_cost < dist[next_idx]) {
        dist[next_idx] = new_cost;
        prev[next_idx] = idx;
        queue.push({new_cost, next_idx});
      }
    }
  }

  if (start_idx != goal_idx && prev[goal_idx] == -1) {
    return {};
  }

  std::vector<GridIndex> path;
  for (int idx = goal_idx; idx != -1; idx = prev[idx]) {
    path.push_back(to_cell(idx));
    if (idx == start_idx) {
      break;
    }
  }

  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DijkstraPlanner, easynav::PlannerMethodBase)
