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
/// \brief Declaration of the DijkstraPlanner class.

#ifndef EASYNAV_PLANNER__DIJKSTRAPLANNER_HPP_
#define EASYNAV_PLANNER__DIJKSTRAPLANNER_HPP_

#include <expected>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "easynav_core/PlannerMethodBase.hpp"

namespace easynav
{

/**
 * @class DijkstraPlanner
 * @brief Grid-based Dijkstra path planner for occupancy grids.
 */
class DijkstraPlanner : public easynav::PlannerMethodBase
{
public:
  /// @brief Default constructor.
  DijkstraPlanner() = default;

  /// @brief Destructor.
  ~DijkstraPlanner() = default;

  /**
   * @brief Initialization hook.
   * @return Success or error message.
   */
  std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Run the planning algorithm and update the path.
   * @param nav_state Current navigation state.
   */
  void update(NavState & nav_state) override;

private:
  struct GridIndex
  {
    int x {0};
    int y {0};
  };

  [[nodiscard]] bool world_to_grid(
    const nav_msgs::msg::OccupancyGrid & map,
    double wx,
    double wy,
    GridIndex & grid) const;

  [[nodiscard]] geometry_msgs::msg::Point grid_to_world(
    const nav_msgs::msg::OccupancyGrid & map,
    const GridIndex & grid) const;

  [[nodiscard]] bool is_cell_traversable(
    const nav_msgs::msg::OccupancyGrid & map,
    const GridIndex & grid) const;

  [[nodiscard]] std::vector<GridIndex> compute_path(
    const nav_msgs::msg::OccupancyGrid & map,
    const GridIndex & start,
    const GridIndex & goal) const;

  std::string map_key_ {"map"};
  std::string path_key_ {"path"};
  int obstacle_threshold_ {50};
  bool allow_unknown_ {false};
  bool allow_diagonal_ {true};
  double cost_weight_ {1.0};

  nav_msgs::msg::Path path_;
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__DIJKSTRAPLANNER_HPP_
