# Dijkstra Planner (EasyNav)

This package provides a **grid-based Dijkstra planner** plugin for EasyNavigation. The planner computes a
`nav_msgs/Path` over a `nav_msgs/OccupancyGrid` using Dijkstra’s algorithm and publishes the result back
into the EasyNav `NavState`.

## Features

- Reads the robot pose, goal list, and occupancy grid from `NavState`.
- Produces a `nav_msgs/Path` in the map frame.
- Supports diagonal motion and unknown-cell handling.
- Cost-aware: cells are weighted by occupancy values.

## Plugin

**Pluginlib class**: `easynav_planner/DijkstraPlanner`

The plugin is registered in `easynav_planner_plugins.xml` and can be loaded via the EasyNav configuration.

## Parameters

All parameters are **namespaced by the plugin name** in your EasyNav configuration.

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `map_key` | `string` | `"map"` | NavState key that holds the `nav_msgs/OccupancyGrid`. |
| `path_key` | `string` | `"path"` | NavState key where the computed `nav_msgs/Path` is stored. |
| `obstacle_threshold` | `int` | `50` | Cells with values >= this threshold are treated as obstacles. |
| `allow_unknown` | `bool` | `false` | If true, unknown cells (`-1`) are traversable. |
| `allow_diagonal` | `bool` | `true` | If true, diagonal neighbor moves are allowed. |
| `cost_weight` | `double` | `1.0` | Weight applied to occupancy costs when computing path cost. |

## Example Configuration

```yaml
planner:
  plugin: "easynav_planner/DijkstraPlanner"
  DijkstraPlanner:
    map_key: "map"
    path_key: "path"
    obstacle_threshold: 50
    allow_unknown: false
    allow_diagonal: true
    cost_weight: 1.0
```

## Inputs / Outputs

**Inputs (NavState keys)**

- `robot_pose` (`nav_msgs/Odometry`)
- `goals` (`nav_msgs/Goals`)
- `map_key` (`nav_msgs/OccupancyGrid`)

**Outputs (NavState keys)**

- `path_key` (`nav_msgs/Path`)

## Notes

- The planner uses the **first goal** in `nav_msgs/Goals`.
- If start/goal is outside the map or non-traversable, no path is produced.
