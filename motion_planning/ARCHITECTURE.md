# Motion-planning module guide

`RRT` is the ROS2 orchestration layer. Planning and control algorithms live in
small modules so they can be changed and tested without constructing a ROS
node. Public headers contain the detailed input, output, mutation, and boundary
contracts for every interface.

## Where to make a change

| Optimization target | Public interface | Implementation |
|---|---|---|
| RRT node callbacks, parameters, TF, publishers | `include/motion_planning/RRT.hpp` | `src/RRT.cpp` |
| RRT node/tree geometry, nearest, steer, near, reparent, path tracing | `include/motion_planning/rrt_tree.hpp` | `src/rrt_tree.cpp` |
| RRT* sampling and planning loop | `include/motion_planning/rrt_star_planner.hpp` | `src/rrt_star_planner.cpp` |
| Edge collision checking and root escape | `include/motion_planning/collision_checker.hpp` | `src/collision_checker.cpp` |
| Rolling global-waypoint goal selection | `include/motion_planning/goal_selector.hpp` | `src/goal_selector.cpp` |
| Node-path conversion and waypoint resampling | `include/motion_planning/path_processing.hpp` | `src/path_processing.cpp` |
| Arc-length lookahead, Pure Pursuit, speed policy | `include/motion_planning/path_tracking.hpp` | `src/path_tracking.cpp` |
| LaserScan range filtering and polar conversion | `include/motion_planning/scan_processing.hpp` | `src/scan_processing.cpp` |
| Static inflation and live-obstacle lifetime | `include/motion_planning/dynamic_obstacle_map.hpp` | `src/dynamic_obstacle_map.cpp` |
| Occupancy-grid coordinate and inflation primitives | `include/motion_planning/occupancy_grid.hpp` | `src/occupancy_grid.cpp` |
| Waypoint CSV input | `include/motion_planning/FileHandler.hpp` | `src/FileHandler.cpp` |
| RViz marker wrappers | `include/motion_planning/Visualization.hpp` | `src/Visualization.cpp` |

## One odometry-cycle data flow

1. `RRT::odom_callback()` records the current pose and refreshes TF.
2. `goal_selection::Selector::update()` chooses a free forward global waypoint.
3. `rrt_star::Planner::plan()` builds and rewires a local RRT* tree.
4. `path_processing` converts and resamples the selected tree path.
5. `path_tracking` selects the arc-length target and computes steering/speed.
6. `RRT` publishes the path/tree markers and Ackermann command.

Laser scans follow a separate short flow:

1. `scan_processing` filters ranges and creates laser-frame hit points.
2. `RRT` applies TF because frame lookup is a ROS responsibility.
3. `dynamic_obstacles::MapLayer` inserts and periodically clears observations.

## Interface contract convention

Each public declaration documents:

- **Input**: coordinate frame, units, required ranges, and ownership;
- **Return**: meaning of the value and failure representation;
- **Operation**: state or argument mutation, when applicable;
- **Boundary behavior**: invalid maps, empty paths, out-of-range coordinates,
  short paths, and rejected samples.

Algorithm modules do not publish ROS messages, query TF, or log. Those effects
remain in `RRT`, which makes module tests deterministic and keeps optimization
work localized.
