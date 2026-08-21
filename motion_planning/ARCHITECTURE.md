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
| Optimal trajectory arc length, progress projection, sampling, slicing | `include/motion_planning/optimal_trajectory.hpp` | `src/optimal_trajectory.cpp` |
| Optimal-reference/RRT-detour mode switching and safe rejoin | `include/motion_planning/reference_path_manager.hpp` | `src/reference_path_manager.cpp` |
| Local-path conversion/resampling, arc-length lookahead, Pure Pursuit, speed | `include/motion_planning/path_tracking.hpp` | `src/path_tracking.cpp` |
| LaserScan filtering plus static/live obstacle-layer lifetime | `include/motion_planning/dynamic_obstacle_map.hpp` | `src/dynamic_obstacle_map.cpp` |
| Occupancy coordinates, inflation, segment/polyline collision, root escape | `include/motion_planning/occupancy_grid.hpp` | `src/occupancy_grid.cpp` |
| Waypoint CSV input | `include/motion_planning/FileHandler.hpp` | `src/FileHandler.cpp` |
| RViz marker wrappers | `include/motion_planning/Visualization.hpp` | `src/Visualization.cpp` |

## One odometry-cycle data flow

1. `RRT::odom_callback()` records the current pose and refreshes TF.
2. `reference_path::Manager::update()` projects trajectory progress and creates
   the forward global goal plus local optimal reference.
3. If the optimal arc is clear, the local optimal reference is used directly.
4. If it is blocked, `rrt_star::Planner::plan()` builds a local detour to the
   same progress-based global goal.
5. `path_tracking` selects the short arc-length lookahead target and computes
   steering/speed on whichever local reference is active.
6. `RRT` publishes the path/tree markers and Ackermann command.

The optimal trajectory precomputes segment and cumulative arc lengths once.
After the first global projection, progress updates search only the configured
backward/forward arc window. A global projection is retried only when the local
projection is farther than `PROJECTION_FALLBACK_DISTANCE`.

While in RRT-detour mode, returning to the optimal reference requires all three
conditions in the same update:

- the optimal arc from current progress to the global goal is collision-free;
- distance from the vehicle to the optimal projection is no greater than
  `OPTIMAL_REJOIN_DISTANCE`;
- the direct short connector from the vehicle to that projection is clear.

Laser scans follow a separate short flow:

1. `dynamic_obstacles::valid_hit_points()` filters ranges and creates
   laser-frame hit points.
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
