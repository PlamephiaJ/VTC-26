#ifndef MOTION_PLANNING__COLLISION_CHECKER_HPP_
#define MOTION_PLANNING__COLLISION_CHECKER_HPP_

#include "motion_planning/rrt_tree.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace collision_checker
{

/**
 * Test a complete tree edge against an occupancy grid.
 *
 * Input:
 * - `dynamic_map`: static map plus inflated live obstacles.
 * - `static_map`: unmodified map, used to distinguish live-obstacle inflation.
 * - `from`/`to`: edge endpoints in the map frame.
 * - margins: configured static and live-obstacle inflation radii.
 *
 * Return: true when the edge is blocked or the map is invalid/out of bounds;
 * false when every sampled grid cell is traversable. A root already inside a
 * live-obstacle inflation bubble may travel outward for one inflation radius,
 * matching the original planner's escape behavior.
 */
bool edge_is_blocked(
    const nav_msgs::msg::OccupancyGrid& dynamic_map,
    const nav_msgs::msg::OccupancyGrid& static_map,
    const rrt_star::Node& from, const rrt_star::Node& to,
    double static_margin, double dynamic_margin);

}  // namespace collision_checker

#endif  // MOTION_PLANNING__COLLISION_CHECKER_HPP_
