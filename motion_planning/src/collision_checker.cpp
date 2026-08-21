#include "motion_planning/collision_checker.hpp"

#include "motion_planning/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>

namespace collision_checker
{

bool edge_is_blocked(
    const nav_msgs::msg::OccupancyGrid& dynamic_map,
    const nav_msgs::msg::OccupancyGrid& static_map,
    const rrt_star::Node& from, const rrt_star::Node& to,
    const double static_margin, const double dynamic_margin)
{
    if (dynamic_map.info.resolution <= 0.0)
    {
        return true;
    }

    const int x_steps = static_cast<int>(
        std::ceil(std::abs(from.x - to.x) / dynamic_map.info.resolution));
    const int y_steps = static_cast<int>(
        std::ceil(std::abs(from.y - to.y) / dynamic_map.info.resolution));
    const int sample_count = std::max(x_steps, y_steps);
    const double dt = sample_count > 0 ? 1.0 / sample_count : 0.0;

    const bool root_in_dynamic_inflation = from.is_root &&
        occupancy_grid::is_xy_coord_occupied(dynamic_map, from.x, from.y) &&
        !occupancy_grid::is_xy_coord_occupied(static_map, from.x, from.y);
    const double escape_distance =
        std::max(static_margin, dynamic_margin) + dynamic_map.info.resolution;
    const rrt_star::Point2D root{from.x, from.y};

    for (int i = 0; i <= sample_count; ++i)
    {
        const double t = i * dt;
        const double x = from.x + t * (to.x - from.x);
        const double y = from.y + t * (to.y - from.y);
        if (!occupancy_grid::is_xy_coord_occupied(dynamic_map, x, y))
        {
            continue;
        }

        const bool within_escape_distance =
            rrt_star::squared_distance(root, {x, y}) <= escape_distance * escape_distance;
        if (root_in_dynamic_inflation && within_escape_distance &&
            !occupancy_grid::is_xy_coord_occupied(static_map, x, y))
        {
            continue;
        }
        return true;
    }
    return false;
}

}  // namespace collision_checker
