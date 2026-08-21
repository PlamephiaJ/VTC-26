#include "motion_planning/dynamic_obstacle_map.hpp"

#include "motion_planning/occupancy_grid.hpp"

#include <stdexcept>

namespace dynamic_obstacles
{

void MapLayer::initialize(
    const nav_msgs::msg::OccupancyGrid& map, const double static_margin)
{
    if (static_margin < 0.0)
    {
        throw std::invalid_argument("static_margin must be non-negative");
    }
    base_map_ = map;
    static_collision_map_ = map;
    occupancy_grid::inflate_map(static_collision_map_, static_margin);
    collision_map_ = static_collision_map_;
    changed_indices_.clear();
    initialized_ = true;
}

bool MapLayer::initialized() const
{
    return initialized_;
}

const nav_msgs::msg::OccupancyGrid& MapLayer::base_map() const
{
    if (!initialized_)
    {
        throw std::logic_error("MapLayer is not initialized");
    }
    return base_map_;
}

const nav_msgs::msg::OccupancyGrid& MapLayer::collision_map() const
{
    if (!initialized_)
    {
        throw std::logic_error("MapLayer is not initialized");
    }
    return collision_map_;
}

std::size_t MapLayer::add_observation(
    const geometry_msgs::msg::Point& map_point,
    const double inflation_margin)
{
    if (!initialized_)
    {
        throw std::logic_error("MapLayer is not initialized");
    }
    if (inflation_margin < 0.0)
    {
        throw std::invalid_argument("inflation_margin must be non-negative");
    }
    if (occupancy_grid::is_xy_coord_occupied(
            base_map_, map_point.x, map_point.y))
    {
        return 0;
    }

    const int center_index = occupancy_grid::xy_coord_to_array_index(
        collision_map_, map_point.x, map_point.y);
    const std::vector<int> changes = occupancy_grid::inflate_cell(
        collision_map_, center_index, inflation_margin, 100);
    changed_indices_.insert(
        changed_indices_.end(), changes.begin(), changes.end());
    return changes.size();
}

void MapLayer::clear_observations()
{
    if (!initialized_)
    {
        return;
    }
    for (const int index : changed_indices_)
    {
        if (index >= 0 && index < static_cast<int>(collision_map_.data.size()) &&
            index < static_cast<int>(static_collision_map_.data.size()))
        {
            collision_map_.data.at(index) = static_collision_map_.data.at(index);
        }
    }
    changed_indices_.clear();
}

}  // namespace dynamic_obstacles
