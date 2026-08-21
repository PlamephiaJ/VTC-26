#ifndef MOTION_PLANNING__DYNAMIC_OBSTACLE_MAP_HPP_
#define MOTION_PLANNING__DYNAMIC_OBSTACLE_MAP_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <vector>

namespace dynamic_obstacles
{

/**
 * Owns the immutable base map and the mutable live-obstacle collision layer.
 *
 * ROS callbacks provide already-transformed hit points. This class performs
 * only occupancy-map mutation and can therefore be tested independently.
 */
class MapLayer
{
public:
    /**
     * Initialize from a newly received base map.
     *
     * Input: source occupancy grid and non-negative static inflation margin.
     * Operation: stores the source map, creates an inflated collision map, and
     * clears all previously recorded live obstacles.
     */
    void initialize(
        const nav_msgs::msg::OccupancyGrid& map, double static_margin);

    /** Return true after initialize() has supplied a usable map. */
    bool initialized() const;

    /** Return the unmodified source map. Throws if not initialized. */
    const nav_msgs::msg::OccupancyGrid& base_map() const;

    /** Return the inflated map including current live obstacles. */
    const nav_msgs::msg::OccupancyGrid& collision_map() const;

    /**
     * Insert one observed obstacle.
     *
     * Input: hit point in map coordinates and non-negative inflation margin.
     * Operation: if the base map cell is free, marks/inflates the corresponding
     * collision-map cell and records changed indices for later restoration.
     * Return: number of collision-map cells newly changed by this observation.
     */
    std::size_t add_observation(
        const geometry_msgs::msg::Point& map_point, double inflation_margin);

    /**
     * Remove every live obstacle added since the previous clear.
     *
     * Operation: restores recorded cells from the statically inflated map and
     * empties the change list. Static obstacles remain unchanged.
     */
    void clear_observations();

private:
    nav_msgs::msg::OccupancyGrid base_map_;
    nav_msgs::msg::OccupancyGrid collision_map_;
    nav_msgs::msg::OccupancyGrid static_collision_map_;
    std::vector<int> changed_indices_;
    bool initialized_ = false;
};

}  // namespace dynamic_obstacles

#endif  // MOTION_PLANNING__DYNAMIC_OBSTACLE_MAP_HPP_
