#ifndef MOTION_PLANNING__GOAL_SELECTOR_HPP_
#define MOTION_PLANNING__GOAL_SELECTOR_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace goal_selection
{

/** Result category for global-waypoint goal selection. */
enum class Status
{
    success,
    empty_waypoints,
    no_waypoint_ahead,
    closest_waypoint_too_far,
    no_free_waypoint_in_range,
};

/** Output from one goal-selection update. */
struct Result
{
    Status status = Status::empty_waypoints;
    std::optional<std::size_t> index;
    bool changed = false;
};

/**
 * Stateful selector for the rolling global waypoint used as the RRT* goal.
 *
 * The selector owns only the current index. Waypoints, pose, occupancy map,
 * and transform are supplied on each call and are never modified.
 */
class Selector
{
public:
    /**
     * Input: positive maximum goal-ahead distance in map units.
     * Effect: creates an uninitialized selector.
     */
    explicit Selector(double goal_ahead_distance);

    /** Forget the current goal so the next update performs initialization. */
    void reset();

    /**
     * Select or advance the current planning goal.
     *
     * Input:
     * - ordered, cyclic global waypoints in the map frame;
     * - current vehicle pose in the map frame;
     * - current collision map;
     * - transform from map frame to vehicle frame, used to reject waypoints
     *   behind the vehicle during initialization.
     *
     * Return: status, selected index on success, and whether the index changed.
     * The selector reinitializes if the goal becomes too far, and advances if
     * it becomes occupied or closer than 75% of the configured range.
     */
    Result update(
        const std::vector<geometry_msgs::msg::Point>& waypoints,
        const geometry_msgs::msg::Pose& vehicle_pose,
        const nav_msgs::msg::OccupancyGrid& collision_map,
        const geometry_msgs::msg::TransformStamped& map_to_vehicle);

    /** Return the current goal index, or std::nullopt before initialization. */
    std::optional<std::size_t> current_index() const;

private:
    Status initialize(
        const std::vector<geometry_msgs::msg::Point>& waypoints,
        const geometry_msgs::msg::Pose& vehicle_pose,
        const nav_msgs::msg::OccupancyGrid& collision_map,
        const geometry_msgs::msg::TransformStamped& map_to_vehicle);

    Status select_forward_free_waypoint(
        const std::vector<geometry_msgs::msg::Point>& waypoints,
        const geometry_msgs::msg::Pose& vehicle_pose,
        const nav_msgs::msg::OccupancyGrid& collision_map);

    double goal_ahead_distance_;
    std::optional<std::size_t> current_index_;
};

}  // namespace goal_selection

#endif  // MOTION_PLANNING__GOAL_SELECTOR_HPP_
