#include "motion_planning/goal_selector.hpp"

#include "motion_planning/occupancy_grid.hpp"
#include "motion_planning/rrt_tree.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <limits>
#include <stdexcept>

namespace goal_selection
{

Selector::Selector(const double goal_ahead_distance)
    : goal_ahead_distance_(goal_ahead_distance)
{
    if (goal_ahead_distance_ <= 0.0)
    {
        throw std::invalid_argument("goal_ahead_distance must be positive");
    }
}

void Selector::reset()
{
    current_index_.reset();
}

std::optional<std::size_t> Selector::current_index() const
{
    return current_index_;
}

Status Selector::initialize(
    const std::vector<geometry_msgs::msg::Point>& waypoints,
    const geometry_msgs::msg::Pose& vehicle_pose,
    const nav_msgs::msg::OccupancyGrid& collision_map,
    const geometry_msgs::msg::TransformStamped& map_to_vehicle)
{
    if (waypoints.empty())
    {
        return Status::empty_waypoints;
    }

    std::optional<std::size_t> closest_index;
    double closest_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < waypoints.size(); ++i)
    {
        geometry_msgs::msg::PointStamped map_point;
        geometry_msgs::msg::PointStamped vehicle_point;
        map_point.point = waypoints.at(i);
        tf2::doTransform(map_point, vehicle_point, map_to_vehicle);
        if (vehicle_point.point.x < 0.0)
        {
            continue;
        }

        const double distance = rrt_star::squared_distance(
            {waypoints.at(i).x, waypoints.at(i).y},
            {vehicle_pose.position.x, vehicle_pose.position.y});
        if (distance < closest_distance)
        {
            closest_distance = distance;
            closest_index = i;
        }
    }

    if (!closest_index)
    {
        return Status::no_waypoint_ahead;
    }
    if (closest_distance > goal_ahead_distance_ * goal_ahead_distance_)
    {
        return Status::closest_waypoint_too_far;
    }

    current_index_ = *closest_index;
    return select_forward_free_waypoint(waypoints, vehicle_pose, collision_map);
}

Status Selector::select_forward_free_waypoint(
    const std::vector<geometry_msgs::msg::Point>& waypoints,
    const geometry_msgs::msg::Pose& vehicle_pose,
    const nav_msgs::msg::OccupancyGrid& collision_map)
{
    if (waypoints.empty())
    {
        return Status::empty_waypoints;
    }

    std::size_t candidate_index = current_index_.value_or(0) % waypoints.size();
    std::optional<std::size_t> selected_index;
    const double minimum_distance = 0.9 * goal_ahead_distance_;
    const double minimum_distance_squared = minimum_distance * minimum_distance;
    const double maximum_distance_squared =
        goal_ahead_distance_ * goal_ahead_distance_;

    for (std::size_t i = 0; i < waypoints.size(); ++i)
    {
        const auto& waypoint = waypoints.at(candidate_index);
        const double distance_squared = rrt_star::squared_distance(
            {waypoint.x, waypoint.y},
            {vehicle_pose.position.x, vehicle_pose.position.y});

        if (distance_squared > maximum_distance_squared && i > 0)
        {
            break;
        }

        if (distance_squared <= maximum_distance_squared &&
            !occupancy_grid::is_xy_coord_occupied(
                collision_map, waypoint.x, waypoint.y))
        {
            selected_index = candidate_index;
            if (distance_squared >= minimum_distance_squared)
            {
                break;
            }
        }
        candidate_index = (candidate_index + 1) % waypoints.size();
    }

    if (!selected_index)
    {
        return Status::no_free_waypoint_in_range;
    }
    current_index_ = *selected_index;
    return Status::success;
}

Result Selector::update(
    const std::vector<geometry_msgs::msg::Point>& waypoints,
    const geometry_msgs::msg::Pose& vehicle_pose,
    const nav_msgs::msg::OccupancyGrid& collision_map,
    const geometry_msgs::msg::TransformStamped& map_to_vehicle)
{
    const std::optional<std::size_t> previous_index = current_index_;
    Status status = Status::success;
    if (!current_index_ || *current_index_ >= waypoints.size())
    {
        status = initialize(waypoints, vehicle_pose, collision_map, map_to_vehicle);
    }

    if (status == Status::success)
    {
        const auto& goal = waypoints.at(*current_index_);
        const double distance_squared = rrt_star::squared_distance(
            {goal.x, goal.y},
            {vehicle_pose.position.x, vehicle_pose.position.y});
        if (distance_squared > goal_ahead_distance_ * goal_ahead_distance_)
        {
            status = initialize(waypoints, vehicle_pose, collision_map, map_to_vehicle);
        }
        else if (occupancy_grid::is_xy_coord_occupied(
                     collision_map, goal.x, goal.y) ||
                 distance_squared <
                     0.75 * 0.75 * goal_ahead_distance_ * goal_ahead_distance_)
        {
            status = select_forward_free_waypoint(
                waypoints, vehicle_pose, collision_map);
        }
    }

    Result result;
    result.status = status;
    if (status == Status::success)
    {
        result.index = current_index_;
        result.changed = previous_index != current_index_;
    }
    return result;
}

}  // namespace goal_selection
