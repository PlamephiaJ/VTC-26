#ifndef MOTION_PLANNING__PATH_TRACKING_HPP_
#define MOTION_PLANNING__PATH_TRACKING_HPP_

#include <optional>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"

namespace path_tracking
{

/** Piecewise speed policy used after Pure Pursuit steering is computed. */
struct SpeedProfile
{
    double low_steering_threshold_degrees = 10.0;
    double medium_steering_threshold_degrees = 20.0;
    double straight_speed = 5.0;
    double medium_turn_speed = 3.0;
    double sharp_turn_speed = 1.5;
};

/**
 * Return the point at the requested planar arc length from the start of a path.
 *
 * The result is linearly interpolated within the segment containing the
 * requested distance. Distances at or below zero select the first path point,
 * and distances beyond the path length select the last path point.
 */
std::optional<geometry_msgs::msg::Point> point_at_distance(
    const nav_msgs::msg::Path& path, double distance);

/**
 * Compute and clamp a Pure Pursuit steering command.
 *
 * Input: target lateral coordinate in the vehicle frame, positive lookahead
 * distance, positive proportional gain, and positive steering limit.
 * Return: signed steering angle in radians within [-limit, limit]. Throws
 * std::invalid_argument when a required scalar is non-positive.
 */
double pure_pursuit_steering(
    double target_lateral_offset, double lookahead_distance,
    double proportional_gain, double steering_limit);

/**
 * Map steering magnitude to a commanded speed.
 *
 * Input: steering angle in radians and an ordered piecewise speed profile.
 * Return: configured straight, medium-turn, or sharp-turn speed.
 */
double speed_for_steering(
    double steering_angle, const SpeedProfile& profile = SpeedProfile{});

}  // namespace path_tracking

#endif  // MOTION_PLANNING__PATH_TRACKING_HPP_
