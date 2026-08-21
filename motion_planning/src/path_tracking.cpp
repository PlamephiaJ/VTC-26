#include "motion_planning/path_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace path_tracking
{

std::optional<geometry_msgs::msg::Point> point_at_distance(
    const nav_msgs::msg::Path& path, const double distance)
{
    if (path.poses.empty())
    {
        return std::nullopt;
    }

    if (distance <= 0.0)
    {
        return path.poses.front().pose.position;
    }

    double accumulated_distance = 0.0;
    for (std::size_t i = 1; i < path.poses.size(); ++i)
    {
        const auto& segment_start = path.poses.at(i - 1).pose.position;
        const auto& segment_end = path.poses.at(i).pose.position;
        const double segment_length = std::hypot(
            segment_end.x - segment_start.x,
            segment_end.y - segment_start.y);

        if (segment_length <= std::numeric_limits<double>::epsilon())
        {
            continue;
        }

        if (accumulated_distance + segment_length >= distance)
        {
            const double ratio = std::clamp(
                (distance - accumulated_distance) / segment_length, 0.0, 1.0);
            geometry_msgs::msg::Point target;
            target.x = segment_start.x + ratio * (segment_end.x - segment_start.x);
            target.y = segment_start.y + ratio * (segment_end.y - segment_start.y);
            target.z = segment_start.z + ratio * (segment_end.z - segment_start.z);
            return target;
        }

        accumulated_distance += segment_length;
    }

    return path.poses.back().pose.position;
}

double pure_pursuit_steering(
    const double target_lateral_offset, const double lookahead_distance,
    const double proportional_gain, const double steering_limit)
{
    if (lookahead_distance <= 0.0 || proportional_gain <= 0.0 ||
        steering_limit <= 0.0)
    {
        throw std::invalid_argument(
            "lookahead, gain, and steering limit must be positive");
    }
    const double steering = proportional_gain * 2.0 * target_lateral_offset /
        (lookahead_distance * lookahead_distance);
    return std::clamp(steering, -steering_limit, steering_limit);
}

double speed_for_steering(
    const double steering_angle, const SpeedProfile& profile)
{
    constexpr double pi = 3.14159265358979323846;
    const double degrees = std::abs(steering_angle) * 180.0 / pi;
    if (degrees <= profile.low_steering_threshold_degrees)
    {
        return profile.straight_speed;
    }
    if (degrees <= profile.medium_steering_threshold_degrees)
    {
        return profile.medium_turn_speed;
    }
    return profile.sharp_turn_speed;
}

}  // namespace path_tracking
