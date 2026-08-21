#include "motion_planning/path_processing.hpp"

#include <cmath>
#include <stdexcept>

namespace path_processing
{

std::vector<geometry_msgs::msg::Point> nodes_to_points(
    const std::vector<rrt_star::Node>& nodes)
{
    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(nodes.size());
    for (const auto& node : nodes)
    {
        geometry_msgs::msg::Point point;
        point.x = node.x;
        point.y = node.y;
        point.z = 0.0;
        points.emplace_back(point);
    }
    return points;
}

std::vector<geometry_msgs::msg::Point> resample_polyline(
    const std::vector<geometry_msgs::msg::Point>& points,
    const double maximum_spacing)
{
    if (maximum_spacing <= 0.0)
    {
        throw std::invalid_argument("maximum_spacing must be positive");
    }
    if (points.size() < 2)
    {
        return points;
    }

    std::vector<geometry_msgs::msg::Point> result;
    for (std::size_t i = 0; i + 1 < points.size(); ++i)
    {
        const auto& start = points.at(i);
        const auto& end = points.at(i + 1);
        result.emplace_back(start);
        const double length = std::hypot(end.x - start.x, end.y - start.y);
        if (length < maximum_spacing)
        {
            continue;
        }

        const int segment_count = static_cast<int>(
            std::ceil(length / maximum_spacing));
        for (int j = 1; j < segment_count; ++j)
        {
            const double ratio = static_cast<double>(j) / segment_count;
            geometry_msgs::msg::Point interpolated;
            interpolated.x = start.x + ratio * (end.x - start.x);
            interpolated.y = start.y + ratio * (end.y - start.y);
            interpolated.z = start.z + ratio * (end.z - start.z);
            result.emplace_back(interpolated);
        }
    }
    result.emplace_back(points.back());
    return result;
}

nav_msgs::msg::Path to_path_message(
    const std::vector<geometry_msgs::msg::Point>& points,
    const std::string& frame_id)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.poses.reserve(points.size());
    for (const auto& point : points)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.pose.position = point;
        path.poses.emplace_back(pose);
    }
    return path;
}

}  // namespace path_processing
