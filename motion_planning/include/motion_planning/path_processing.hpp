#ifndef MOTION_PLANNING__PATH_PROCESSING_HPP_
#define MOTION_PLANNING__PATH_PROCESSING_HPP_

#include "motion_planning/rrt_tree.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"

#include <string>
#include <vector>

namespace path_processing
{

/**
 * Convert an RRT node path to ROS map points.
 *
 * Input: nodes ordered from path start to path end.
 * Return: points in the same order, with z set to zero.
 */
std::vector<geometry_msgs::msg::Point> nodes_to_points(
    const std::vector<rrt_star::Node>& nodes);

/**
 * Resample a polyline so no output segment exceeds `maximum_spacing`.
 *
 * Input: ordered points and a positive maximum spacing.
 * Return: the same geometric polyline with evenly interpolated points added;
 * original endpoints are retained. Empty and one-point inputs are returned
 * unchanged. Throws std::invalid_argument for non-positive spacing.
 */
std::vector<geometry_msgs::msg::Point> resample_polyline(
    const std::vector<geometry_msgs::msg::Point>& points,
    double maximum_spacing);

/**
 * Package points as a nav_msgs/Path.
 *
 * Input: ordered points and their coordinate-frame name.
 * Return: path containing one PoseStamped per point; orientations remain at
 * their default because the controller consumes positions only.
 */
nav_msgs::msg::Path to_path_message(
    const std::vector<geometry_msgs::msg::Point>& points,
    const std::string& frame_id);

}  // namespace path_processing

#endif  // MOTION_PLANNING__PATH_PROCESSING_HPP_
