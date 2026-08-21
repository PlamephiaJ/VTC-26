#ifndef MOTION_PLANNING__SCAN_PROCESSING_HPP_
#define MOTION_PLANNING__SCAN_PROCESSING_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <vector>

namespace scan_processing
{

/**
 * Convert valid planar laser ranges into points in the laser frame.
 *
 * Input: LaserScan message and positive maximum accepted range.
 * Return: Cartesian hit points for finite ranges at or below the limit.
 * NaN, infinity, and farther readings are omitted. The input is not modified.
 */
std::vector<geometry_msgs::msg::Point> valid_hit_points(
    const sensor_msgs::msg::LaserScan& scan, double maximum_range);

}  // namespace scan_processing

#endif  // MOTION_PLANNING__SCAN_PROCESSING_HPP_
