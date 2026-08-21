#include "motion_planning/scan_processing.hpp"

#include <cmath>
#include <stdexcept>

namespace scan_processing
{

std::vector<geometry_msgs::msg::Point> valid_hit_points(
    const sensor_msgs::msg::LaserScan& scan, const double maximum_range)
{
    if (maximum_range <= 0.0)
    {
        throw std::invalid_argument("maximum_range must be positive");
    }

    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(scan.ranges.size());
    for (std::size_t i = 0; i < scan.ranges.size(); ++i)
    {
        const double range = scan.ranges.at(i);
        if (!std::isfinite(range) || range > maximum_range)
        {
            continue;
        }
        const double angle = scan.angle_min + scan.angle_increment * i;
        geometry_msgs::msg::Point point;
        point.x = range * std::cos(angle);
        point.y = range * std::sin(angle);
        points.emplace_back(point);
    }
    return points;
}

}  // namespace scan_processing
