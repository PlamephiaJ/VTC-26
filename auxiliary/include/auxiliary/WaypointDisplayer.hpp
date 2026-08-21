/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-07-01 16:10:33
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 16:28:24
 * @FilePath: /auxiliary/include/auxiliary/WaypointDisplayer.hpp
 * @Description: Waypoint displayer header file.
 */

#ifndef WAYPOINT_DISPLAYER_HPP
#define WAYPOINT_DISPLAYER_HPP

#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class WaypointDisplayer : public rclcpp::Node {
private:
    std::string waypoint_file_path_;
    std::string map_frame_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    visualization_msgs::msg::Marker marker_;

    std::vector<geometry_msgs::msg::Point> nav_points_;

    rclcpp::TimerBase::SharedPtr timer_{nullptr};

    double r_ = 1.0;
    double g_ = 0.8;
    double b_ = 0.0;
    double publish_interval_s_ = 5.0;

    void on_timer();

public:
    WaypointDisplayer();
};

#endif
