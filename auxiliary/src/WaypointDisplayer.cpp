/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-06-28 16:24:07
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 16:30:38
 * @FilePath: /auxiliary/src/waypoints_displayer_sim.cpp
 * @Description: Auxiliary node for displaying waypoints in the simulator.
 */

#include "../include/auxiliary/WaypointDisplayer.hpp"
#include "../include/auxiliary/FileHandler.hpp"

#include <stdexcept>

WaypointDisplayer::WaypointDisplayer() : Node("waypoint_displayer_node")
{
    map_frame_ = this->declare_parameter<std::string>("map_frame_name", "map");
    waypoint_file_path_ = this->declare_parameter<std::string>("waypoint_file_path", "");
    r_ = this->declare_parameter<double>("color_r", 1.0);
    g_ = this->declare_parameter<double>("color_g", 0.8);
    b_ = this->declare_parameter<double>("color_b", 0.0);
    publish_interval_s_ =
        this->declare_parameter<double>("publish_interval_second", 5.0);

    if (waypoint_file_path_.empty())
    {
        throw std::invalid_argument("waypoint_file_path cannot be empty.");
    }
    if (publish_interval_s_ <= 0.0)
    {
        throw std::invalid_argument("publish_interval_second must be greater than 0.");
    }

    CSVHandler csv_handler(waypoint_file_path_);
    nav_points_ = csv_handler.read_waypoint_list_from_csv();

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "global_waypoints_display", rclcpp::QoS(1).transient_local().reliable());
    marker_.header.frame_id = map_frame_;
    marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 0.05; marker_.scale.y = 0.1; marker_.scale.z = 0.1;
    marker_.color.r = r_; marker_.color.g = g_; marker_.color.b = b_; marker_.color.a = 0.5;
    marker_.action = visualization_msgs::msg::Marker::MODIFY;
    marker_.id = 0;
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(publish_interval_s_),
        [this]() {return this->on_timer();});
}

void WaypointDisplayer::on_timer()
{
    marker_.points = nav_points_;
    marker_.header.stamp = this->get_clock().get()->now();
    marker_publisher_->publish(marker_);
}
