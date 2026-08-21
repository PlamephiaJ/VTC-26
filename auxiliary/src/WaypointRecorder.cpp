/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-06-28 16:24:07
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 19:13:01
 * @FilePath: /auxiliary/src/WaypointRecorder.cpp
 * @Description: Auxiliary node for recording the vehicle position in the simulator.
 * This position is collected by using tf2 transformation.
 * The output directory is configured through the output_directory parameter.
 */

#include "../include/auxiliary/WaypointRecorder.hpp"

#include <cstdio>
#include <ctime>
#include <stdexcept>

void WaypointRecorder::on_timer_auto()
{
    std::string target_frame = map_frame_;
    std::string source_frame = vehicle_frame_;

    geometry_msgs::msg::TransformStamped t;

    try
    {
        t = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    }
    catch(const tf2::TransformException & e)
    {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            target_frame.c_str(), source_frame.c_str(), e.what());
        return;
    }

    const auto x = t.transform.translation.x;
    const auto y = t.transform.translation.y;

    if (enable_speed_)
    {
        file_ << x << "," << y << "," << current_speed_ << "\n";
        RCLCPP_INFO(
            this->get_logger(), "Waypoint No.: %llu, logging: %f, %f, %f.",
            waypoint_num_, x, y, current_speed_);
    }
    else
    {
        file_ << x << "," << y << "\n";
        RCLCPP_INFO(
            this->get_logger(), "Waypoint No.: %llu, logging: %f, %f",
            waypoint_num_, x, y);
    }

    waypoint_num_++;
    marker_.id++;
    marker_.header.stamp = this->get_clock().get()->now();
    marker_.pose.position.x = x; marker_.pose.position.y = y; marker_.pose.position.z = 0;
    marker_.pose.orientation.x = 0; marker_.pose.orientation.y = 0; marker_.pose.orientation.z = 0; marker_.pose.orientation.w = 1;
    marker_publisher_->publish(marker_);
}

void WaypointRecorder::on_timer_manual()
{
    char c;
    if (read(STDIN_FILENO, &c, 1) == 1 && c == c_input_)
    {
        std::string target_frame = map_frame_;
        std::string source_frame = vehicle_frame_;

        geometry_msgs::msg::TransformStamped t;

        try
        {
            t = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        }
        catch(const tf2::TransformException & e)
        {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", target_frame.c_str(), source_frame.c_str(), e.what());
            return;
        }

        const auto x = t.transform.translation.x;
        const auto y = t.transform.translation.y;
        
        if (enable_speed_)
        {
            file_ << x << "," << y << "," << current_speed_ << "\n";
            RCLCPP_INFO(
                this->get_logger(), "Waypoint No.: %llu, logging: %f, %f, %f.",
                waypoint_num_, x, y, current_speed_);
        }
        else
        {
            file_ << x << "," << y << "\n";
            RCLCPP_INFO(
                this->get_logger(), "Waypoint No.: %llu, logging: %f, %f",
                waypoint_num_, x, y);
        }

        waypoint_num_++;
        marker_.id++;
        marker_.header.stamp = this->get_clock().get()->now();
        marker_.pose.position.x = x; marker_.pose.position.y = y; marker_.pose.position.z = 0;
        marker_.pose.orientation.x = 0; marker_.pose.orientation.y = 0; marker_.pose.orientation.z = 0; marker_.pose.orientation.w = 1;
        marker_publisher_->publish(marker_);
    }
}

std::string WaypointRecorder::generate_waypoint_file_path()
{
    time_t now = time(nullptr);
    char* curr_time = ctime(&now);

    std::string s = std::to_string(sample_interval_s_);
    s = s.substr(0, s.find('.') + 2);

    std::string time_string = std::string(curr_time).substr(0, 24);
    for (auto &c : time_string)
    {
        if (c == ' ')
        {
            c = '_';
        }
    }

    std::string path = output_directory_ + "/waypoints_" + s + "s_" + time_string + ".csv";
    RCLCPP_INFO_STREAM(this->get_logger(), path);
    return path;
}

void WaypointRecorder::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    current_speed_ = odom_msg->twist.twist.linear.x;
}

WaypointRecorder::WaypointRecorder() : Node("waypoint_recorder_node")
{
    mode_ = this->declare_parameter<std::string>("mode", "auto");
    map_frame_ = this->declare_parameter<std::string>("map_frame_name", "map");
    sample_interval_s_ = this->declare_parameter<double>("sample_interval_second", 0.1);
    const auto manual_record_key =
        this->declare_parameter<std::string>("manual_record_key", "r");
    enable_speed_ = this->declare_parameter<bool>("enable_speed", true);
    output_directory_ =
        this->declare_parameter<std::string>("output_directory", "/sim_ws/waypoint");
    vehicle_frame_ = this->declare_parameter<std::string>("vehicle_frame_name", "");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "");

    if (!(mode_ == "auto" || mode_ == "manual"))
    {
        throw std::invalid_argument(
            "mode must be \"auto\" or \"manual\". Current value: " + mode_);
    }

    if (mode_ == "auto" && sample_interval_s_ <= 0.0)
    {
        throw std::invalid_argument(
            "sample_interval_second must be greater than 0 in auto mode.");
    }

    if (mode_ == "manual")
    {
        if (manual_record_key.empty())
        {
            throw std::invalid_argument("manual_record_key cannot be empty in manual mode.");
        }
        c_input_ = manual_record_key[0];
    }

    if (output_directory_.empty())
    {
        throw std::invalid_argument("output_directory cannot be empty.");
    }
    if (output_directory_.size() > 1 && output_directory_.back() == '/')
    {
        output_directory_.pop_back();
    }

    const auto node_namespace = std::string(this->get_namespace());
    const auto namespaced_prefix =
        node_namespace == "/" ? "" : node_namespace.substr(1) + "/";
    if (vehicle_frame_.empty())
    {
        vehicle_frame_ = mode_ == "auto" ? namespaced_prefix + "base_link" : "car0/base_link";
    }
    if (odom_topic_.empty())
    {
        odom_topic_ = mode_ == "auto" ? "odom" : "/car0/odom";
    }

    const auto interval = std::chrono::duration<double>(sample_interval_s_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    file_path_ = generate_waypoint_file_path();
    tmpfilename_ = output_directory_ + "/temp.csv";

    std::ifstream file(tmpfilename_);
    if (file.good())
    {
        if (remove(tmpfilename_.c_str()) != 0)
        {
            throw std::runtime_error("Couldn't remove existing temp file: " + tmpfilename_);
        }
    }
    file.close();

    file_.open(tmpfilename_);

    if (!file_.is_open())
    {
        throw std::runtime_error(
            "Failed to open the temp csv file. Check output_directory: " + output_directory_);
    }

    std::vector<std::string> column_names = {"x", "y"};
    if (enable_speed_)
    {
        column_names.emplace_back("speed");
    }

    for (int i = 0; i < (int)column_names.size(); i++)
    {
        file_ << column_names[i];
        if (i < (int)column_names.size() - 1)
        {
            file_ << ",";
        }
    }
    file_ << "\n";

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "waypoint_marker", rclcpp::QoS(1000).transient_local().reliable());
    subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 1, std::bind(&WaypointRecorder::odom_callback, this, std::placeholders::_1));

    marker_.header.frame_id = map_frame_;
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::SPHERE;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.scale.x = 0.2; marker_.scale.y = 0.2; marker_.scale.z = 0.2;
    marker_.color.r = 1.0; marker_.color.g = 0.0; marker_.color.b = 0.0; marker_.color.a = 0.5;
    marker_.action = visualization_msgs::msg::Marker::MODIFY;

    if (mode_ == "auto")
    {
        timer_ = this->create_wall_timer(interval, [this]() {return this->on_timer_auto();});
    }
    else if (mode_ == "manual")
    {
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &orig_termios_) != 0)
        {
            throw std::runtime_error("manual mode requires an interactive terminal.");
        }
        struct termios new_termios = orig_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        if (tcsetattr(STDIN_FILENO, TCSANOW, &new_termios) != 0)
        {
            throw std::runtime_error("Couldn't configure the terminal for manual mode.");
        }
        terminal_configured_ = true;
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&WaypointRecorder::on_timer_manual, this));
    }
}

WaypointRecorder::~WaypointRecorder()
{
    if (terminal_configured_)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    }
    file_.close();

    RCLCPP_INFO(this->get_logger(), "Removing repeating element...");

    std::ifstream input_file(tmpfilename_);
    if (!input_file.is_open())
    {
        RCLCPP_INFO(this->get_logger(), "Couldn't open the temp file.");
        return;
    }

    std::ofstream output_file(file_path_);
    if (!output_file.is_open())
    {
        RCLCPP_INFO(this->get_logger(), "Couldn't open the output file.");
        input_file.close();
        return;
    }

    std::unordered_set<std::string> unique_lines;
    std::string line;
    while (std::getline(input_file, line))
    {
        if (unique_lines.insert(line).second)
        {
            output_file << line << "\n";
        }
    }
    output_file.close();

    if (remove(tmpfilename_.c_str()) != 0)
    {
        RCLCPP_INFO(this->get_logger(), "Couldn't remove the temp file. Please do it manually.");
    }

    input_file.close();
    RCLCPP_INFO(this->get_logger(), "file has been saved to: %s", file_path_.c_str());
}
