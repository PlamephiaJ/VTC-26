/*
 * @Author: Yuhao Chen
 * @Date: 2024-06-08 09:54:00
 * @LastEditors: Yuhao Chen
 * @LastEditTime: 2024-07-01 19:15:33
 * @Description: RRT* algorithm for F1Tenth source file.
 */

#include "../include/motion_planning/RRT.hpp"
#include "../include/motion_planning/occupancy_grid.hpp"
#include "../include/motion_planning/FileHandler.hpp"

void RRT::init_marker()
{
    marker_point_.header.frame_id = marker_frame_id_;
    marker_point_.type = visualization_msgs::msg::Marker::SPHERE;
    marker_point_.scale.x = 0.3;
    marker_point_.scale.y = 0.3;
    marker_point_.scale.z = 0.3;
    marker_point_.color.r = 0.0;
    marker_point_.color.g = 0.0;
    marker_point_.color.b = 0.0;
    marker_point_.color.a = 1.0;

    marker_line_.header.frame_id = marker_frame_id_;
    marker_line_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_line_.pose.orientation.w = 1.0;
    marker_line_.scale.x = 0.05;
    marker_line_.color.r = 0.0f;
    marker_line_.color.g = 0.0f;
    marker_line_.color.b = 0.0f;
    marker_line_.color.a = 1.0f;
}

void RRT::draw_point(const geometry_msgs::msg::Point & point, double time_to_live, int color)
{
    marker_point_.id = unique_point_id_;
    marker_point_.action = visualization_msgs::msg::Marker::ADD;
    marker_point_.header.stamp = this->get_clock().get()->now();
    marker_point_.pose.position = point;
    if (color == RED)
    {
        marker_point_.color.r += 1.0;
    }
    else if (color == GREEN)
    {
        marker_point_.color.g += 1.0;
    }
    else if (color == BLUE)
    {
        marker_point_.color.b += 1.0;
    }

    if (time_to_live < 0)
    {
        publisher_marker_->publish(marker_point_);
    }
    else
    {
        marker_point_.lifetime = rclcpp::Duration::from_seconds(time_to_live);
        publisher_marker_->publish(marker_point_);
        marker_point_.lifetime = rclcpp::Duration::from_seconds(0.0);
    }
    if (unique_point_id_ >= INT32_MAX) unique_point_id_ = 0;
    else unique_point_id_++;
    marker_point_.color.r = marker_point_.color.g = marker_point_.color.b = 0.0f;
}

void RRT::draw_line(const std::vector<geometry_msgs::msg::Point> & points, double time_to_live, int color)
{
    marker_line_.id = unique_line_id_;
    marker_line_.action = visualization_msgs::msg::Marker::ADD;
    marker_line_.header.stamp = this->get_clock().get()->now();
    marker_line_.points = points;
    if (color == RED)
    {
        marker_line_.color.r += 1.0;
    }
    else if (color == GREEN)
    {
        marker_line_.color.g += 1.0;
    }
    else if (color == BLUE)
    {
        marker_line_.color.b += 1.0;
    }

    if (time_to_live < 0)
    {
        publisher_marker_->publish(marker_line_);
    }
    else
    {
        marker_line_.lifetime = rclcpp::Duration::from_seconds(time_to_live);
        publisher_marker_->publish(marker_line_);
        marker_line_.lifetime = rclcpp::Duration::from_seconds(0.0);
    }
    if (unique_line_id_ >= INT32_MAX) unique_line_id_ = 0;
    else unique_line_id_++;
    marker_line_.color.r = marker_line_.color.g = marker_line_.color.b = 0.0f;
}

void RRT::draw_point(const RRT_Node & point, double time_to_live, int color)
{
    marker_point_.id = unique_point_id_;
    marker_point_.action = visualization_msgs::msg::Marker::ADD;
    marker_point_.header.stamp = this->get_clock().get()->now();
    marker_point_.pose.position.x = point.x;
    marker_point_.pose.position.y = point.y;
    if (color == RED)
    {
        marker_point_.color.r += 1.0;
    }
    else if (color == GREEN)
    {
        marker_point_.color.g += 1.0;
    }
    else if (color == BLUE)
    {
        marker_point_.color.b += 1.0;
    }

    if (time_to_live < 0)
    {
        publisher_marker_->publish(marker_point_);
    }
    else
    {
        marker_point_.lifetime = rclcpp::Duration::from_seconds(time_to_live);
        publisher_marker_->publish(marker_point_);
        marker_point_.lifetime = rclcpp::Duration::from_seconds(0.0);
    }
    if (unique_point_id_ >= INT32_MAX) unique_point_id_ = 0;
    else unique_point_id_++;
    marker_point_.color.r = marker_point_.color.g = marker_point_.color.b = 0.0f;
}

void RRT::draw_line(const std::vector<RRT_Node> & points, double time_to_live, int color)
{
    marker_line_.id = unique_line_id_;
    marker_line_.action = visualization_msgs::msg::Marker::ADD;
    marker_line_.header.stamp = this->get_clock().get()->now();
    geometry_msgs::msg::Point p;
    p.z = 0.0;
    for (const auto & point : points)
    {
        p.x = point.x;
        p.y = point.y;
        marker_line_.points.emplace_back(p);
    }
    if (color == RED)
    {
        marker_line_.color.r += 1.0;
    }
    else if (color == GREEN)
    {
        marker_line_.color.g += 1.0;
    }
    else if (color == BLUE)
    {
        marker_line_.color.b += 1.0;
    }

    if (time_to_live < 0)
    {
        publisher_marker_->publish(marker_line_);
    }
    else
    {
        marker_line_.lifetime = rclcpp::Duration::from_seconds(time_to_live);
        publisher_marker_->publish(marker_line_);
        marker_line_.lifetime = rclcpp::Duration::from_seconds(0.0);
    }
    if (unique_line_id_ >= INT32_MAX) unique_line_id_ = 0;
    else unique_line_id_++;
    marker_line_.color.r = marker_line_.color.g = marker_line_.color.b = 0.0f;
}

void RRT::clear_marker()
{
    marker_point_.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_line_.action = visualization_msgs::msg::Marker::DELETEALL;
    publisher_marker_->publish(marker_line_);
     publisher_marker_->publish(marker_point_);
    marker_point_.action = visualization_msgs::msg::Marker::ADD;
    marker_line_.action = visualization_msgs::msg::Marker::ADD;
}

RRT::~RRT()
{
    delete goal_visualizer_;
    delete lookahead_visualizer_;
    delete global_waypoints_visualizer_;
    RCLCPP_INFO(this->get_logger(), "Exiting RRT Node.");
}

RRT::RRT(): rclcpp::Node("rrt_node"), gen_((std::random_device())())
{
    this->declare_parameter("MARGIN", MARGIN_);
    MARGIN_ = this->get_parameter("MARGIN").as_double();
    if (MARGIN_ < 0)
    {
        throw std::invalid_argument("Bad configuration. MARGIN must >= 0.");
    }

    this->declare_parameter("DISTANCE_GOAL_AHEAD", DISTANCE_GOAL_AHEAD_);
    DISTANCE_GOAL_AHEAD_ = this->get_parameter("DISTANCE_GOAL_AHEAD").as_double();
    if (DISTANCE_GOAL_AHEAD_ <= 0)
    {
        throw std::invalid_argument("Bad configuration. DISTANCE_GOAL_AHEAD must > 0.");
    }

    this->declare_parameter("SCAN_RANGE", SCAN_RANGE_);
    SCAN_RANGE_ = this->get_parameter("SCAN_RANGE").as_double();
    if (SCAN_RANGE_ <= 0)
    {
        throw std::invalid_argument("Bad configuration. SCAN_RANGE must > 0.");
    }

    this->declare_parameter("DETECTED_OBS_MARGIN", DETECTED_OBS_MARGIN_);
    DETECTED_OBS_MARGIN_ =  this->get_parameter("DETECTED_OBS_MARGIN").as_double();
    if (DETECTED_OBS_MARGIN_ < 0)
    {
        throw std::invalid_argument("Bad configuration. DETECTED_OBS_MARGIN must >= 0.");
    }

    this->declare_parameter("MIN_RRT_ITERATIONS", MIN_RRT_ITERATIONS_);
    MIN_RRT_ITERATIONS_ = this->get_parameter("MIN_RRT_ITERATIONS").as_int();
    if (MIN_RRT_ITERATIONS_ <= 0) {
        throw std::invalid_argument("Bad configuration. MIN_RRT_ITERATIONS must > 0.");
    }

    this->declare_parameter("MAX_RRT_ITERATIONS", MAX_RRT_ITERATIONS_);
    MAX_RRT_ITERATIONS_ = this->get_parameter("MAX_RRT_ITERATIONS").as_int();
    if (MAX_RRT_ITERATIONS_ < MIN_RRT_ITERATIONS_)
    {
        throw std::invalid_argument("Bad configuration. MAX_RRT_ITERATIONS must >= MIN_RRT_ITERATIONS.");
    }

    this->declare_parameter("STD", STD_);
    STD_ = this->get_parameter("STD").as_double();
    if (STD_ <= 0)
    {
        throw std::invalid_argument("Bad configuration. STD must > 0.");
    }

    this->declare_parameter("STEP_SIZE", STEP_SIZE_);
    STEP_SIZE_ = this->get_parameter("STEP_SIZE").as_double();
    if (STEP_SIZE_ <= 0)
    {
        throw std::invalid_argument("Bad configuration. STEP_SIZE must > 0.");
    }

    this->declare_parameter("NEAR_RANGE", NEAR_RANGE_);
    NEAR_RANGE_ = this->get_parameter("NEAR_RANGE").as_double();
    if (NEAR_RANGE_ <= 0)
    {
        throw std::invalid_argument("Bad configuration. NEAR_RANGE must > 0.");
    }

    this->declare_parameter("GOAL_TOLERANCE", GOAL_TOLERANCE_);
    GOAL_TOLERANCE_ = this->get_parameter("GOAL_TOLERANCE").as_double();
    if (GOAL_TOLERANCE_ <= 0)
    {
        throw std::invalid_argument("Bad configuration. GOAL_TOLERANCE must > 0.");
    }

    this->declare_parameter("GOAL_SAMPLE_RATE", GOAL_SAMPLE_RATE_);
    GOAL_SAMPLE_RATE_ = this->get_parameter("GOAL_SAMPLE_RATE").as_double();
    if (GOAL_SAMPLE_RATE_ < 0 || GOAL_SAMPLE_RATE_ > 1)
    {
        throw std::invalid_argument("Bad configuration. GOAL_SAMPLE_RATE must be between 0 and 1.");
    }

    this->declare_parameter("RRT_WAYPOINT_INTERVAL", RRT_WAYPOINT_INTERVAL_);
    RRT_WAYPOINT_INTERVAL_ = this->get_parameter("RRT_WAYPOINT_INTERVAL").as_double();
    if (RRT_WAYPOINT_INTERVAL_ <= 0)
    {
        throw std::invalid_argument("Bad configuration. RRT_WAYPOINT_INTERVAL must > 0.");
    }

    this->declare_parameter("DISTANCE_LOOK_AHEAD", DISTANCE_LOOK_AHEAD_);
    DISTANCE_LOOK_AHEAD_ = this->get_parameter("DISTANCE_LOOK_AHEAD").as_double();
    if (DISTANCE_LOOK_AHEAD_ <= 0)
    {
        throw std::invalid_argument("Bad configuration. DISTANCE_LOOK_AHEAD must > 0.");
    }

    this->declare_parameter("PID_P", PID_P_);
    PID_P_ = this->get_parameter("PID_P").as_double();
    if (PID_P_ <= 0) {
        throw std::invalid_argument("Bad configuration. PID_P must > 0.");
    }

    this->declare_parameter("odom_topic", odom_topic_);
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    if (odom_topic_.empty())
    {
        throw std::invalid_argument("Bad configuration. odom_topic must not be empty.");
    }

    this->declare_parameter("map_topic", map_topic_);
    map_topic_ = this->get_parameter("map_topic").as_string();
    if (map_topic_.empty())
    {
        throw std::invalid_argument("Bad configuration. map_topic must not be empty.");
    }

    this->declare_parameter("scan_topic", scan_topic_);
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    if (scan_topic_.empty())
    {
        throw std::invalid_argument("Bad configuration. scan_topic must not be empty.");
    }

    this->declare_parameter("dynamic_map_topic", dynamic_map_topic_);
    dynamic_map_topic_ = this->get_parameter("dynamic_map_topic").as_string();
    if (dynamic_map_topic_.empty())
    {
        throw std::invalid_argument("Bad configuration. dynamic_map_topic must not be empty.");
    }

    this->declare_parameter("drive_topic", drive_topic_);
    drive_topic_ = this->get_parameter("drive_topic").as_string();
    if (drive_topic_.empty())
    {
        throw std::invalid_argument("Bad configuration. drive_topic must not be empty.");
    }

    this->declare_parameter("control_topic", control_topic_);
    control_topic_ = this->get_parameter("control_topic").as_string();
    if (control_topic_.empty())
    {
        throw std::invalid_argument("Bad configuration. control_topic must not be empty.");
    }

    this->declare_parameter("start_on_launch", start_on_launch_);
    start_on_launch_ = this->get_parameter("start_on_launch").as_bool();
    is_vehicle_enabled_ = start_on_launch_;

    this->declare_parameter("waypoint_file_path", waypoint_file_path_);
    waypoint_file_path_ = this->get_parameter("waypoint_file_path").as_string();

    CSVHandler csv_handler(waypoint_file_path_);
    try
    {
        global_waypoints_ = csv_handler.read_waypoint_list_from_csv();
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "ERROR: %s", e.what());
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    laser_frame_ = this->get_namespace() + laser_frame_;
    laser_frame_ = laser_frame_.substr(1);
    car_frame_ = this->get_namespace() + car_frame_;
    car_frame_ = car_frame_.substr(1);

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    subscriber_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic_, map_qos, std::bind(&RRT::map_callback, this, std::placeholders::_1));
    subscriber_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));
    subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 1, std::bind(&RRT::odom_callback, this, std::placeholders::_1));
    subscriber_control_ = this->create_subscription<std_msgs::msg::String>(control_topic_, 10, std::bind(&RRT::control_callback, this, std::placeholders::_1));
    publisher_dynamic_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(dynamic_map_topic_, 1);
    publisher_path_ = this->create_publisher<visualization_msgs::msg::Marker>("path", 1);
    publisher_tree_node_ = this->create_publisher<visualization_msgs::msg::Marker>("tree_nodes", 1);
    publisher_tree_branches_ = this->create_publisher<visualization_msgs::msg::Marker>("tree_branches", 1);
    publisher_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_marker", 10);
    publisher_goal_visualizer_ = this->create_publisher<visualization_msgs::msg::Marker>("goal", 10);
    auto global_waypoints_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    publisher_global_waypoints_ = this->create_publisher<visualization_msgs::msg::Marker>("global_waypoints", global_waypoints_qos);
    publisher_lookahead_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead", 10);
    publisher_drive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 1);
    init_marker();

    std_msgs::msg::ColorRGBA red; red.r =1.0; red.a=1.0;
    std_msgs::msg::ColorRGBA green; green.g =1.0; green.a=1.0;
    std_msgs::msg::ColorRGBA blue; blue.b =1.0; blue.a=1.0;
    std_msgs::msg::ColorRGBA yellow; yellow.r =1.0; yellow.g=0.75; yellow.a=1.0;

    goal_visualizer_ = new MarkerVisualizer(publisher_goal_visualizer_, "goal", "map", green, 0.3, visualization_msgs::msg::Marker::SPHERE);
    lookahead_visualizer_ = new MarkerVisualizer(publisher_lookahead_, "lookahead", "map", red, 0.2, visualization_msgs::msg::Marker::SPHERE);
    global_waypoints_visualizer_ = new PointsVisualizer(publisher_global_waypoints_, "global_waypoints", "map", yellow, 0.08);
    for (const auto& waypoint : global_waypoints_)
    {
        global_waypoints_visualizer_->add_point(waypoint);
    }
    global_waypoints_visualizer_->publish_points(false);
    RCLCPP_INFO(this->get_logger(), "Published %zu global waypoints.", global_waypoints_.size());
    global_waypoints_timer_ = this->create_wall_timer(std::chrono::seconds(5), [this]()
    {
        global_waypoints_visualizer_->publish_points(false);
    });

    tree_nodes_.header.frame_id = tree_branch_.header.frame_id = "map";
    tree_nodes_.ns = "nodes"; tree_branch_.ns = "branch";
    tree_nodes_.action = tree_branch_.action = visualization_msgs::msg::Marker::ADD;
    tree_nodes_.pose.orientation.w = tree_branch_.pose.orientation.w = 1.0;
    tree_nodes_.id = 5; tree_branch_.id = 6;
    tree_nodes_.type = visualization_msgs::msg::Marker::POINTS;
    tree_branch_.type = visualization_msgs::msg::Marker::LINE_LIST;
    tree_nodes_.scale.x = tree_nodes_.scale.y = tree_nodes_.scale.z = 0.05;
    tree_branch_.scale.x = 0.01;
    tree_nodes_.color = red; tree_branch_.color = blue;

    RCLCPP_INFO_STREAM(this->get_logger(), "Node started successfully!");
    if (is_vehicle_enabled_)
    {
        RCLCPP_WARN(this->get_logger(), "Vehicle motion is enabled on launch.");
    }
    else
    {
        stop_vehicle();
        RCLCPP_INFO(this->get_logger(), "Vehicle stopped. Publish 'start' to %s to enable motion.", control_topic_.c_str());
    }
}

void RRT::control_callback(const std_msgs::msg::String::ConstSharedPtr control_msg)
{
    if (control_msg->data == "start")
    {
        is_vehicle_enabled_ = true;
        RCLCPP_INFO(this->get_logger(), "Vehicle motion enabled.");
    }
    else if (control_msg->data == "stop")
    {
        is_vehicle_enabled_ = false;
        stop_vehicle();
        RCLCPP_INFO(this->get_logger(), "Vehicle stopped. Planning and visualization remain active.");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Invalid control command '%s'. Expected 'start' or 'stop'.", control_msg->data.c_str());
    }
}

void RRT::map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg)
{
    map_ = *map_msg;
    dynamic_map_ = *map_msg;
    RCLCPP_INFO(this->get_logger(), "Initial map received.");
    previous_map_updating_time_ = this->get_clock()->now();
    // inflate map to set safety bubble.
    occupancy_grid::inflate_map(dynamic_map_, MARGIN_);
    is_map_initialized_ = true;
    is_goal_initialized_ = false;
    subscriber_map_.reset();
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    if (!is_map_initialized_)
    {
        return;
    }

    if (!lookup_transform())
    {
        return;
    }
    
    // clear obstacles every 0.5 seconds.
    if ((this->get_clock()->now() - previous_map_updating_time_).seconds() > 0.5)
    {
        clear_map();
        previous_map_updating_time_ = this->get_clock()->now();
    }


    const float angle_min = scan_msg->angle_min;
    const float angle_increment = scan_msg->angle_increment;
    const int size_laserscan = scan_msg->ranges.size();

    for (int i = 0; i < size_laserscan; i++)
    {
        const float range = scan_msg->ranges.at(i);
        if (range > SCAN_RANGE_ || std::isnan(range) || std::isinf(range))
        {
            continue;
        }
        const float angle = angle_min + angle_increment * i;
        geometry_msgs::msg::Point hit_point_laserframe;
        hit_point_laserframe.x = range * std::cos(angle);
        hit_point_laserframe.y = range * std::sin(angle);
        auto hit_point_mapframe = tf_point_laser_to_map(hit_point_laserframe);
        // if this is a new obstacle.
        if (!occupancy_grid::is_xy_coord_occupied(map_, hit_point_mapframe.x, hit_point_mapframe.y))
        {
            const std::vector<int> new_obstacles_indices = occupancy_grid::inflate_cell(dynamic_map_, occupancy_grid::xy_coord_to_array_index(dynamic_map_, hit_point_mapframe.x, hit_point_mapframe.y), DETECTED_OBS_MARGIN_, 100);
            obstacle_indices_.insert(obstacle_indices_.end(), new_obstacles_indices.begin(), new_obstacles_indices.end());
        }
    }

    publisher_dynamic_map_->publish(dynamic_map_);
}

void RRT::clear_map()
{
    for (const auto& index : obstacle_indices_)
    {
        if (index >= 0 && index < int(dynamic_map_.data.size()) && index < int(map_.data.size()))
        {
            dynamic_map_.data.at(index) = map_.data.at(index);
        }
    }
    obstacle_indices_.clear();
}

bool RRT::lookup_transform()
{
    try
    {
       tf2_transform_laser_to_map_ = tf_buffer_->lookupTransform(map_frame_, laser_frame_, tf2::TimePointZero);
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", laser_frame_.c_str(), map_frame_.c_str(), e.what());
        return false;
    }


    try
    {
        tf2_transform_map_to_laser_ = tf_buffer_->lookupTransform(laser_frame_, map_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& e)
    {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", map_frame_.c_str(), laser_frame_.c_str(), e.what());
        return false;
    }

    try
    {
        tf2_transform_map_to_car_ = tf_buffer_->lookupTransform(car_frame_, map_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& e)
    {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", map_frame_.c_str(), car_frame_.c_str(), e.what());
        return false;
    }

    try
    {
        tf2_transform_car_to_map_ = tf_buffer_->lookupTransform(map_frame_, car_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& e)
    {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", car_frame_.c_str(), map_frame_.c_str(), e.what());
        return false;
    }

    return true;
}

geometry_msgs::msg::Point RRT::tf_point_laser_to_map(const geometry_msgs::msg::Point & point_laserframe)
{
    geometry_msgs::msg::PointStamped point_stamped_mapframe, point_stamped_laserframe;
    point_stamped_laserframe.point = point_laserframe;
    tf2::doTransform(point_stamped_laserframe, point_stamped_mapframe, tf2_transform_laser_to_map_);
    return point_stamped_mapframe.point;
}

geometry_msgs::msg::Point RRT::tf_point_map_to_laser(const geometry_msgs::msg::Point & point_mapframe)
{
    geometry_msgs::msg::PointStamped point_stamped_mapframe, point_stamped_laserframe;
    point_stamped_mapframe.point = point_mapframe;
    tf2::doTransform(point_stamped_mapframe, point_stamped_laserframe, tf2_transform_map_to_laser_);
    return point_stamped_laserframe.point;
}

geometry_msgs::msg::Point RRT::tf_point_map_to_car(const geometry_msgs::msg::Point & point_mapframe)
{
    geometry_msgs::msg::PointStamped point_stamped_mapframe, point_stamped_carframe;
    point_stamped_mapframe.point = point_mapframe;
    tf2::doTransform(point_stamped_mapframe, point_stamped_carframe, tf2_transform_map_to_car_);
    return point_stamped_carframe.point;
}

geometry_msgs::msg::Point RRT::tf_point_car_to_map(const geometry_msgs::msg::Point & point_carframe)
{
    geometry_msgs::msg::PointStamped point_stamped_mapframe, point_stamped_carframe;
    point_stamped_carframe.point = point_carframe;
    tf2::doTransform(point_stamped_carframe, point_stamped_mapframe, tf2_transform_car_to_map_);
    return point_stamped_mapframe.point;
}

// use square to avoid sqrt operation which might cause heavy calculation workload.
double euclidean_distance_square(const double x1, const double y1, const double x2, const double y2)
{
    return std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2);
}

bool RRT::initialize_goal_waypoint()
{
    if (global_waypoints_.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "The waypoint list is empty.");
        return false;
    }

    int closest_waypoint_index = find_closest_ahead_waypoint(global_waypoints_, current_pose_);
    if (closest_waypoint_index < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Couldn't find a waypoint ahead of the vehicle.");
        return false;
    }

    float closest_distance_square = euclidean_distance_square(global_waypoints_.at(closest_waypoint_index).x, global_waypoints_.at(closest_waypoint_index).y, current_pose_.position.x, current_pose_.position.y);
    // the closet waypoint is too far.
    if (closest_distance_square > std::pow(DISTANCE_GOAL_AHEAD_, 2))
    {
        RCLCPP_ERROR(this->get_logger(), "Couldn't find a goal in range. Reposition the car somewhere near the waypoints");
        return false;
    }

    current_goal_index_ = closest_waypoint_index;
    is_goal_initialized_ = find_ahead_goal_waypoint();
    return is_goal_initialized_;
}

bool RRT::find_ahead_goal_waypoint()
{
    if (global_waypoints_.empty())
    {
        return false;
    }

    int current_goal_index = current_goal_index_;
    if (current_goal_index >= (int)global_waypoints_.size())
    {
        current_goal_index = 0;
    }
    const float pose_x = current_pose_.position.x;
    const float pose_y = current_pose_.position.y;
    const float min_goal_distance_square = std::pow(DISTANCE_GOAL_AHEAD_ * 0.9, 2);
    const float max_goal_distance_square = std::pow(DISTANCE_GOAL_AHEAD_, 2);
    int selected_goal_index = -1;

    // filter the free waypoints between 0.9 * goal-ahead distance to 1.0 * goal-ahead distance.
    for (int i = 0; i < int(global_waypoints_.size()); i++)
    {
        const auto& waypoint = global_waypoints_.at(current_goal_index);
        const float current_distance_square = euclidean_distance_square(waypoint.x, waypoint.y, pose_x, pose_y);

        if (current_distance_square > max_goal_distance_square && i > 0)
        {
            break;
        }

        if (current_distance_square <= max_goal_distance_square && !occupancy_grid::is_xy_coord_occupied(dynamic_map_, waypoint.x, waypoint.y))
        {
            selected_goal_index = current_goal_index;
            if (current_distance_square >= min_goal_distance_square)
            {
                break;
            }
        }

        current_goal_index = (current_goal_index + 1) % int(global_waypoints_.size());
    }

    if (selected_goal_index < 0)
    {
        RCLCPP_WARN(this->get_logger(), "Couldn't find a free goal waypoint in range.");
        return false;
    }

    current_goal_index_ = selected_goal_index;
    visualize_goal();
    return true;
}

int RRT::find_closest_ahead_waypoint(const std::vector<geometry_msgs::msg::Point>& waypoints, const geometry_msgs::msg::Pose& pose)
{
    float min_distance_square = std::numeric_limits<float>::max();
    int min_distance_index = -1;
    const int size_waypoint_list = waypoints.size();
    for (int i = 0; i < size_waypoint_list; i++)
    {
        if (tf_point_map_to_car(waypoints.at(i)).x < 0)
        {
            continue;
        }
        float distance_square = euclidean_distance_square(waypoints.at(i).x, waypoints.at(i).y, pose.position.x, pose.position.y);
        if (distance_square < min_distance_square)
        {
            min_distance_square = distance_square;
            min_distance_index = i;
        }
    }
    return min_distance_index;
}

bool RRT::find_current_goal()
{
    if (!is_goal_initialized_ && !initialize_goal_waypoint())
    {
        return false;
    }

    const float distance_to_goal_square = euclidean_distance_square(global_waypoints_.at(current_goal_index_).x, global_waypoints_.at(current_goal_index_).y, current_pose_.position.x, current_pose_.position.y);
    // if goal is too far, reinitialize the goal.
    if (distance_to_goal_square > std::pow(DISTANCE_GOAL_AHEAD_, 2))
    {
        return initialize_goal_waypoint();
    }
    // if goal is occupied, move forward.
    if (occupancy_grid::is_xy_coord_occupied(dynamic_map_, global_waypoints_.at(current_goal_index_).x, global_waypoints_.at(current_goal_index_).y))
    {
        return find_ahead_goal_waypoint();
    }
    // if goal is too close, move forward.
    else if(distance_to_goal_square < std::pow(DISTANCE_GOAL_AHEAD_ * 0.75, 2))
    {
        return find_ahead_goal_waypoint();
    }
    return true;
}

void RRT::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    current_pose_ = odom_msg->pose.pose;
    if (!is_map_initialized_)
    {
        return;
    }

    if (!lookup_transform())
    {
        stop_vehicle();
        return;
    }

    if (!find_current_goal())
    {
        stop_vehicle();
        return;
    }

    RRT_Node start_node{};
    start_node.x = current_pose_.position.x;
    start_node.y = current_pose_.position.y;
    start_node.parent = 0;
    start_node.cost = 0.0;
    start_node.is_root = true;

    std::vector<RRT_Node> tree;
    std::vector<int> nodes_near_goal;
    bool is_path_found = false;

    tree.push_back(start_node);

    for (int iteration = 0; iteration < MAX_RRT_ITERATIONS_; iteration++)
    {
        std::pair<float, float> sampled_point;
        if (!sample(sampled_point))
        {
            RCLCPP_WARN(this->get_logger(), "Couldn't sample a free point in the map.");
            break;
        }
        int nearest_node_index = nearest(tree, sampled_point);
        RRT_Node new_node = steer(tree.at(nearest_node_index), sampled_point);

        if (line_cost(tree.at(nearest_node_index), new_node) <= std::numeric_limits<double>::epsilon())
        {
            continue;
        }

        if (!check_collision(tree.at(nearest_node_index), new_node))
        {
            std::vector<int> near_nodes = near(tree, new_node);
            tree.emplace_back(new_node);

            int min_cost_node_index = nearest_node_index;
            float min_cost = tree.at(nearest_node_index).cost + line_cost(tree.at(nearest_node_index), new_node);

            // RRT*: find the best parent.
            for (int i = 0; i < int(near_nodes.size()); i++)
            {
                if(!check_collision(tree.at(near_nodes.at(i)), new_node))
                {
                    float cost = tree.at(near_nodes.at(i)).cost + line_cost(tree.at(near_nodes.at(i)), new_node);
                    if(cost < min_cost) {
                       min_cost_node_index = near_nodes.at(i);
                       min_cost = cost;
                   }
                }
            }

            tree.back().is_root = false;
            tree.back().cost = min_cost;

            tree.back().parent = min_cost_node_index;
            tree.at(min_cost_node_index).children.emplace_back(tree.size() - 1);

            // RRT*: rewiring.
            for (int i = 0; i < int(near_nodes.size()); i++)
            {
                float new_cost = tree.back().cost + line_cost(new_node, tree.at(near_nodes.at(i)));
                if (new_cost < tree.at(near_nodes.at(i)).cost && !check_collision(tree.at(near_nodes.at(i)), new_node))
                {
                    tree.at(near_nodes.at(i)).cost = new_cost;
                    int old_parent = tree.at(near_nodes.at(i)).parent;
                    tree.at(near_nodes.at(i)).parent = tree.size() - 1;
                    tree.back().children.emplace_back(near_nodes.at(i));
                    std::vector<int>::iterator start = tree.at(old_parent).children.begin();
                    std::vector<int>::iterator end = tree.at(old_parent).children.end();
                    tree.at(old_parent).children.erase(remove(start, end, near_nodes.at(i)), end);
                    update_descendant_costs(tree, near_nodes.at(i));
                }
            }

            if (is_goal(tree.back(), global_waypoints_.at(current_goal_index_).x, global_waypoints_.at(current_goal_index_).y))
            {
                nodes_near_goal.emplace_back(tree.size() - 1);
            }
        }

        if(iteration + 1 >= MIN_RRT_ITERATIONS_ && !nodes_near_goal.empty())
        {
            // find out the lowerest cost node to goal waypoint.
            int best_node_index = *min_element(nodes_near_goal.begin(), nodes_near_goal.end(), [this, &tree](const int n1, const int n2)
            {
                const auto& goal = global_waypoints_.at(current_goal_index_);
                const double cost1 = tree.at(n1).cost + std::sqrt(euclidean_distance_square(tree.at(n1).x, tree.at(n1).y, goal.x, goal.y));
                const double cost2 = tree.at(n2).cost + std::sqrt(euclidean_distance_square(tree.at(n2).x, tree.at(n2).y, goal.x, goal.y));
                return cost1 < cost2;
            });

            std::vector<RRT_Node> found_path = find_path(tree, tree.at(best_node_index));

            // visualize the path.
            visualization_msgs::msg::Marker path_marker;
            path_marker.header.frame_id = "map";
            path_marker.id = 20;
            path_marker.ns = "path";
            path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            path_marker.scale.x = 0.05;
            path_marker.action = visualization_msgs::msg::Marker::ADD;
            path_marker.pose.orientation.w = 1.0;
            path_marker.color.g = 1.0;
            path_marker.color.r = 0.0;
            path_marker.color.a = 1.0;

            for (int i=0; i < (int)found_path.size(); i++)
            {
                geometry_msgs::msg::Point p;
                p.x = found_path.at(i).x;
                p.y = found_path.at(i).y;
                path_marker.points.emplace_back(p);
            }

            std::vector<geometry_msgs::msg::Point> path_interpolated;

            // smooth the path.
            for (int i = 0; i < (int)path_marker.points.size() - 1; i++)
            {
                path_interpolated.emplace_back(path_marker.points[i]);
                float distance = std::sqrt(std::pow(path_marker.points[i + 1].x - path_marker.points[i].x, 2) + std::pow(path_marker.points[i + 1].y - path_marker.points[i].y, 2));
                if (distance < RRT_WAYPOINT_INTERVAL_)
                {
                    continue;
                }

                int num = static_cast<int>(std::ceil(distance / RRT_WAYPOINT_INTERVAL_));
                for(int j = 1; j < num; j++)
                {
                    geometry_msgs::msg::Point p;
                    p.x = path_marker.points[i].x + j * ((path_marker.points[i + 1].x - path_marker.points[i].x) / num);
                    p.y = path_marker.points[i].y + j * ((path_marker.points[i + 1].y - path_marker.points[i].y) / num);
                    path_interpolated.emplace_back(p);
                }
            }
            path_interpolated.emplace_back(path_marker.points.back());

            nav_msgs::msg::Path path;
            path.header.frame_id = "map";

            for (const auto& point : path_interpolated)
            {
                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.pose.position = point;
                path.poses.emplace_back(pose_stamped);
            }

            path_marker.points = path_interpolated;
            publisher_path_->publish(path_marker);
            follow_the_path(path);
            visualize_tree(tree);
            is_path_found = true;
            break;
        }
    }

    if (!is_path_found)
    {
        const bool start_occupied = occupancy_grid::is_xy_coord_occupied(dynamic_map_, start_node.x, start_node.y);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Couldn't find a path. Stopping the vehicle. Tree nodes: %zu, goal candidates: %zu, start occupied: %s.",
            tree.size(), nodes_near_goal.size(), start_occupied ? "true" : "false");
        stop_vehicle();
    }
}

bool RRT::sample(std::pair<float, float>& sampled_point)
{
    const auto& goal = global_waypoints_.at(current_goal_index_);
    if (uni_dist_(gen_) < GOAL_SAMPLE_RATE_ && !occupancy_grid::is_xy_coord_occupied(dynamic_map_, goal.x, goal.y))
    {
        sampled_point.first = goal.x;
        sampled_point.second = goal.y;
        return true;
    }

    std::normal_distribution<double> norm_dist_x(0.6 * global_waypoints_.at(current_goal_index_).x + 0.4 * current_pose_.position.x, STD_);
    std::normal_distribution<double> norm_dist_y(0.6 * global_waypoints_.at(current_goal_index_).y + 0.4 * current_pose_.position.y, STD_);

    constexpr int MAX_SAMPLE_ATTEMPTS = 1000;
    for (int i = 0; i < MAX_SAMPLE_ATTEMPTS; i++)
    {
        const double x = norm_dist_x(gen_);
        const double y = norm_dist_y(gen_);
        if (!occupancy_grid::is_xy_coord_occupied(dynamic_map_, x, y))
        {
            sampled_point.first = x;
            sampled_point.second = y;
            return true;
        }
    }
    return false;
}

int RRT::nearest(std::vector<RRT_Node>& tree, std::pair<float, float>& sampled_point)
{
    int nearest_node_index = 0;
    float min_distance_square = std::numeric_limits<float>::max();
    const int size_tree = tree.size();
    for (int i = 0; i < size_tree; i++)
    {
        float distance_square = euclidean_distance_square(tree.at(i).x, tree.at(i).y, sampled_point.first, sampled_point.second);
        if (distance_square < min_distance_square)
        {
            nearest_node_index = i;
            min_distance_square = distance_square;
        }
    }
    return nearest_node_index;
}

RRT_Node RRT::steer(RRT_Node& nearest_node, std::pair<float, float>& sampled_point)
{
    RRT_Node new_node{};
    float distance = std::sqrt(euclidean_distance_square(nearest_node.x, nearest_node.y, sampled_point.first, sampled_point.second));

    if (distance <= std::numeric_limits<float>::epsilon())
    {
        new_node.x = nearest_node.x;
        new_node.y = nearest_node.y;
        return new_node;
    }

    new_node.x = nearest_node.x + std::min(STEP_SIZE_, distance) * (sampled_point.first - nearest_node.x) / distance;
    new_node.y = nearest_node.y + std::min(STEP_SIZE_, distance) * (sampled_point.second - nearest_node.y) / distance;

    return new_node;
}

bool RRT::check_collision(RRT_Node& nearest_node, RRT_Node& new_node)
{
    if (dynamic_map_.info.resolution <= 0.0)
    {
        return true;
    }

    int x_cell_diff = std::ceil(std::abs(nearest_node.x - new_node.x) / dynamic_map_.info.resolution);
    int y_cell_diff = std::ceil(std::abs(nearest_node.y - new_node.y) / dynamic_map_.info.resolution);
    const int num_steps = std::max(x_cell_diff, y_cell_diff);
    const float dt = num_steps > 0 ? 1.0 / num_steps : 0.0;
    const bool root_in_inflated_space = nearest_node.is_root &&
        occupancy_grid::is_xy_coord_occupied(dynamic_map_, nearest_node.x, nearest_node.y) &&
        !occupancy_grid::is_xy_coord_occupied(map_, nearest_node.x, nearest_node.y);
    const float escape_distance = std::max(MARGIN_, DETECTED_OBS_MARGIN_) + dynamic_map_.info.resolution;
    const float escape_distance_square = std::pow(escape_distance, 2);
    float t = 0.0;

    for (int i = 0; i <= num_steps; i++)
    {
        float x = nearest_node.x + t * (new_node.x - nearest_node.x);
        float y = nearest_node.y + t * (new_node.y - nearest_node.y);
        if (occupancy_grid::is_xy_coord_occupied(dynamic_map_, x, y))
        {
            const float distance_from_root_square = euclidean_distance_square(nearest_node.x, nearest_node.y, x, y);
            if (root_in_inflated_space && distance_from_root_square <= escape_distance_square &&
                !occupancy_grid::is_xy_coord_occupied(map_, x, y))
            {
                t += dt;
                continue;
            }
            return true;
        }
        t += dt;
    }
    return false;
}

double RRT::line_cost(RRT_Node& n1, RRT_Node& n2)
{
    return std::sqrt(euclidean_distance_square(n1.x, n1.y, n2.x, n2.y));
}

std::vector<int> RRT::near(std::vector<RRT_Node>& tree, RRT_Node& node)
{
    std::vector<int> neighborhood;
    const int size_tree = tree.size();
    for (int i = 0; i < size_tree; i++)
    {
        if (line_cost(tree.at(i), node) < NEAR_RANGE_)
        {
            neighborhood.emplace_back(i);
        }
    }
    return neighborhood;
}

bool RRT::is_goal(RRT_Node& new_node, float x_goal, float y_goal)
{
    return euclidean_distance_square(x_goal, y_goal, new_node.x, new_node.y) < std::pow(GOAL_TOLERANCE_, 2);
}

std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node>& tree, RRT_Node& node)
{
    std::vector<RRT_Node> found_path;
    RRT_Node current = node;
    while (!current.is_root)
    {
        found_path.emplace_back(current);
        current = tree.at(current.parent);
    }
    found_path.emplace_back(current);
    std::reverse(found_path.begin(), found_path.end());
    return found_path;
}

void RRT::update_descendant_costs(std::vector<RRT_Node>& tree, const int node_index)
{
    for (const auto child_index : tree.at(node_index).children)
    {
        tree.at(child_index).cost = tree.at(node_index).cost + line_cost(tree.at(node_index), tree.at(child_index));
        update_descendant_costs(tree, child_index);
    }
}

void RRT::visualize_goal()
{
    geometry_msgs::msg::Pose goal_pose;
    goal_pose.orientation.w = 1.0;
    goal_pose.position.x = global_waypoints_.at(current_goal_index_).x;
    goal_pose.position.y = global_waypoints_.at(current_goal_index_).y;
    goal_visualizer_->set_pose(goal_pose);
    goal_visualizer_->publish_marker();
}

void RRT::visualize_tree(std::vector<RRT_Node>& tree)
{
    visualize_goal();

    tree_nodes_.points.clear();
    tree_branch_.points.clear();

    const int size_tree = tree.size();
    for (int i = 0; i < size_tree; i++)
    {
        geometry_msgs::msg::Point p;
        p.x = tree.at(i).x;
        p.y = tree.at(i).y;
        tree_nodes_.points.emplace_back(p);
        for (int j = 0; j < (int)tree.at(i).children.size(); j++)
        {
            tree_branch_.points.emplace_back(p);
            geometry_msgs::msg::Point p_child;
            p_child.x = tree.at(tree.at(i).children.at(j)).x;
            p_child.y = tree.at(tree.at(i).children.at(j)).y;
            tree_branch_.points.push_back(p_child);
        }
    }
    publisher_tree_branches_->publish(tree_branch_);
    publisher_tree_node_->publish(tree_nodes_);
}

void RRT::follow_the_path(const nav_msgs::msg::Path& path)
{
    int i = 0;
    while (i < (int)path.poses.size() - 1)
    {
        float x = path.poses.at(i).pose.position.x;
        float y = path.poses.at(i).pose.position.y;
        float x_car = current_pose_.position.x;
        float y_car = current_pose_.position.y;
        if (euclidean_distance_square(x, y, x_car, y_car) > std::pow(DISTANCE_LOOK_AHEAD_, 2))
        {
            break;
        }
        i++;
    }

    geometry_msgs::msg::Point waypoint_mapframe;
    waypoint_mapframe.x = path.poses.at(i).pose.position.x;
    waypoint_mapframe.y = path.poses.at(i).pose.position.y;
    waypoint_mapframe.z = 0.0;
    auto waypoint_carframe = tf_point_map_to_car(waypoint_mapframe);

    float steering_angle = PID_P_ * 2 * waypoint_carframe.y / std::pow(DISTANCE_LOOK_AHEAD_, 2);

    steering_angle = std::min(steering_angle, 0.41f);
    steering_angle = std::max(steering_angle, -0.41f);
    float speed = get_speed(steering_angle);
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp = this->now();
    msg.drive.speed = speed;
    msg.drive.steering_angle = steering_angle;
    msg.drive.steering_angle_velocity = 1.0;
    if (is_vehicle_enabled_)
    {
        publisher_drive_->publish(msg);
    }
    else
    {
        stop_vehicle();
    }

    geometry_msgs::msg::Pose lookahead_pose;
    lookahead_pose.orientation.w = 1.0;
    lookahead_pose.position = waypoint_mapframe;
    lookahead_visualizer_->set_pose(lookahead_pose);
    lookahead_visualizer_->publish_marker();
}

float rad_to_deg(const float rads)
{
    return rads * 180.0 / M_PI;
}

float deg_to_rad(const float degs)
{
    return degs * M_PI / 180.0;
}

void RRT::stop_vehicle()
{
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp = this->now();
    msg.drive.speed = 0.0;
    msg.drive.steering_angle = 0.0;
    publisher_drive_->publish(msg);
}

float RRT::get_speed(const float steering_angle)
{
    // return 0.0;
    float abs_angle = std::abs(steering_angle);
    if (abs_angle <= deg_to_rad(10))
    {
        return 2.0;
    }
    else if (abs_angle <= deg_to_rad(20))
    {
        return 1.0;
    }
    else
    {
        return 0.5;
    }
}
