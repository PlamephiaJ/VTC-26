#include "motion_planning/RRT.hpp"

#include "motion_planning/FileHandler.hpp"
#include "motion_planning/occupancy_grid.hpp"
#include "motion_planning/path_processing.hpp"
#include "motion_planning/path_tracking.hpp"
#include "motion_planning/scan_processing.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <functional>
#include <stdexcept>

RRT::RRT()
    : rclcpp::Node("rrt_node")
{
    load_parameters();
    load_global_waypoints();
    initialize_algorithm_modules();
    initialize_ros_interfaces();

    RCLCPP_INFO(this->get_logger(), "RRT motion-planning node started.");
    if (is_vehicle_enabled_)
    {
        RCLCPP_WARN(this->get_logger(), "Vehicle motion is enabled on launch.");
    }
    else
    {
        stop_vehicle();
        RCLCPP_INFO(
            this->get_logger(),
            "Vehicle stopped. Publish 'start' to %s to enable motion.",
            control_topic_.c_str());
    }
}

RRT::~RRT()
{
    RCLCPP_INFO(this->get_logger(), "Exiting RRT node.");
}

void RRT::load_parameters()
{
    this->declare_parameter("MARGIN", map_inflation_margin_);
    map_inflation_margin_ = this->get_parameter("MARGIN").as_double();
    if (map_inflation_margin_ < 0.0)
    {
        throw std::invalid_argument("Bad configuration. MARGIN must be >= 0.");
    }

    this->declare_parameter("DISTANCE_GOAL_AHEAD", goal_ahead_distance_);
    goal_ahead_distance_ = this->get_parameter("DISTANCE_GOAL_AHEAD").as_double();
    if (goal_ahead_distance_ <= 0.0)
    {
        throw std::invalid_argument(
            "Bad configuration. DISTANCE_GOAL_AHEAD must be > 0.");
    }

    this->declare_parameter("SCAN_RANGE", scan_range_);
    scan_range_ = this->get_parameter("SCAN_RANGE").as_double();
    if (scan_range_ <= 0.0)
    {
        throw std::invalid_argument("Bad configuration. SCAN_RANGE must be > 0.");
    }

    this->declare_parameter(
        "DETECTED_OBS_MARGIN", detected_obstacle_margin_);
    detected_obstacle_margin_ =
        this->get_parameter("DETECTED_OBS_MARGIN").as_double();
    if (detected_obstacle_margin_ < 0.0)
    {
        throw std::invalid_argument(
            "Bad configuration. DETECTED_OBS_MARGIN must be >= 0.");
    }

    this->declare_parameter("MIN_RRT_ITERATIONS", minimum_rrt_iterations_);
    minimum_rrt_iterations_ =
        this->get_parameter("MIN_RRT_ITERATIONS").as_int();
    if (minimum_rrt_iterations_ <= 0)
    {
        throw std::invalid_argument(
            "Bad configuration. MIN_RRT_ITERATIONS must be > 0.");
    }

    this->declare_parameter("MAX_RRT_ITERATIONS", maximum_rrt_iterations_);
    maximum_rrt_iterations_ =
        this->get_parameter("MAX_RRT_ITERATIONS").as_int();
    if (maximum_rrt_iterations_ < minimum_rrt_iterations_)
    {
        throw std::invalid_argument(
            "Bad configuration. MAX_RRT_ITERATIONS must be >= "
            "MIN_RRT_ITERATIONS.");
    }

    this->declare_parameter("STD", sample_standard_deviation_);
    sample_standard_deviation_ = this->get_parameter("STD").as_double();
    if (sample_standard_deviation_ <= 0.0)
    {
        throw std::invalid_argument("Bad configuration. STD must be > 0.");
    }

    this->declare_parameter("STEP_SIZE", step_size_);
    step_size_ = this->get_parameter("STEP_SIZE").as_double();
    if (step_size_ <= 0.0)
    {
        throw std::invalid_argument("Bad configuration. STEP_SIZE must be > 0.");
    }

    this->declare_parameter("NEAR_RANGE", near_range_);
    near_range_ = this->get_parameter("NEAR_RANGE").as_double();
    if (near_range_ <= 0.0)
    {
        throw std::invalid_argument("Bad configuration. NEAR_RANGE must be > 0.");
    }

    this->declare_parameter("GOAL_TOLERANCE", goal_tolerance_);
    goal_tolerance_ = this->get_parameter("GOAL_TOLERANCE").as_double();
    if (goal_tolerance_ <= 0.0)
    {
        throw std::invalid_argument(
            "Bad configuration. GOAL_TOLERANCE must be > 0.");
    }

    this->declare_parameter("GOAL_SAMPLE_RATE", goal_sample_rate_);
    goal_sample_rate_ = this->get_parameter("GOAL_SAMPLE_RATE").as_double();
    if (goal_sample_rate_ < 0.0 || goal_sample_rate_ > 1.0)
    {
        throw std::invalid_argument(
            "Bad configuration. GOAL_SAMPLE_RATE must be in [0, 1].");
    }

    this->declare_parameter("RRT_WAYPOINT_INTERVAL", rrt_waypoint_interval_);
    rrt_waypoint_interval_ =
        this->get_parameter("RRT_WAYPOINT_INTERVAL").as_double();
    if (rrt_waypoint_interval_ <= 0.0)
    {
        throw std::invalid_argument(
            "Bad configuration. RRT_WAYPOINT_INTERVAL must be > 0.");
    }

    this->declare_parameter("DISTANCE_LOOK_AHEAD", lookahead_distance_);
    lookahead_distance_ =
        this->get_parameter("DISTANCE_LOOK_AHEAD").as_double();
    if (lookahead_distance_ <= 0.0)
    {
        throw std::invalid_argument(
            "Bad configuration. DISTANCE_LOOK_AHEAD must be > 0.");
    }

    this->declare_parameter("PID_P", pursuit_gain_);
    pursuit_gain_ = this->get_parameter("PID_P").as_double();
    if (pursuit_gain_ <= 0.0)
    {
        throw std::invalid_argument("Bad configuration. PID_P must be > 0.");
    }

    this->declare_parameter("odom_topic", odom_topic_);
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    this->declare_parameter("map_topic", map_topic_);
    map_topic_ = this->get_parameter("map_topic").as_string();
    this->declare_parameter("scan_topic", scan_topic_);
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    this->declare_parameter("dynamic_map_topic", dynamic_map_topic_);
    dynamic_map_topic_ = this->get_parameter("dynamic_map_topic").as_string();
    this->declare_parameter("drive_topic", drive_topic_);
    drive_topic_ = this->get_parameter("drive_topic").as_string();
    this->declare_parameter("control_topic", control_topic_);
    control_topic_ = this->get_parameter("control_topic").as_string();
    if (odom_topic_.empty() || map_topic_.empty() || scan_topic_.empty() ||
        dynamic_map_topic_.empty() || drive_topic_.empty() ||
        control_topic_.empty())
    {
        throw std::invalid_argument(
            "Bad configuration. ROS topic names must not be empty.");
    }

    this->declare_parameter("start_on_launch", start_on_launch_);
    start_on_launch_ = this->get_parameter("start_on_launch").as_bool();
    is_vehicle_enabled_ = start_on_launch_;

    this->declare_parameter("waypoint_file_path", waypoint_file_path_);
    waypoint_file_path_ =
        this->get_parameter("waypoint_file_path").as_string();
}

void RRT::load_global_waypoints()
{
    CSVHandler csv_handler(waypoint_file_path_);
    try
    {
        global_waypoints_ = csv_handler.read_waypoint_list_from_csv();
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(this->get_logger(), "Waypoint load failed: %s", error.what());
    }
}

void RRT::initialize_algorithm_modules()
{
    rrt_star::PlannerConfig planner_config;
    planner_config.minimum_iterations = minimum_rrt_iterations_;
    planner_config.maximum_iterations = maximum_rrt_iterations_;
    planner_config.sample_standard_deviation = sample_standard_deviation_;
    planner_config.step_size = step_size_;
    planner_config.near_radius = near_range_;
    planner_config.goal_tolerance = goal_tolerance_;
    planner_config.goal_sample_rate = goal_sample_rate_;
    planner_config.static_margin = map_inflation_margin_;
    planner_config.dynamic_margin = detected_obstacle_margin_;
    planner_ = std::make_unique<rrt_star::Planner>(planner_config);
    goal_selector_ =
        std::make_unique<goal_selection::Selector>(goal_ahead_distance_);
}

void RRT::initialize_ros_interfaces()
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    laser_frame_ = (this->get_namespace() + laser_frame_).substr(1);
    vehicle_frame_ = (this->get_namespace() + vehicle_frame_).substr(1);

    const auto map_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, map_qos,
        std::bind(&RRT::map_callback, this, std::placeholders::_1));
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, 1,
        std::bind(&RRT::scan_callback, this, std::placeholders::_1));
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 1,
        std::bind(&RRT::odom_callback, this, std::placeholders::_1));
    control_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        control_topic_, 10,
        std::bind(&RRT::control_callback, this, std::placeholders::_1));

    dynamic_map_publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(dynamic_map_topic_, 1);
    path_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>("path", 1);
    tree_node_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>("tree_nodes", 1);
    tree_branch_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>("tree_branches", 1);
    goal_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>("goal", 10);
    const auto waypoint_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    waypoint_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "global_waypoints", waypoint_qos);
    lookahead_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>("lookahead", 10);
    drive_publisher_ =
        this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic_, 1);

    initialize_visualization();
}

void RRT::initialize_visualization()
{
    std_msgs::msg::ColorRGBA red;
    red.r = 1.0;
    red.a = 1.0;
    std_msgs::msg::ColorRGBA green;
    green.g = 1.0;
    green.a = 1.0;
    std_msgs::msg::ColorRGBA blue;
    blue.b = 1.0;
    blue.a = 1.0;
    std_msgs::msg::ColorRGBA yellow;
    yellow.r = 1.0;
    yellow.g = 0.75;
    yellow.a = 1.0;

    goal_visualizer_ = std::make_unique<MarkerVisualizer>(
        goal_publisher_, "goal", map_frame_, green, 0.3,
        visualization_msgs::msg::Marker::SPHERE);
    lookahead_visualizer_ = std::make_unique<MarkerVisualizer>(
        lookahead_publisher_, "lookahead", map_frame_, red, 0.2,
        visualization_msgs::msg::Marker::SPHERE);
    global_waypoints_visualizer_ = std::make_unique<PointsVisualizer>(
        waypoint_publisher_, "global_waypoints", map_frame_, yellow, 0.08);
    for (const auto& waypoint : global_waypoints_)
    {
        global_waypoints_visualizer_->add_point(waypoint);
    }
    global_waypoints_visualizer_->publish_points(false);
    global_waypoints_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        [this]() {global_waypoints_visualizer_->publish_points(false);});

    tree_nodes_.header.frame_id = map_frame_;
    tree_branches_.header.frame_id = map_frame_;
    tree_nodes_.ns = "nodes";
    tree_branches_.ns = "branch";
    tree_nodes_.action = visualization_msgs::msg::Marker::ADD;
    tree_branches_.action = visualization_msgs::msg::Marker::ADD;
    tree_nodes_.pose.orientation.w = 1.0;
    tree_branches_.pose.orientation.w = 1.0;
    tree_nodes_.id = 5;
    tree_branches_.id = 6;
    tree_nodes_.type = visualization_msgs::msg::Marker::POINTS;
    tree_branches_.type = visualization_msgs::msg::Marker::LINE_LIST;
    tree_nodes_.scale.x = 0.05;
    tree_nodes_.scale.y = 0.05;
    tree_nodes_.scale.z = 0.05;
    tree_branches_.scale.x = 0.01;
    tree_nodes_.color = red;
    tree_branches_.color = blue;
}

void RRT::control_callback(const std_msgs::msg::String::ConstSharedPtr message)
{
    if (message->data == "start")
    {
        is_vehicle_enabled_ = true;
        RCLCPP_INFO(this->get_logger(), "Vehicle motion enabled.");
    }
    else if (message->data == "stop")
    {
        is_vehicle_enabled_ = false;
        stop_vehicle();
        RCLCPP_INFO(
            this->get_logger(),
            "Vehicle stopped. Planning and visualization remain active.");
    }
    else
    {
        RCLCPP_WARN(
            this->get_logger(), "Invalid control command '%s'.",
            message->data.c_str());
    }
}

void RRT::map_callback(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
{
    obstacle_map_.initialize(*message, map_inflation_margin_);
    goal_selector_->reset();
    previous_obstacle_clear_time_ = this->get_clock()->now();
    map_subscriber_.reset();
    RCLCPP_INFO(this->get_logger(), "Initial map received and inflated.");
}

bool RRT::lookup_transforms()
{
    try
    {
        laser_to_map_ = tf_buffer_->lookupTransform(
            map_frame_, laser_frame_, tf2::TimePointZero);
        map_to_vehicle_ = tf_buffer_->lookupTransform(
            vehicle_frame_, map_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& error)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Required TF unavailable: %s", error.what());
        return false;
    }
    return true;
}

geometry_msgs::msg::Point RRT::laser_point_to_map(
    const geometry_msgs::msg::Point& laser_point) const
{
    geometry_msgs::msg::PointStamped source;
    geometry_msgs::msg::PointStamped target;
    source.point = laser_point;
    tf2::doTransform(source, target, laser_to_map_);
    return target.point;
}

geometry_msgs::msg::Point RRT::map_point_to_vehicle(
    const geometry_msgs::msg::Point& map_point) const
{
    geometry_msgs::msg::PointStamped source;
    geometry_msgs::msg::PointStamped target;
    source.point = map_point;
    tf2::doTransform(source, target, map_to_vehicle_);
    return target.point;
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
{
    if (!obstacle_map_.initialized() || !lookup_transforms())
    {
        return;
    }

    const rclcpp::Time now = this->get_clock()->now();
    if ((now - previous_obstacle_clear_time_).seconds() > 0.5)
    {
        obstacle_map_.clear_observations();
        previous_obstacle_clear_time_ = now;
    }

    for (const auto& laser_point :
         scan_processing::valid_hit_points(*message, scan_range_))
    {
        obstacle_map_.add_observation(
            laser_point_to_map(laser_point), detected_obstacle_margin_);
    }
    dynamic_map_publisher_->publish(obstacle_map_.collision_map());
}

void RRT::log_goal_failure(const goal_selection::Status status)
{
    const char* reason = "unknown goal-selection error";
    switch (status)
    {
        case goal_selection::Status::empty_waypoints:
            reason = "global waypoint list is empty";
            break;
        case goal_selection::Status::no_waypoint_ahead:
            reason = "no waypoint is ahead of the vehicle";
            break;
        case goal_selection::Status::closest_waypoint_too_far:
            reason = "closest forward waypoint is outside the goal range";
            break;
        case goal_selection::Status::no_free_waypoint_in_range:
            reason = "no collision-free forward waypoint is in range";
            break;
        case goal_selection::Status::success:
            return;
    }
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Cannot select planning goal: %s.", reason);
}

void RRT::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
{
    current_pose_ = message->pose.pose;
    if (!obstacle_map_.initialized())
    {
        return;
    }
    if (!lookup_transforms())
    {
        stop_vehicle();
        return;
    }

    const goal_selection::Result goal_result = goal_selector_->update(
        global_waypoints_, current_pose_, obstacle_map_.collision_map(),
        map_to_vehicle_);
    if (!goal_result.index)
    {
        log_goal_failure(goal_result.status);
        stop_vehicle();
        return;
    }

    const std::size_t goal_index = *goal_result.index;
    visualize_goal(goal_index);
    const auto& goal_point = global_waypoints_.at(goal_index);
    const rrt_star::PlanResult plan = planner_->plan(
        {current_pose_.position.x, current_pose_.position.y},
        {goal_point.x, goal_point.y}, obstacle_map_.collision_map(),
        obstacle_map_.base_map());

    if (!plan.success)
    {
        const bool start_occupied = occupancy_grid::is_xy_coord_occupied(
            obstacle_map_.collision_map(), current_pose_.position.x,
            current_pose_.position.y);
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Could not find a path. Tree nodes: %zu, goal candidates: %zu, "
            "start occupied: %s, sampling failed: %s.",
            plan.tree.size(), plan.goal_candidate_count,
            start_occupied ? "true" : "false",
            plan.failure == rrt_star::PlanFailure::sampling_failed ?
                "true" : "false");
        stop_vehicle();
        return;
    }

    const auto raw_points = path_processing::nodes_to_points(plan.path);
    const auto path_points = path_processing::resample_polyline(
        raw_points, rrt_waypoint_interval_);
    const nav_msgs::msg::Path path =
        path_processing::to_path_message(path_points, map_frame_);
    publish_path_marker(path_points);
    follow_path(path);
    visualize_tree(plan.tree);
}

void RRT::follow_path(const nav_msgs::msg::Path& path)
{
    const auto target =
        path_tracking::point_at_distance(path, lookahead_distance_);
    if (!target)
    {
        RCLCPP_WARN(this->get_logger(), "Cannot follow an empty path.");
        stop_vehicle();
        return;
    }

    const geometry_msgs::msg::Point vehicle_target =
        map_point_to_vehicle(*target);
    const double steering = path_tracking::pure_pursuit_steering(
        vehicle_target.y, lookahead_distance_, pursuit_gain_, steering_limit_);

    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = this->now();
    command.drive.speed = path_tracking::speed_for_steering(steering);
    command.drive.steering_angle = steering;
    command.drive.steering_angle_velocity = 1.0;
    if (is_vehicle_enabled_)
    {
        drive_publisher_->publish(command);
    }
    else
    {
        stop_vehicle();
    }

    geometry_msgs::msg::Pose lookahead_pose;
    lookahead_pose.orientation.w = 1.0;
    lookahead_pose.position = *target;
    lookahead_visualizer_->set_pose(lookahead_pose);
    lookahead_visualizer_->publish_marker();
}

void RRT::stop_vehicle()
{
    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = this->now();
    command.drive.speed = 0.0;
    command.drive.steering_angle = 0.0;
    drive_publisher_->publish(command);
}

void RRT::visualize_goal(const std::size_t goal_index)
{
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position = global_waypoints_.at(goal_index);
    goal_visualizer_->set_pose(pose);
    goal_visualizer_->publish_marker();
}

void RRT::publish_path_marker(
    const std::vector<geometry_msgs::msg::Point>& points)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.id = 20;
    marker.ns = "path";
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale.x = 0.05;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    marker.points = points;
    path_publisher_->publish(marker);
}

void RRT::visualize_tree(const rrt_star::Tree& tree)
{
    tree_nodes_.points.clear();
    tree_branches_.points.clear();
    for (const auto& node : tree)
    {
        geometry_msgs::msg::Point parent;
        parent.x = node.x;
        parent.y = node.y;
        tree_nodes_.points.emplace_back(parent);
        for (const std::size_t child_index : node.children)
        {
            tree_branches_.points.emplace_back(parent);
            geometry_msgs::msg::Point child;
            child.x = tree.at(child_index).x;
            child.y = tree.at(child_index).y;
            tree_branches_.points.emplace_back(child);
        }
    }
    tree_branch_publisher_->publish(tree_branches_);
    tree_node_publisher_->publish(tree_nodes_);
}
