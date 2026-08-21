#include "motion_planning/dynamic_obstacle_map.hpp"
#include "motion_planning/goal_selector.hpp"
#include "motion_planning/path_processing.hpp"
#include "motion_planning/rrt_star_planner.hpp"
#include "motion_planning/rrt_tree.hpp"
#include "motion_planning/scan_processing.hpp"

#include "gtest/gtest.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

nav_msgs::msg::OccupancyGrid make_free_grid()
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = 20;
    grid.info.height = 20;
    grid.info.resolution = 0.5;
    grid.info.origin.position.x = -5.0;
    grid.info.origin.position.y = -5.0;
    grid.data.resize(grid.info.width * grid.info.height, 0);
    return grid;
}

geometry_msgs::msg::Point point(const double x, const double y)
{
    geometry_msgs::msg::Point result;
    result.x = x;
    result.y = y;
    return result;
}

}  // namespace

TEST(RrtTree, SteersByAtMostConfiguredStep)
{
    rrt_star::Node source;
    const rrt_star::Node result =
        rrt_star::steer_towards(source, {3.0, 4.0}, 2.5);

    EXPECT_DOUBLE_EQ(1.5, result.x);
    EXPECT_DOUBLE_EQ(2.0, result.y);
}

TEST(RrtTree, ReparentMaintainsLinksAndDescendantCosts)
{
    rrt_star::Tree tree(5);
    tree.at(0).is_root = true;
    tree.at(0).children = {1, 2};
    tree.at(1).x = 1.0;
    tree.at(1).parent = 0;
    tree.at(1).cost = 1.0;
    tree.at(1).children = {3};
    tree.at(2).y = 1.0;
    tree.at(2).parent = 0;
    tree.at(2).cost = 1.0;
    tree.at(3).x = 2.0;
    tree.at(3).parent = 1;
    tree.at(3).cost = 2.0;
    tree.at(3).children = {4};
    tree.at(4).x = 3.0;
    tree.at(4).parent = 3;
    tree.at(4).cost = 3.0;

    rrt_star::reparent_node(tree, 3, 2, 2.5);

    EXPECT_TRUE(tree.at(1).children.empty());
    EXPECT_NE(
        tree.at(2).children.end(),
        std::find(tree.at(2).children.begin(), tree.at(2).children.end(), 3));
    EXPECT_EQ(2u, tree.at(3).parent);
    EXPECT_DOUBLE_EQ(2.5, tree.at(3).cost);
    EXPECT_DOUBLE_EQ(3.5, tree.at(4).cost);
}

TEST(PathProcessing, ResamplingBoundsSpacingAndPreservesEndpoints)
{
    const std::vector<geometry_msgs::msg::Point> input = {
        point(0.0, 0.0), point(1.0, 0.0)};

    const auto result = path_processing::resample_polyline(input, 0.3);

    ASSERT_EQ(5u, result.size());
    EXPECT_DOUBLE_EQ(0.0, result.front().x);
    EXPECT_DOUBLE_EQ(1.0, result.back().x);
    for (std::size_t i = 1; i < result.size(); ++i)
    {
        EXPECT_LE(result.at(i).x - result.at(i - 1).x, 0.3);
    }
}

TEST(ScanProcessing, FiltersInvalidAndFarRanges)
{
    constexpr double half_pi = 1.57079632679489661923;
    sensor_msgs::msg::LaserScan scan;
    scan.angle_min = 0.0;
    scan.angle_increment = static_cast<float>(half_pi);
    scan.ranges = {
        1.0F, std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::quiet_NaN(), 5.0F};

    const auto hits = scan_processing::valid_hit_points(scan, 2.0);

    ASSERT_EQ(1u, hits.size());
    EXPECT_NEAR(1.0, hits.front().x, 1e-6);
    EXPECT_NEAR(0.0, hits.front().y, 1e-6);
}

TEST(DynamicObstacleMap, ClearingObservationRestoresStaticLayer)
{
    dynamic_obstacles::MapLayer layer;
    const auto grid = make_free_grid();
    layer.initialize(grid, 0.0);

    EXPECT_GT(layer.add_observation(point(0.0, 0.0), 0.5), 0u);
    EXPECT_GT(layer.collision_map().data.at(210), 50);

    layer.clear_observations();

    EXPECT_EQ(0, layer.collision_map().data.at(210));
}

TEST(GoalSelector, ChoosesForwardWaypointNearConfiguredRange)
{
    goal_selection::Selector selector(3.5);
    const std::vector<geometry_msgs::msg::Point> waypoints = {
        point(-1.0, 0.0), point(1.0, 0.0), point(2.0, 0.0), point(3.4, 0.0)};
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    geometry_msgs::msg::TransformStamped map_to_vehicle;
    map_to_vehicle.transform.rotation.w = 1.0;

    const auto result = selector.update(
        waypoints, pose, make_free_grid(), map_to_vehicle);

    ASSERT_TRUE(result.index.has_value());
    EXPECT_EQ(goal_selection::Status::success, result.status);
    EXPECT_EQ(3u, *result.index);
}

TEST(RrtStarPlanner, FindsDirectPathInFreeMap)
{
    rrt_star::PlannerConfig config;
    config.minimum_iterations = 1;
    config.maximum_iterations = 2;
    config.sample_standard_deviation = 1.0;
    config.step_size = 1.0;
    config.near_radius = 1.5;
    config.goal_tolerance = 0.2;
    config.goal_sample_rate = 1.0;
    config.static_margin = 0.0;
    config.dynamic_margin = 0.0;
    rrt_star::Planner planner(config, 7u);
    const auto grid = make_free_grid();

    const auto result = planner.plan({0.0, 0.0}, {1.0, 0.0}, grid, grid);

    ASSERT_TRUE(result.success);
    ASSERT_EQ(2u, result.path.size());
    EXPECT_DOUBLE_EQ(0.0, result.path.front().x);
    EXPECT_DOUBLE_EQ(1.0, result.path.back().x);
}
