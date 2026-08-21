#include "motion_planning/occupancy_grid.hpp"

#include "gtest/gtest.h"

nav_msgs::msg::OccupancyGrid create_grid(const int width, const int height)
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.resolution = 1.0;
    grid.info.origin.position.x = -1.0;
    grid.info.origin.position.y = -2.0;
    grid.data.resize(width * height, 0);
    return grid;
}

TEST(OccupancyGrid, ConvertsCoordinatesWithinBounds)
{
    auto grid = create_grid(4, 3);

    EXPECT_EQ(0, occupancy_grid::xy_coord_to_array_index(grid, -1.0, -2.0));
    EXPECT_EQ(1, occupancy_grid::xy_coord_to_array_index(grid, 0.0, -2.0));
    EXPECT_EQ(11, occupancy_grid::xy_coord_to_array_index(grid, 2.9, 0.9));
}

TEST(OccupancyGrid, RejectsCoordinatesOutsideBounds)
{
    auto grid = create_grid(4, 3);

    EXPECT_EQ(-1, occupancy_grid::xy_coord_to_array_index(grid, -1.01, -2.0));
    EXPECT_EQ(-1, occupancy_grid::xy_coord_to_array_index(grid, 3.0, -2.0));
    EXPECT_EQ(-1, occupancy_grid::xy_coord_to_array_index(grid, -1.0, -2.01));
    EXPECT_EQ(-1, occupancy_grid::xy_coord_to_array_index(grid, -1.0, 1.0));
    EXPECT_TRUE(occupancy_grid::is_xy_coord_occupied(grid, -1.01, -2.0));
}

TEST(OccupancyGrid, InflatesSymmetrically)
{
    auto grid = create_grid(5, 5);
    const auto changes = occupancy_grid::inflate_cell(grid, 12, 1.0, 100);

    EXPECT_EQ(9u, changes.size());
    for (int y = 1; y <= 3; y++)
    {
        for (int x = 1; x <= 3; x++)
        {
            EXPECT_EQ(100, grid.data.at(y * 5 + x));
        }
    }
}

TEST(OccupancyGrid, InflatesLastRowAndColumn)
{
    auto grid = create_grid(5, 5);
    const auto changes = occupancy_grid::inflate_cell(grid, 24, 1.0, 100);

    EXPECT_EQ(4u, changes.size());
    EXPECT_EQ(100, grid.data.at(18));
    EXPECT_EQ(100, grid.data.at(19));
    EXPECT_EQ(100, grid.data.at(23));
    EXPECT_EQ(100, grid.data.at(24));
}
