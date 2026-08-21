#ifndef MOTION_PLANNING__OCCUPANCY_GRID_HPP_
#define MOTION_PLANNING__OCCUPANCY_GRID_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"

#include <utility>
#include <vector>

/** Coordinate conversion, occupancy queries, and inflation primitives. */
namespace occupancy_grid
{

constexpr int OCCUPIED_THRESHOLD = 50;

/**
 * Input: grid and integer cell coordinates.
 * Return: row-major data index, or -1 when the cell is outside/invalid.
 */
int xy_index_to_array_index(
    const nav_msgs::msg::OccupancyGrid& grid, int x_index, int y_index);

/**
 * Input: grid and continuous map-frame coordinate.
 * Return: containing cell's row-major data index, or -1 when outside/invalid.
 */
int xy_coord_to_array_index(
    const nav_msgs::msg::OccupancyGrid& grid, float x, float y);

/**
 * Input: grid and valid row-major data index.
 * Return: integer x/y cell indices. Callers must validate the input index.
 */
std::pair<int, int> array_index_to_xy_index(
    const nav_msgs::msg::OccupancyGrid& grid, int array_index);

/** Return the map-frame x coordinate of a valid data index's cell origin. */
float array_index_to_x_coord(
    const nav_msgs::msg::OccupancyGrid& grid, int array_index);

/** Return the map-frame y coordinate of a valid data index's cell origin. */
float array_index_to_y_coord(
    const nav_msgs::msg::OccupancyGrid& grid, int array_index);

/**
 * Input: grid and map-frame coordinate.
 * Return: true for occupied cells and for coordinates outside the grid. This
 * fail-closed behavior prevents the planner from leaving the known map.
 */
bool is_xy_coord_occupied(
    const nav_msgs::msg::OccupancyGrid& grid, float x, float y);

/** Mark the containing cell fully occupied; outside coordinates do nothing. */
void set_xy_coord_occupied(
    nav_msgs::msg::OccupancyGrid& grid, float x, float y);

/**
 * Inflate one valid cell by a square margin.
 *
 * Input: mutable grid, center data index, non-negative metric margin, and
 * value to write into previously free cells.
 * Return: indices whose values were changed. Invalid input returns an empty
 * list and leaves the grid unchanged.
 */
std::vector<int> inflate_cell(
    nav_msgs::msg::OccupancyGrid& grid, int array_index, float margin, int value);

/**
 * Inflate every occupied source cell in place by a non-negative metric margin.
 * Input/output: the supplied grid is permanently modified.
 */
void inflate_map(nav_msgs::msg::OccupancyGrid& grid, float margin);

}  // namespace occupancy_grid

#endif  // MOTION_PLANNING__OCCUPANCY_GRID_HPP_
