/*
 * @Author: Yuhao Chen
 * @Date: 2024-06-23 11:22:01
 * @LastEditors: Yuhao Chen
 * @LastEditTime: 2024-06-25 11:22:09
 * @Description: Occupancy grid releated functions source file.
 */

#include "../include/motion_planning/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>

int occupancy_grid::xy_index_to_array_index(const nav_msgs::msg::OccupancyGrid& grid, const int i_x, const int i_y)
{
    if (i_x < 0 || i_y < 0 || i_x >= int(grid.info.width) || i_y >= int(grid.info.height))
    {
        return -1;
    }

    const int array_index = i_y * int(grid.info.width) + i_x;
    if (array_index >= int(grid.data.size()))
    {
        return -1;
    }
    return array_index;
}

int occupancy_grid::xy_coord_to_array_index(const nav_msgs::msg::OccupancyGrid& grid, const float x, const float y)
{
    if (grid.info.resolution <= 0.0)
    {
        return -1;
    }

    int i_x = static_cast<int>(std::floor((x - grid.info.origin.position.x) / grid.info.resolution));
    int i_y = static_cast<int>(std::floor((y - grid.info.origin.position.y) / grid.info.resolution));
    return xy_index_to_array_index(grid, i_x, i_y);
}

std::pair<int, int> occupancy_grid::array_index_to_xy_index(const nav_msgs::msg::OccupancyGrid& grid, const int i)
{
    int i_y = i / grid.info.width;
    int i_x = i - i_y * grid.info.width;
    return {i_x, i_y};
}

float occupancy_grid::array_index_to_x_coord(const nav_msgs::msg::OccupancyGrid& grid, const int i)
{
    return grid.info.origin.position.x + array_index_to_xy_index(grid, i).first * grid.info.resolution;
}

float occupancy_grid::array_index_to_y_coord(const nav_msgs::msg::OccupancyGrid& grid, const int i)
{
    return grid.info.origin.position.y + array_index_to_xy_index(grid, i).second * grid.info.resolution;
}

bool occupancy_grid::is_xy_coord_occupied(const nav_msgs::msg::OccupancyGrid& grid, const float x, const float y)
{
    const int array_index = xy_coord_to_array_index(grid, x, y);
    if (array_index < 0)
    {
        return true;
    }
    return int(grid.data.at(array_index)) > OCCUPIED_THRESHOLD;
}

void occupancy_grid::set_xy_coord_occupied(nav_msgs::msg::OccupancyGrid& grid, const float x, const float y)
{
    const int array_index = xy_coord_to_array_index(grid, x, y);
    if (array_index >= 0)
    {
        grid.data.at(array_index) = 100;
    }
}

std::vector<int> occupancy_grid::inflate_cell(nav_msgs::msg::OccupancyGrid &grid, const int i, const float margin, const int val)
{
    std::vector<int> changes;
    if (i < 0 || i >= int(grid.data.size()) || grid.info.width == 0 || grid.info.height == 0 || grid.info.resolution <= 0.0 || margin < 0.0)
    {
        return changes;
    }

    int margin_cell = static_cast<int>(std::ceil(margin / grid.info.resolution));
    std::pair<int, int> xy_index = array_index_to_xy_index(grid, i);
    for (int x = std::max(0, xy_index.first - margin_cell); x <= std::min(int(grid.info.width) - 1, xy_index.first + margin_cell); x++)
    {
        for (int y = std::max(0, xy_index.second - margin_cell); y <= std::min(int(grid.info.height) - 1, xy_index.second + margin_cell); y++)
        {
            const int array_index = xy_index_to_array_index(grid, x, y);
            if (array_index < 0)
            {
                continue;
            }
            if (grid.data.at(array_index) < OCCUPIED_THRESHOLD && grid.data.at(array_index) != val)
            {
                grid.data.at(array_index) = val;
                changes.emplace_back(array_index);
            }
        }
    }
    return changes;
}

void occupancy_grid::inflate_map(nav_msgs::msg::OccupancyGrid& grid, const float margin)
{
    std::vector<int> occupied_indices;
    int size_grid_data = grid.data.size();
    for (int i = 0; i < size_grid_data; i++)
    {
        if (grid.data.at(i) > OCCUPIED_THRESHOLD)
        {
            occupied_indices.emplace_back(i);
        }
    }
    
    int size_occupied_indices = occupied_indices.size();
    for (int i = 0; i < size_occupied_indices; i++)
    {
        inflate_cell(grid, occupied_indices.at(i), margin, 100);
    }
}

bool occupancy_grid::segment_is_blocked(
    const nav_msgs::msg::OccupancyGrid& collision_map,
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& end,
    const SegmentCheckOptions& options)
{
    if (collision_map.info.resolution <= 0.0)
    {
        return true;
    }

    const int x_steps = static_cast<int>(
        std::ceil(std::abs(start.x - end.x) / collision_map.info.resolution));
    const int y_steps = static_cast<int>(
        std::ceil(std::abs(start.y - end.y) / collision_map.info.resolution));
    const int sample_count = std::max(x_steps, y_steps);
    const bool escape_enabled = options.escape_reference_map != nullptr &&
        options.start_escape_distance > 0.0 &&
        is_xy_coord_occupied(collision_map, start.x, start.y) &&
        !is_xy_coord_occupied(
            *options.escape_reference_map, start.x, start.y);
    const double escape_distance_squared =
        options.start_escape_distance * options.start_escape_distance;

    for (int i = 0; i <= sample_count; ++i)
    {
        const double ratio = sample_count > 0 ?
            static_cast<double>(i) / sample_count : 0.0;
        const double x = start.x + ratio * (end.x - start.x);
        const double y = start.y + ratio * (end.y - start.y);
        if (!is_xy_coord_occupied(collision_map, x, y))
        {
            continue;
        }

        const double dx = x - start.x;
        const double dy = y - start.y;
        const bool inside_escape_region =
            dx * dx + dy * dy <= escape_distance_squared;
        if (escape_enabled && inside_escape_region &&
            !is_xy_coord_occupied(*options.escape_reference_map, x, y))
        {
            continue;
        }
        return true;
    }
    return false;
}

bool occupancy_grid::polyline_is_blocked(
    const nav_msgs::msg::OccupancyGrid& collision_map,
    const std::vector<geometry_msgs::msg::Point>& points)
{
    if (points.empty())
    {
        return true;
    }
    if (points.size() == 1)
    {
        return is_xy_coord_occupied(
            collision_map, points.front().x, points.front().y);
    }
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        if (segment_is_blocked(collision_map, points.at(i - 1), points.at(i)))
        {
            return true;
        }
    }
    return false;
}
