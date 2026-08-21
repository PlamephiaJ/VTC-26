#include "motion_planning/rrt_star_planner.hpp"

#include "motion_planning/collision_checker.hpp"
#include "motion_planning/occupancy_grid.hpp"

#include <limits>
#include <stdexcept>

namespace rrt_star
{

namespace
{

void validate_config(const PlannerConfig& config)
{
    if (config.minimum_iterations <= 0 ||
        config.maximum_iterations < config.minimum_iterations ||
        config.sample_standard_deviation <= 0.0 || config.step_size <= 0.0 ||
        config.near_radius <= 0.0 || config.goal_tolerance <= 0.0 ||
        config.goal_sample_rate < 0.0 || config.goal_sample_rate > 1.0 ||
        config.static_margin < 0.0 || config.dynamic_margin < 0.0)
    {
        throw std::invalid_argument("Invalid RRT* planner configuration");
    }
}

}  // namespace

Planner::Planner(
    const PlannerConfig& config,
    const std::optional<std::uint32_t> random_seed)
    : config_(config),
      random_generator_(random_seed.value_or(std::random_device{}()))
{
    validate_config(config_);
}

std::optional<Point2D> Planner::sample_free_point(
    const Point2D& start, const Point2D& goal,
    const nav_msgs::msg::OccupancyGrid& dynamic_map)
{
    if (unit_distribution_(random_generator_) < config_.goal_sample_rate &&
        !occupancy_grid::is_xy_coord_occupied(dynamic_map, goal.x, goal.y))
    {
        return goal;
    }

    const double center_x = 0.6 * goal.x + 0.4 * start.x;
    const double center_y = 0.6 * goal.y + 0.4 * start.y;
    std::normal_distribution<double> x_distribution(
        center_x, config_.sample_standard_deviation);
    std::normal_distribution<double> y_distribution(
        center_y, config_.sample_standard_deviation);

    constexpr int maximum_attempts = 1000;
    for (int attempt = 0; attempt < maximum_attempts; ++attempt)
    {
        const Point2D sample{
            x_distribution(random_generator_), y_distribution(random_generator_)};
        if (!occupancy_grid::is_xy_coord_occupied(dynamic_map, sample.x, sample.y))
        {
            return sample;
        }
    }
    return std::nullopt;
}

PlanResult Planner::plan(
    const Point2D& start, const Point2D& goal,
    const nav_msgs::msg::OccupancyGrid& dynamic_map,
    const nav_msgs::msg::OccupancyGrid& static_map)
{
    PlanResult result;
    Node root;
    root.x = start.x;
    root.y = start.y;
    root.parent = 0;
    root.cost = 0.0;
    root.is_root = true;
    result.tree.emplace_back(root);

    std::vector<std::size_t> goal_candidates;
    for (int iteration = 0; iteration < config_.maximum_iterations; ++iteration)
    {
        const auto sample = sample_free_point(start, goal, dynamic_map);
        if (!sample)
        {
            result.failure = PlanFailure::sampling_failed;
            break;
        }

        const std::size_t nearest = nearest_index(result.tree, *sample);
        Node new_node = steer_towards(
            result.tree.at(nearest), *sample, config_.step_size);
        if (edge_length(result.tree.at(nearest), new_node) <=
            std::numeric_limits<double>::epsilon())
        {
            continue;
        }

        if (collision_checker::edge_is_blocked(
                dynamic_map, static_map, result.tree.at(nearest), new_node,
                config_.static_margin, config_.dynamic_margin))
        {
            continue;
        }

        const std::vector<std::size_t> near_nodes =
            near_indices(result.tree, new_node, config_.near_radius);
        std::size_t best_parent = nearest;
        double best_cost = result.tree.at(nearest).cost +
            edge_length(result.tree.at(nearest), new_node);

        for (const std::size_t candidate : near_nodes)
        {
            if (collision_checker::edge_is_blocked(
                    dynamic_map, static_map, result.tree.at(candidate), new_node,
                    config_.static_margin, config_.dynamic_margin))
            {
                continue;
            }
            const double candidate_cost = result.tree.at(candidate).cost +
                edge_length(result.tree.at(candidate), new_node);
            if (candidate_cost < best_cost)
            {
                best_parent = candidate;
                best_cost = candidate_cost;
            }
        }

        new_node.parent = best_parent;
        new_node.cost = best_cost;
        const std::size_t new_index = result.tree.size();
        result.tree.emplace_back(new_node);
        result.tree.at(best_parent).children.emplace_back(new_index);

        for (const std::size_t near_index : near_nodes)
        {
            const double rewired_cost = result.tree.at(new_index).cost +
                edge_length(result.tree.at(new_index), result.tree.at(near_index));
            if (rewired_cost >= result.tree.at(near_index).cost ||
                collision_checker::edge_is_blocked(
                    dynamic_map, static_map, result.tree.at(near_index),
                    result.tree.at(new_index), config_.static_margin,
                    config_.dynamic_margin))
            {
                continue;
            }
            reparent_node(result.tree, near_index, new_index, rewired_cost);
        }

        if (is_within_goal(result.tree.back(), goal, config_.goal_tolerance))
        {
            goal_candidates.emplace_back(new_index);
        }

        if (iteration + 1 >= config_.minimum_iterations && !goal_candidates.empty())
        {
            const std::size_t best =
                best_goal_candidate(result.tree, goal_candidates, goal);
            result.path = trace_path(result.tree, best);
            result.goal_candidate_count = goal_candidates.size();
            result.success = true;
            result.failure = PlanFailure::none;
            return result;
        }
    }

    result.goal_candidate_count = goal_candidates.size();
    return result;
}

}  // namespace rrt_star
