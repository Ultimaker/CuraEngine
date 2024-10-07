// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GRADUAL_FLOW_GCODE_PATH_H
#define GRADUAL_FLOW_GCODE_PATH_H

#include <optional>
#include <vector>

#include <fmt/format.h>
#include <range/v3/all.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/take.hpp>
#include <spdlog/spdlog.h>

#include "geometry/PointsSet.h"
#include "gradual_flow/Utils.h"
#include "pathPlanning/GCodePath.h"

namespace cura::gradual_flow
{

enum class FlowState
{
    STABLE,
    TRANSITION,
    UNDEFINED
};

struct FlowLimitedPath
{
    const GCodePath* original_gcode_path_data;
    std::vector<Point3LL> points{};
    double speed{ targetSpeed() }; // um/s
    double flow_{ extrusionVolumePerMm() * speed }; // um/s
    double total_length{ totalLength() }; // um

    double targetSpeed() const // um/s
    {
        return original_gcode_path_data->config.speed_derivatives.speed * original_gcode_path_data->speed_factor * 1e3;
    }

    /*
     * Returns if the path is a travel move.
     *
     * @return `true` If the path is a travel move, `false` otherwise
     */
    bool isTravel() const
    {
        return targetFlow() <= 0;
    }

    /*
     * Returns if the path is a retract move.
     *
     * @return `true` If the path is a retract move, `false` otherwise
     */
    bool isRetract() const
    {
        return original_gcode_path_data->retract;
    }

    /*
     * Returns the extrusion volume per um of the path.
     *
     * @return The extrusion volume per um of the path in um^3/um
     */
    double extrusionVolumePerMm() const // um^3/um
    {
        return original_gcode_path_data->flow * original_gcode_path_data->config.line_width * original_gcode_path_data->config.layer_thickness * original_gcode_path_data->flow;
    }

    /*
     * Returns the extrusion volume of the path.
     *
     * @return The extrusion volume of the path in um^3/s
     */
    double flow() const // um^3/s
    {
        return flow_;
    }

    /*
     * Returns the target extrusion volume of the path.
     *
     * @return The target extrusion volume of the path in um^3/s
     */
    double targetFlow() const // um^3/s
    {
        return extrusionVolumePerMm() * targetSpeed();
    }

    /*
     * Returns the path as an SVG path data string.
     *
     * @return the SVG path data string
     */
    std::string toSvgPathData() const
    {
        std::string path_data;
        auto is_first_point = true;
        for (auto point : points)
        {
            const auto identifier = is_first_point ? "M" : "L";
            path_data += fmt::format("{}{} {} ", identifier, point.x_ * 1e-3, point.y_ * 1e-3);
            is_first_point = false;
        }
        return path_data;
    }

    /*
     * Returns the path as an SVG path-element.
     *
     * @return the SVG path
     */
    std::string toSvgPath()
    {
        const auto path_data = toSvgPathData();

        if (isTravel())
        {
            return fmt::format("<path d=\"{}\" fill=\"none\" stroke=\"black\" stroke-width=\"0.05\" />", path_data);
        }

        const auto [r, g, b] = gradual_flow::utils::hsvToRgb(flow() * .00000003, 100., 100.);
        const auto color = fmt::format("rgb({},{},{})", r, g, b);
        return fmt::format("<path d=\"{}\" fill=\"none\" stroke=\"{}\" stroke-width=\"0.1\" />", path_data, color);
    }

    /*
     * Returns the total length of the path.
     *
     * @return the length in um
     */
    double totalLength() const // um
    {
        double path_length = 0;
        if (! points.empty())
        {
            auto last_point = points.front();
            for (const auto& point : points | ranges::views::drop(1))
            {
                path_length += std::hypot(point.x_ - last_point.x_, point.y_ - last_point.y_);
                last_point = point;
            }
        }
        return path_length;
    }

    /*
     * Returns the total duration of the path.
     *
     * @return the duration in seconds
     */
    double totalDuration() const // s
    {
        return total_length / speed;
    }

    /*
     * Splits either the beginning or the end of the path into a new path.
     *
     * @param partition_duration duration of the partitioned paths in s
     * @param partition_speed speed of the partitioned paths in um/s
     * @param direction
     * @return a tuple of the partitioned path and the remaining path, the
     * remaining path can possibly be empty if the duration of the
     * partitioned path is equal or longer than the duration of the original
     * path
     */
    std::tuple<FlowLimitedPath, std::optional<FlowLimitedPath>, double>
        partition(const double partition_duration, const double partition_speed, const utils::Direction direction) const
    {
        const auto total_path_duration = total_length / partition_speed;
        if (partition_duration >= total_path_duration)
        {
            const auto remaining_partition_duration = partition_duration - total_path_duration;
            const FlowLimitedPath gcode_path{ .original_gcode_path_data = original_gcode_path_data, .points = points, .speed = partition_speed };
            return std::make_tuple(gcode_path, std::nullopt, remaining_partition_duration);
        }

        auto current_partition_duration = 0.0;
        auto partition_index = direction == utils::Direction::Forward ? 0 : points.size() - 1;
        auto iteration_direction = direction == utils::Direction::Forward ? 1 : -1;
        Point3LL prev_point = points[partition_index];

        while (true)
        {
            const Point3LL next_point = points[partition_index + iteration_direction];
            const auto segment_length = std::hypot(next_point.x_ - prev_point.x_, next_point.y_ - prev_point.y_);
            const auto segment_duration = segment_length / partition_speed;

            if (current_partition_duration + segment_duration < partition_duration)
            {
                prev_point = next_point;
                current_partition_duration += segment_duration;
                partition_index += iteration_direction;
            }
            else
            {
                const auto duration_left = partition_duration - current_partition_duration;
                auto segment_ratio = duration_left / segment_duration;
                assert(segment_ratio >= -1e-6 && segment_ratio <= 1. + 1e-6);
                const auto partition_x = prev_point.x_ + static_cast<long long>(static_cast<double>(next_point.x_ - prev_point.x_) * segment_ratio);
                const auto partition_y = prev_point.y_ + static_cast<long long>(static_cast<double>(next_point.y_ - prev_point.y_) * segment_ratio);
                const auto partition_z = prev_point.z_ + static_cast<long long>(static_cast<double>(next_point.z_ - prev_point.z_) * segment_ratio);
                const Point3LL partition_point(partition_x, partition_y, partition_z);

                /*
                 *                       partition point
                 *                            v
                 *   0---------1---------2----x------3---------4
                 *                       ^           ^
                 *    partition index when           partition_index when
                 *          going forwards           going backwards
                 *
                 * When we partition the path in a "left" and "right" path we
                 * expect we end up with the same path for the same partition
                 * if we go forwards or backwards. This is why
                 *         partition_point_index = partition_index + 1
                 * when going _forwards_, while going _backwards_ it is equal
                 * to
                 *           partition_point_index = partition_index
                 *
                 * Given this new index every point for which
                 *                 0 >= i > partition_point_index
                 * holds belongs to the _left_ path while every point for which
                 *        partition_point_index >= i > points.size()
                 * belongs to the right path.
                 */
                const auto partition_point_index = direction == utils::Direction::Forward ? partition_index + 1 : partition_index;

                // points left of the partition_index
                std::vector<Point3LL> left_points;
                for (unsigned int i = 0; i < partition_point_index; ++i)
                {
                    left_points.emplace_back(points[i]);
                }
                left_points.emplace_back(partition_point);

                // points right of the partition_index
                std::vector<Point3LL> right_points;
                right_points.emplace_back(partition_point);
                for (unsigned int i = partition_point_index; i < points.size(); ++i)
                {
                    right_points.emplace_back(points[i]);
                }

                switch (direction)
                {
                case utils::Direction::Forward:
                {
                    const FlowLimitedPath partition_gcode_path{
                        .original_gcode_path_data = original_gcode_path_data,
                        .points = left_points,
                        .speed = partition_speed,
                    };
                    const FlowLimitedPath remaining_gcode_path{
                        .original_gcode_path_data = original_gcode_path_data,
                        .points = right_points,
                        .speed = speed,
                    };
                    return std::make_tuple(partition_gcode_path, remaining_gcode_path, .0);
                };
                case utils::Direction::Backward:
                {
                    const FlowLimitedPath partition_gcode_path{
                        .original_gcode_path_data = original_gcode_path_data,
                        .points = right_points,
                        .speed = partition_speed,
                    };
                    const FlowLimitedPath remaining_gcode_path{
                        .original_gcode_path_data = original_gcode_path_data,
                        .points = left_points,
                        .speed = speed,
                    };
                    return std::make_tuple(partition_gcode_path, remaining_gcode_path, .0);
                }
                }
            }
        }
    }

    GCodePath toClassicPath(const bool include_first_point) const
    {
        GCodePath output_path = *original_gcode_path_data;

        output_path.points.clear();
        for (auto& point : points | ranges::views::drop(include_first_point ? 0 : 1))
        {
            output_path.points.push_back(point);
        }

        output_path.config.speed_derivatives.speed = speed * 1e-3;

        return output_path;
    }
};

struct GCodeState
{
    double current_flow{ 0.0 }; // um^3/s
    double flow_acceleration{ 0.0 }; // um^3/s^2
    double flow_deceleration{ 0.0 }; // um^3/s^2
    double discretized_duration{ 0.0 }; // s
    double discretized_duration_remaining{ 0.0 }; // s
    double target_end_flow{ 0.0 }; // um^3/s
    double reset_flow_duration{ 0.0 }; // s
    FlowState flow_state{ FlowState::UNDEFINED };

    std::vector<FlowLimitedPath> processGcodePaths(const std::vector<FlowLimitedPath>& gcode_paths)
    {
        // reset the discretized_duration_remaining
        discretized_duration_remaining = 0;

        std::vector<FlowLimitedPath> forward_pass_gcode_paths;
        for (auto& gcode_path : gcode_paths)
        {
            auto discretized_paths = processGcodePath(gcode_path, gradual_flow::utils::Direction::Forward);
            for (auto& path : discretized_paths)
            {
                forward_pass_gcode_paths.emplace_back(path);
            }
        }

        // reset the discretized_duration_remaining
        discretized_duration_remaining = 0;

        // set the current flow to the target end flow. When executing the backward pass we want to
        // start with this flow and gradually increase it to the target flow. However, if the
        // highest flow we can achieve is lower than this target flow we want to use that flow
        // instead.
        current_flow = std::min(current_flow, target_end_flow);

        std::list<FlowLimitedPath> backward_pass_gcode_paths;
        for (auto& gcode_path : forward_pass_gcode_paths | ranges::views::reverse)
        {
            auto discretized_paths = processGcodePath(gcode_path, gradual_flow::utils::Direction::Backward);
            for (auto& path : discretized_paths)
            {
                backward_pass_gcode_paths.emplace_front(path);
            }
        }

        return std::vector<FlowLimitedPath>(backward_pass_gcode_paths.begin(), backward_pass_gcode_paths.end());
    }

    /*
     * Discretizes a GCodePath into multiple GCodePaths with a gradual increase in flow.
     *
     * @param path the path to discretize
     *
     * @return a vector of discretized paths with a gradual increase in flow
     */
    std::vector<FlowLimitedPath> processGcodePath(const FlowLimitedPath& path, const utils::Direction direction)
    {
        if (path.isTravel())
        {
            if (path.isRetract() || path.totalDuration() > reset_flow_duration)
            {
                flow_state = FlowState::UNDEFINED;
            }
            return { path };
        }

        // After a long travel move we want to reset the flow to the target end flow
        if (flow_state == FlowState::UNDEFINED && direction == utils::Direction::Forward)
        {
            current_flow = path.targetFlow();
        }

        auto target_flow = path.flow();
        if (target_flow <= current_flow)
        {
            current_flow = target_flow;
            discretized_duration_remaining = 0;
            flow_state = FlowState::STABLE;
            return { path };
        }

        const auto extrusion_volume_per_mm = path.extrusionVolumePerMm(); // um^3/um

        std::vector<FlowLimitedPath> discretized_paths;

        FlowLimitedPath remaining_path = path;

        if (discretized_duration_remaining > 0.)
        {
            const auto discretized_segment_speed = current_flow / extrusion_volume_per_mm; // um^3/s / um^3/um = um/s
            const auto [partitioned_gcode_path, new_remaining_path, remaining_partition_duration]
                = path.partition(discretized_duration_remaining, discretized_segment_speed, direction);
            discretized_duration_remaining = std::max(.0, discretized_duration_remaining - remaining_partition_duration);
            if (new_remaining_path.has_value())
            {
                remaining_path = new_remaining_path.value();
                discretized_paths.emplace_back(partitioned_gcode_path);
            }
            else
            {
                flow_state = FlowState::TRANSITION;
                return { partitioned_gcode_path };
            }
        }

        // while we have not reached the target flow, iteratively discretize the path
        // such that the new path has a duration of discretized_duration and with each
        // iteration an increased flow of flow_acceleration
        while (current_flow < target_flow)
        {
            const auto flow_delta = (direction == utils::Forward ? flow_acceleration : flow_deceleration) * discretized_duration;
            current_flow = std::min(target_flow, current_flow + flow_delta);

            const auto segment_speed = current_flow / extrusion_volume_per_mm; // um^3/s / um^3/um = um/s

            if (current_flow == target_flow)
            {
                remaining_path.speed = segment_speed;
                discretized_duration_remaining = std::max(discretized_duration_remaining - remaining_path.totalDuration(), .0);
                flow_state = discretized_duration_remaining > 0. ? FlowState::TRANSITION : FlowState::STABLE;
                discretized_paths.emplace_back(remaining_path);
                return discretized_paths;
            }

            const auto [partitioned_gcode_path, new_remaining_path, remaining_partition_duration] = remaining_path.partition(discretized_duration, segment_speed, direction);

            // when we have remaining paths, we should have no remaining duration as the
            // remaining duration should then be consumed by the remaining paths
            assert(! new_remaining_path.has_value() || remaining_partition_duration == 0);
            // having no remaining paths implies that there is a duration remaining that should be consumed
            // by the next path
            assert(new_remaining_path.has_value() || remaining_partition_duration > 0);

            discretized_paths.emplace_back(partitioned_gcode_path);

            if (new_remaining_path.has_value())
            {
                remaining_path = new_remaining_path.value();
            }
            else
            {
                flow_state = FlowState::TRANSITION;
                discretized_duration_remaining = remaining_partition_duration;
                return discretized_paths;
            }
        }
        discretized_paths.emplace_back(remaining_path);

        flow_state = discretized_duration_remaining > 0. ? FlowState::TRANSITION : FlowState::STABLE;

        return discretized_paths;
    }
};

} // namespace cura::gradual_flow

#endif // GRADUAL_FLOW_GCODE_PATH_H
