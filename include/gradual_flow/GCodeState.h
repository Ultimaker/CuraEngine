// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <algorithm>
#include <cassert>
#include <list>
#include <vector>

#include "gradual_flow/utils/Direction.h"
#include "gradual_flow/FlowLimitedPath.h"
#include "gradual_flow/FlowState.h"

namespace curaengine::gradual_flow
{
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
} // namespace curaengine::gradual_flow
