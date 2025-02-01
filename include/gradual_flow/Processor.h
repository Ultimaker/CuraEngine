// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GRADUAL_FLOW_PROCESSOR_H
#define GRADUAL_FLOW_PROCESSOR_H

#include "Application.h"
#include "LayerPlan.h"
#include "Scene.h"
#include "gradual_flow/FlowLimitedPath.h"

namespace cura::gradual_flow::Processor
{

/*!
 * \brief Processes the gradual flow acceleration splitting
 * \param extruder_plan_paths The paths of the extruder plan to be processed. I gradual flow is enabled, they will be
 *                            completely rewritten, very likely with a different amout of output paths.
 * \param extruder_nr The used extruder number
 * \param layer_nr The current layer number
 */
void process(std::vector<GCodePath>& extruder_plan_paths, const size_t extruder_nr, const size_t layer_nr)
{
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const Settings& extruder_settings = scene.extruders[extruder_nr].settings_;

    if (extruder_settings.get<bool>("gradual_flow_enabled"))
    {
        // Convert the gcode paths to a format that suits our calculations more
        std::vector<FlowLimitedPath> gcode_paths;

        // Process first path
        for (const GCodePath& path : extruder_plan_paths | ranges::views::take(1))
        {
            gcode_paths.push_back(FlowLimitedPath{ .original_gcode_path_data = &path, .points = path.points });
        }

        /* Process remaining paths
         * We need to add the last point of the previous path to the current path
         * since the paths in Cura are a connected line string and a new path begins
         * where the previous path ends (see figure below).
         *    {                Path A            } {          Path B        } { ...etc
         *    a.1-----------a.2------a.3---------a.4------b.1--------b.2--- c.1-------
         * For our purposes it is easier that each path is a separate line string, and
         * no knowledge of the previous path is needed.
         */
        for (const auto& path : extruder_plan_paths | ranges::views::drop(1))
        {
            std::vector<Point3LL> points{ gcode_paths.back().points.back() };
            points.insert(points.end(), path.points.begin(), path.points.end());

            gcode_paths.emplace_back(FlowLimitedPath{ .original_gcode_path_data = &path, .points = std::move(points) });
        }

        constexpr auto non_zero_flow_view = ranges::views::transform(
                                                [](const auto& path)
                                                {
                                                    return path.flow();
                                                })
                                          | ranges::views::drop_while(
                                                [](const auto flow)
                                                {
                                                    return flow == 0.0;
                                                });
        auto gcode_paths_non_zero_flow_view = gcode_paths | non_zero_flow_view;

        const auto flow_limit = extruder_settings.get<double>(layer_nr == 0 ? "layer_0_max_flow_acceleration" : "max_flow_acceleration") * 1e9;

        auto target_flow = ranges::empty(gcode_paths_non_zero_flow_view) ? 0.0 : ranges::front(gcode_paths_non_zero_flow_view);

        GCodeState state{
            .current_flow = target_flow,
            .flow_acceleration = flow_limit,
            .flow_deceleration = flow_limit,
            .discretized_duration = extruder_settings.get<double>("gradual_flow_discretisation_step_size"),
            // take the first path's target flow as the target flow, this might
            // not be correct, but it is safe to assume the target flow for the
            // next layer is the same as the target flow of the current layer
            .target_end_flow = target_flow,
            .reset_flow_duration = extruder_settings.get<double>("reset_flow_duration"),
        };

        const auto limited_flow_acceleration_paths = state.processGcodePaths(gcode_paths);
        // Copy newly generated paths to actual plan

        std::vector<GCodePath> new_paths;
        for (const auto& [index, gcode_path] : limited_flow_acceleration_paths | ranges::views::enumerate)
        {
            // since the first point is added from the previous path in the initial conversion,
            // we should remove it here again. Note that the first point is added for every path
            // except the first one, so we should only remove it if it is not the first path
            const auto include_first_point = index == 0;
            new_paths.push_back(gcode_path.toClassicPath(include_first_point));
        }

        extruder_plan_paths = new_paths;
    }
}
} // namespace cura::gradual_flow::Processor

#endif // GRADUAL_FLOW_PROCESSOR_H
