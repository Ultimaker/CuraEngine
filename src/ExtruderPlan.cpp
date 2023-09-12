// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "ExtruderPlan.h"

namespace cura
{
ExtruderPlan::ExtruderPlan(
    const size_t extruder,
    const LayerIndex layer_nr,
    const bool is_initial_layer,
    const bool is_raft_layer,
    const coord_t layer_thickness,
    const FanSpeedLayerTimeSettings& fan_speed_layer_time_settings,
    const RetractionConfig& retraction_config)
    : extruder_nr(extruder)
    , layer_nr(layer_nr)
    , is_initial_layer(is_initial_layer)
    , is_raft_layer(is_raft_layer)
    , layer_thickness(layer_thickness)
    , fan_speed_layer_time_settings(fan_speed_layer_time_settings)
    , retraction_config(retraction_config)
{
}

void ExtruderPlan::insertCommand(NozzleTempInsert&& insert)
{
    inserts.emplace_back(insert);
}

void ExtruderPlan::handleInserts(const size_t path_idx, GCodeExport& gcode, const double cumulative_path_time)
{
    while (! inserts.empty() && path_idx >= inserts.front().path_idx && inserts.front().time_after_path_start < cumulative_path_time)
    { // handle the Insert to be inserted before this path_idx (and all inserts not handled yet)
        inserts.front().write(gcode);
        inserts.pop_front();
    }
}

void ExtruderPlan::handleAllRemainingInserts(GCodeExport& gcode)
{
    while (! inserts.empty())
    { // handle the Insert to be inserted before this path_idx (and all inserts not handled yet)
        NozzleTempInsert& insert = inserts.front();
        insert.write(gcode);
        inserts.pop_front();
    }
}

void ExtruderPlan::setFanSpeed(double _fan_speed)
{
    fan_speed = _fan_speed;
}
double ExtruderPlan::getFanSpeed()
{
    return fan_speed;
}

void ExtruderPlan::applyBackPressureCompensation(const Ratio back_pressure_compensation)
{
    constexpr double epsilon_speed_factor = 0.001; // Don't put on actual 'limit double minimum', because we don't want printers to stall.
    for (auto& path : paths)
    {
        const double nominal_width_for_path = static_cast<double>(path.config.getLineWidth());
        if (path.width_factor <= 0.0 || nominal_width_for_path <= 0.0 || path.config.isTravelPath() || path.config.isBridgePath())
        {
            continue;
        }
        const double line_width_for_path = path.width_factor * nominal_width_for_path;
        path.speed_back_pressure_factor = std::max(epsilon_speed_factor, 1.0 + (nominal_width_for_path / line_width_for_path - 1.0) * back_pressure_compensation);
    }
}
} // namespace cura