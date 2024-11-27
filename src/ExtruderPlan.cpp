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
    : extruder_nr_(extruder)
    , layer_nr_(layer_nr)
    , is_initial_layer_(is_initial_layer)
    , is_raft_layer_(is_raft_layer)
    , layer_thickness_(layer_thickness)
    , fan_speed_layer_time_settings_(fan_speed_layer_time_settings)
    , retraction_config_(retraction_config)
{
}

void ExtruderPlan::insertCommand(NozzleTempInsert&& insert)
{
    inserts_.emplace_back(insert);
}

void ExtruderPlan::handleInserts(const size_t path_idx, GCodeExport& gcode, const double cumulative_path_time)
{
    while (! inserts_.empty() && path_idx >= inserts_.front().path_idx && inserts_.front().time_after_path_start < cumulative_path_time)
    { // handle the Insert to be inserted before this path_idx (and all inserts not handled yet)
        inserts_.front().write(gcode);
        inserts_.pop_front();
    }
}

void ExtruderPlan::handleAllRemainingInserts(GCodeExport& gcode)
{
    while (! inserts_.empty())
    { // handle the Insert to be inserted before this path_idx (and all inserts not handled yet)
        NozzleTempInsert& insert = inserts_.front();
        insert.write(gcode);
        inserts_.pop_front();
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
    for (auto& path : paths_)
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

std::shared_ptr<const SliceMeshStorage> ExtruderPlan::findFirstPrintedMesh() const
{
    for (const GCodePath& path : paths_)
    {
        if (path.mesh)
        {
            return path.mesh;
        }
    }

    return nullptr;
}

} // namespace cura
