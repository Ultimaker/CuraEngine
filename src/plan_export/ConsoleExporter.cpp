// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "plan_export/ConsoleExporter.h"

#include <spdlog/spdlog.h>

#include "geometry/Point3LL.h"
#include "settings/types/LayerIndex.h"

namespace cura
{

void ConsoleExporter::writeExtrusion(
    const Point3LL& p,
    const Velocity& speed,
    const size_t extruder_nr,
    const double extrusion_mm3_per_mm,
    const coord_t line_width,
    const coord_t line_thickness,
    const PrintFeatureType feature,
    const bool update_extrusion_offset)
{
    spdlog::info("EXTRUSION {}", p);
}

void ConsoleExporter::writeTravelMove(const Point3LL& position, const Velocity& /*speed*/, const PrintFeatureType /*feature*/)
{
    spdlog::info("TRAVEL {}", position);
}

void ConsoleExporter::writeLayerStart(const LayerIndex& layer_index, const Point3LL& /*start_position*/)
{
    spdlog::info("######## START NEW LAYER {}", layer_index.value);
}

void ConsoleExporter::writeLayerEnd(const LayerIndex& layer_index, const coord_t /*z*/, const coord_t /*layer_thickness*/)
{
    spdlog::info("######## LAYER END {}", layer_index.value);
}

} // namespace cura
