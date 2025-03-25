// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "plan_export/MultiExporter.h"

namespace cura
{

void MultiExporter::writeExtrusion(
    const Point3LL& p,
    const Velocity& speed,
    const size_t extruder_nr,
    const double extrusion_mm3_per_mm,
    const coord_t line_width,
    const coord_t line_thickness,
    const PrintFeatureType feature,
    const bool update_extrusion_offset)
{
    for (const std::shared_ptr<PlanExporter>& exporter : exporters_)
    {
        exporter->writeExtrusion(p, speed, extruder_nr, extrusion_mm3_per_mm, line_width, line_thickness, feature, update_extrusion_offset);
    }
}

void MultiExporter::writeTravelMove(const Point3LL& position, const Velocity& speed, const PrintFeatureType feature)
{
    for (const std::shared_ptr<PlanExporter>& exporter : exporters_)
    {
        exporter->writeTravelMove(position, speed, feature);
    }
}

void MultiExporter::writeLayerStart(const LayerIndex& layer_index, const Point3LL& start_position)
{
    for (const std::shared_ptr<PlanExporter>& exporter : exporters_)
    {
        exporter->writeLayerStart(layer_index, start_position);
    }
}

void MultiExporter::writeLayerEnd(const LayerIndex& layer_index, const coord_t z, const coord_t layer_thickness)
{
    for (const std::shared_ptr<PlanExporter>& exporter : exporters_)
    {
        exporter->writeLayerEnd(layer_index, z, layer_thickness);
    }
}

void MultiExporter::writeExtruderChange(const ExtruderNumber next_extruder)
{
    for (const std::shared_ptr<PlanExporter>& exporter : exporters_)
    {
        exporter->writeExtruderChange(next_extruder);
    }
}

void MultiExporter::appendExporter(const std::shared_ptr<PlanExporter> exporter)
{
    exporters_.push_back(exporter);
}

} // namespace cura
