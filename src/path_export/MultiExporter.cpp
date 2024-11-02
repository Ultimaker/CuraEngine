// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_export/MultiExporter.h"

namespace cura
{

void MultiExporter::writeExtrusion(
    const Point3LL& p,
    const Velocity& speed,
    const double extrusion_mm3_per_mm,
    const coord_t line_width,
    const coord_t line_thickness,
    const PrintFeatureType feature,
    const bool update_extrusion_offset)
{
    for (const std::shared_ptr<PathExporter>& exporter : exporters_)
    {
        exporter->writeExtrusion(p, speed, extrusion_mm3_per_mm, line_width, line_thickness, feature, update_extrusion_offset);
    }
}

void MultiExporter::appendExporter(const std::shared_ptr<PathExporter> exporter)
{
    exporters_.push_back(exporter);
}

} // namespace cura
