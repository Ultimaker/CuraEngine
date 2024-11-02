// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_export/ConsoleExporter.h"

#include <spdlog/spdlog.h>

#include "geometry/Point3LL.h"

namespace cura
{

void ConsoleExporter::writeExtrusion(
    const Point3LL& p,
    const Velocity& speed,
    const double extrusion_mm3_per_mm,
    const PrintFeatureType feature,
    const bool update_extrusion_offset)
{
    spdlog::info("EXTRUSION {}", p);
}

} // namespace cura
