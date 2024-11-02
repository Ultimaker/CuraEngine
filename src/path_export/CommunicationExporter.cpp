// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_export/CommunicationExporter.h"

#include "communication/Communication.h"

namespace cura
{

CommunicationExporter::CommunicationExporter(const std::shared_ptr<Communication>& communication)
    : communication_(communication)
{
}

void CommunicationExporter::writeExtrusion(
    const Point3LL& p,
    const Velocity& speed,
    const double extrusion_mm3_per_mm,
    const coord_t line_width,
    const coord_t line_thickness,
    const PrintFeatureType feature,
    const bool update_extrusion_offset)
{
    communication_->sendLineTo(feature, p, line_width, line_thickness, speed);
}

} // namespace cura
