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
    const size_t /*extruder_nr*/,
    const double /*extrusion_mm3_per_mm*/,
    const coord_t line_width,
    const coord_t line_thickness,
    const PrintFeatureType feature,
    const bool /*update_extrusion_offset*/)
{
    communication_->sendLineTo(feature, p, line_width, line_thickness, speed);
}

void CommunicationExporter::writeTravelMove(const Point3LL& position, const Velocity& speed, const PrintFeatureType feature)
{
    communication_->sendLineTo(feature, position, 0, 0, speed);
}

void CommunicationExporter::writeLayerStart(const LayerIndex& layer_index, const Point3LL& start_position)
{
    communication_->setLayerForSend(layer_index);
    communication_->sendCurrentPosition(start_position);
}

void CommunicationExporter::writeLayerEnd(const LayerIndex& layer_index, const coord_t z, const coord_t layer_thickness)
{
    communication_->sendLayerComplete(layer_index, z, layer_thickness);
}

} // namespace cura
