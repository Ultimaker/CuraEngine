// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHEXPORT_COMMUNICATIONEXPORTER_H
#define PATHEXPORT_COMMUNICATIONEXPORTER_H

#include <memory>
#include <vector>

#include "plan_export/PlanExporter.h"

namespace cura
{
class Communication;

class CommunicationExporter : public PlanExporter
{
public:
    explicit CommunicationExporter(const std::shared_ptr<Communication>& communication);

    void writeExtrusion(
        const Point3LL& p,
        const Velocity& speed,
        const size_t extruder_nr,
        const double extrusion_mm3_per_mm,
        const coord_t line_width,
        const coord_t line_thickness,
        const PrintFeatureType feature,
        const bool update_extrusion_offset) override;

    void writeTravelMove(const Point3LL& position, const Velocity& speed, const PrintFeatureType feature) override;

    void writeLayerStart(const LayerIndex& layer_index, const Point3LL& start_position) override;

    void writeLayerEnd(const LayerIndex& layer_index, const coord_t z, const coord_t layer_thickness) override;

    void writeExtruderChange(const ExtruderNumber next_extruder) override;

private:
    std::shared_ptr<Communication> communication_;
};

} // namespace cura

#endif // PATHEXPORT_COMMUNICATIONEXPORTER_H
