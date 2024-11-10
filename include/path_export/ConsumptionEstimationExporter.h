// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHEXPORTER_CONSUMPTIONESTIMATIONEXPORTER_H
#define PATHEXPORTER_CONSUMPTIONESTIMATIONEXPORTER_H

#include <map>
#include <optional>

#include "path_export/PathExporter.h"
#include "settings/types/Duration.h"

namespace cura
{

class ConsumptionEstimationExporter : public PathExporter
{
public:
    const std::map<PrintFeatureType, Duration>& getDurations() const;

    const std::map<size_t, double>& getExtrusionAmounts() const;

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

    void writeLayerEnd(const LayerIndex& layer_index, const coord_t z, const coord_t layer_thickness) override;

    void writeLayerStart(const LayerIndex& layer_index, const Point3LL& start_position) override;

private:
    std::optional<double> getDistanceToLastPosition(const Point3LL& p) const;

    void addDuration(const std::optional<double>& distance, const Velocity& speed, PrintFeatureType feature);

private:
    std::map<PrintFeatureType, Duration> durations_;
    std::map<size_t, double> extrusions_amounts_;
    std::optional<Point3LL> last_position_;
};

} // namespace cura

#endif // PATHEXPORTER_CONSUMPTIONESTIMATIONEXPORTER_H
