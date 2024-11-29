// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "plan_export/ConsumptionEstimationExporter.h"

#include <settings/types/Velocity.h>

namespace cura
{

const std::map<PrintFeatureType, Duration>& ConsumptionEstimationExporter::getDurations() const
{
    return durations_;
}

const std::map<size_t, double>& ConsumptionEstimationExporter::getExtrusionAmounts() const
{
    return extrusions_amounts_;
}

void ConsumptionEstimationExporter::writeExtrusion(
    const Point3LL& p,
    const Velocity& speed,
    const size_t extruder_nr,
    const double extrusion_mm3_per_mm,
    const coord_t /*line_width*/,
    const coord_t /*line_thickness*/,
    const PrintFeatureType feature,
    const bool /*update_extrusion_offset*/)
{
    std::optional<double> distance = getDistanceToLastPosition(p);
    addDuration(distance, speed, feature);

    if (distance.has_value()) [[likely]]
    {
        double extrusion_amount = distance.value() * extrusion_mm3_per_mm;

        if (auto iterator = extrusions_amounts_.find(extruder_nr); iterator != extrusions_amounts_.end()) [[likely]]
        {
            iterator->second += extrusion_amount;
        }
        else
        {
            extrusions_amounts_.insert({ extruder_nr, extrusion_amount });
        }
    }

    last_position_ = p;
}

void ConsumptionEstimationExporter::writeTravelMove(const Point3LL& position, const Velocity& speed, const PrintFeatureType feature)
{
    std::optional<double> distance = getDistanceToLastPosition(position);
    addDuration(distance, speed, feature);

    last_position_ = position;
}

void ConsumptionEstimationExporter::writeLayerEnd(const LayerIndex& /*layer_index*/, const coord_t /*z*/, const coord_t /*layer_thickness*/)
{
}

void ConsumptionEstimationExporter::writeLayerStart(const LayerIndex& /*layer_index*/, const Point3LL& /*start_position*/)
{
}
std::optional<double> ConsumptionEstimationExporter::getDistanceToLastPosition(const Point3LL& p) const
{
    if (last_position_.has_value()) [[likely]]
    {
        return (p - last_position_.value()).vSizeMM();
    }

    return std::nullopt;
}

void ConsumptionEstimationExporter::addDuration(const std::optional<double>& distance, const Velocity& speed, PrintFeatureType feature)
{
    if (distance.has_value()) [[likely]]
    {
        const Duration duration = distance.value() / speed;

        if (auto iterator = durations_.find(feature); iterator != durations_.end())
        {
            iterator->second += duration;
        }
        else
        {
            durations_.insert({ feature, duration });
        }
    }
}

} // namespace cura