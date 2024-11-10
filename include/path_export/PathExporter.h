// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHEXPORT_PATHEXPORTER_H
#define PATHEXPORT_PATHEXPORTER_H

#include <geometry/Point3LL.h>

#include "PrintFeatureType.h"
#include "utils/Coord_t.h"

namespace cura
{
class Point3LL;
class Velocity;
class LayerIndex;

class PathExporter
{
public:
    virtual ~PathExporter() = default; // Force class being polymorphic

    virtual void writeExtrusion(
        const Point3LL& p,
        const Velocity& speed,
        const size_t extruder_nr,
        const double extrusion_mm3_per_mm,
        const coord_t line_width,
        const coord_t line_thickness,
        const PrintFeatureType feature,
        const bool update_extrusion_offset)
        = 0;

    virtual void writeTravelMove(const Point3LL& position, const Velocity& speed, const PrintFeatureType feature) = 0;

    virtual void writeLayerEnd(const LayerIndex& layer_index, const coord_t z, const coord_t layer_thickness) = 0;

    virtual void writeLayerStart(const LayerIndex& layer_index, const Point3LL& start_position) = 0;
};

} // namespace cura

#endif // PATHEXPORT_PATHEXPORTER_H
