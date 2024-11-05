// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHEXPORT_CONSOLEEXPORTER_H
#define PATHEXPORT_CONSOLEEXPORTER_H

#include "path_export/PathExporter.h"

namespace cura
{

class ConsoleExporter : public PathExporter
{
public:
    virtual void writeExtrusion(
        const Point3LL& p,
        const Velocity& speed,
        const double extrusion_mm3_per_mm,
        const coord_t line_width,
        const coord_t line_thickness,
        const PrintFeatureType feature,
        const bool update_extrusion_offset) override;

    virtual void writeLayerStart(const LayerIndex& layer_index) override;

    virtual void writeLayerEnd(const LayerIndex& layer_index, const coord_t z, const coord_t layer_thickness) override;
};

} // namespace cura

#endif // PATHEXPORT_CONSOLEEXPORTER_H
