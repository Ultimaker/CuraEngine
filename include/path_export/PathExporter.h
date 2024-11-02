// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHEXPORT_PATHEXPORTER_H
#define PATHEXPORT_PATHEXPORTER_H

#include "PrintFeatureType.h"

namespace cura
{
class Point3LL;
class Velocity;

class PathExporter
{
public:
    virtual void writeExtrusion(
        const Point3LL& p,
        const Velocity& speed,
        const double extrusion_mm3_per_mm,
        const PrintFeatureType feature,
        const bool update_extrusion_offset)
        = 0;
};

} // namespace cura

#endif // PATHEXPORT_PATHEXPORTER_H
