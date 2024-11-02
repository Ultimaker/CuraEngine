// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_FEATUREEXTRUSION_H
#define PATHPLANNING_FEATUREEXTRUSION_H

#include "GCodePathConfig.h"
#include "path_planning/ExtruderMoveSet.h"

namespace cura
{
class Point3LL;

class FeatureExtrusion : public ExtruderMoveSet
{
public:
    explicit FeatureExtrusion(const GCodePathConfig& config);

    void addExtrusionMove(const Point3LL& position);

    const Velocity& getSpeed() const;

    double getExtrusionMM3perMM() const;

    PrintFeatureType getPrintFeatureType() const;

private:
    GCodePathConfig config_; //!< The configuration settings of the path.
};

} // namespace cura

#endif // PATHPLANNING_FEATUREEXTRUSION_H
