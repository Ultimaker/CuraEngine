// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_FEATUREEXTRUSION_H
#define PATHPLANNING_FEATUREEXTRUSION_H

#include "GCodePathConfig.h"
#include "geometry/Point3LL.h"
#include "path_planning/PrintOperationSequence.h"

namespace cura
{

class FeatureExtrusion : public PrintOperationSequence
{
public:
    explicit FeatureExtrusion(const GCodePathConfig& config, const Point3LL& start_position);

    void appendExtruderMoveSequence(const std::shared_ptr<ExtruderMoveSequence>& extruder_move_sequence);

    const Point3LL& getStartPosition();

    const Velocity& getSpeed() const;

    double getExtrusionMM3perMM() const;

    PrintFeatureType getPrintFeatureType() const;

    coord_t getLineWidth() const;

    coord_t getLayerThickness() const;

private:
    const Ratio& getFlow() const;

    const Ratio& getWidthFactor() const;

private:
    Point3LL start_position_;
    GCodePathConfig config_; //!< The configuration settings of the path.
    Ratio flow_{ 1.0 }; //!< A type-independent flow configuration
    Ratio width_factor_{ 1.0 }; //!< Adjustment to the line width. Similar to flow, but causes the speed_back_pressure_factor to be adjusted.
};

} // namespace cura

#endif // PATHPLANNING_FEATUREEXTRUSION_H
