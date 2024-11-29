// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_FEATUREEXTRUSION_H
#define PATHPLANNING_FEATUREEXTRUSION_H

#include "GCodePathConfig.h"
#include "geometry/Point3LL.h"
#include "print_operation/ContinuousExtruderMoveSequencePtr.h"
#include "print_operation/PrintOperationSequence.h"

namespace cura
{

class FeatureExtrusion : public PrintOperationSequence
{
public:
    explicit FeatureExtrusion(const GCodePathConfig& config);

    void appendExtruderMoveSequence(const ContinuousExtruderMoveSequencePtr& extruder_move_sequence, bool check_non_empty = true);

    const Velocity& getSpeed() const;

    double getExtrusionMM3perMM() const;

    PrintFeatureType getPrintFeatureType() const;

    coord_t getLineWidth() const;

    coord_t getLayerThickness() const;

private:
    const Ratio& getFlow() const;

    const Ratio& getWidthFactor() const;

private:
    GCodePathConfig config_; //!< The configuration settings of the path.
    Ratio flow_{ 1.0 }; //!< A type-independent flow configuration
    Ratio width_factor_{ 1.0 }; //!< Adjustment to the line width. Similar to flow, but causes the speed_back_pressure_factor to be adjusted.
};

} // namespace cura

#endif // PATHPLANNING_FEATUREEXTRUSION_H
