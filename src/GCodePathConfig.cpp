// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "GCodePathConfig.h"

#include "settings/types/LayerIndex.h"
#include "utils/IntPoint.h" // INT2MM

namespace cura
{

double GCodePathConfig::getExtrusionMM3perMM() const
{
    return extrusion_mm3_per_mm;
}

Velocity GCodePathConfig::getSpeed() const
{
    return speed_derivatives.speed;
}

Acceleration GCodePathConfig::getAcceleration() const
{
    return speed_derivatives.acceleration;
}

Velocity GCodePathConfig::getJerk() const
{
    return speed_derivatives.jerk;
}

coord_t GCodePathConfig::getLineWidth() const
{
    return line_width;
}

coord_t GCodePathConfig::getLayerThickness() const
{
    return layer_thickness;
}

const PrintFeatureType& GCodePathConfig::getPrintFeatureType() const
{
    return type;
}

bool GCodePathConfig::isTravelPath() const
{
    return line_width == 0;
}

bool GCodePathConfig::isBridgePath() const
{
    return is_bridge_path;
}

double GCodePathConfig::getFanSpeed() const
{
    return fan_speed;
}

Ratio GCodePathConfig::getFlowRatio() const
{
    return flow;
}

double GCodePathConfig::calculateExtrusion() const
{
    return INT2MM(line_width) * INT2MM(layer_thickness) * double(flow);
}


} // namespace cura
