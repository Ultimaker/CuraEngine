//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "settings/types/LayerIndex.h"
#include "utils/IntPoint.h" // INT2MM
#include "GCodePathConfig.h"

namespace cura 
{

GCodePathConfig::GCodePathConfig(const GCodePathConfig& other)
: type(other.type)
, speed_derivatives(other.speed_derivatives)
, line_width(other.line_width)
, layer_thickness(other.layer_thickness)
, flow(other.flow)
, extrusion_mm3_per_mm(other.extrusion_mm3_per_mm)
, is_bridge_path(other.is_bridge_path)
, fan_speed(other.fan_speed)
{
}



GCodePathConfig::GCodePathConfig(const PrintFeatureType& type, const coord_t line_width, const coord_t layer_height, const Ratio& flow, const GCodePathConfig::SpeedDerivatives speed_derivatives, const bool is_bridge_path, const double fan_speed)
: type(type)
, speed_derivatives(speed_derivatives)
, line_width(line_width)
, layer_thickness(layer_height)
, flow(flow)
, extrusion_mm3_per_mm(calculateExtrusion())
, is_bridge_path(is_bridge_path)
, fan_speed(fan_speed)
{
}

void GCodePathConfig::smoothSpeed(GCodePathConfig::SpeedDerivatives first_layer_config, const LayerIndex& layer_nr, const LayerIndex& max_speed_layer_nr) 
{
    double max_speed_layer = max_speed_layer_nr;
    speed_derivatives.speed = (speed_derivatives.speed * layer_nr) / max_speed_layer + (first_layer_config.speed * (max_speed_layer - layer_nr) / max_speed_layer);
    speed_derivatives.acceleration = (speed_derivatives.acceleration * layer_nr) / max_speed_layer + (first_layer_config.acceleration * (max_speed_layer - layer_nr) / max_speed_layer);
    speed_derivatives.jerk = (speed_derivatives.jerk * layer_nr) / max_speed_layer + (first_layer_config.jerk * (max_speed_layer - layer_nr) / max_speed_layer);
}

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


}//namespace cura
