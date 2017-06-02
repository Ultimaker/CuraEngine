/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#include "utils/intpoint.h" // INT2MM
#include "utils/logoutput.h"
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
{
}



GCodePathConfig::GCodePathConfig(PrintFeatureType type, int line_width, int layer_height, double flow, GCodePathConfig::SpeedDerivatives speed_derivatives)
: type(type)
, speed_derivatives(speed_derivatives)
, line_width(line_width)
, layer_thickness(layer_height)
, flow(flow)
, extrusion_mm3_per_mm(calculateExtrusion())
{
}

void GCodePathConfig::smoothSpeed(GCodePathConfig::SpeedDerivatives first_layer_config, int layer_nr, int max_speed_layer_nr) 
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

double GCodePathConfig::getSpeed() const
{
    return speed_derivatives.speed;
}

double GCodePathConfig::getAcceleration() const
{
    return speed_derivatives.acceleration;
}

double GCodePathConfig::getJerk() const
{
    return speed_derivatives.jerk;
}

int GCodePathConfig::getLineWidth() const
{
    return line_width;
}

bool GCodePathConfig::isTravelPath() const
{
    return line_width == 0;
}

double GCodePathConfig::getFlowPercentage() const
{
    return flow;
}

double GCodePathConfig::calculateExtrusion() const
{
    return INT2MM(line_width) * INT2MM(layer_thickness) * double(flow) / 100.0;
}


}//namespace cura
