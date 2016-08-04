/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#include "utils/intpoint.h" // INT2MM
#include "GCodePathConfig.h"

namespace cura 
{

GCodePathConfig::BasicConfig::BasicConfig()
: speed(0)
, acceleration(0)
, jerk(0)
, line_width(0)
, flow(100)
{
}


GCodePathConfig::BasicConfig::BasicConfig(double speed, double acceleration, double jerk, int line_width, double flow)
: speed(speed)
, acceleration(acceleration)
, jerk(jerk)
, line_width(line_width)
, flow(flow)
{
}

void GCodePathConfig::BasicConfig::set(double speed, double acceleration, double jerk, int line_width, double flow)
{
    this->speed = speed;
    this->acceleration = acceleration;
    this->jerk = jerk;
    this->line_width = line_width;
    this->flow = flow;
}


GCodePathConfig::GCodePathConfig(PrintFeatureType type)
: extrusion_mm3_per_mm(0.0)
, type(type)
{
}

void GCodePathConfig::init(double speed, double acceleration, double jerk, int line_width, double flow)
{
    iconic_config.set(speed, acceleration, jerk, line_width, flow);
    current_config = iconic_config;
}

void GCodePathConfig::setLayerHeight(int layer_height)
{
    this->layer_thickness = layer_height;
    calculateExtrusion();
}

void GCodePathConfig::smoothSpeed(GCodePathConfig::BasicConfig first_layer_config, int layer_nr, double max_speed_layer) 
{
    current_config.speed = (iconic_config.speed * layer_nr) / max_speed_layer + (first_layer_config.speed * (max_speed_layer - layer_nr) / max_speed_layer);
    current_config.acceleration = (iconic_config.acceleration * layer_nr) / max_speed_layer + (first_layer_config.acceleration * (max_speed_layer - layer_nr) / max_speed_layer);
    current_config.jerk = (iconic_config.jerk * layer_nr) / max_speed_layer + (first_layer_config.jerk * (max_speed_layer - layer_nr) / max_speed_layer);
}

void GCodePathConfig::setSpeedIconic()
{
    current_config.speed = iconic_config.speed;
    current_config.acceleration = iconic_config.acceleration;
    current_config.jerk = iconic_config.jerk;
}

double GCodePathConfig::getExtrusionMM3perMM() const
{
    return extrusion_mm3_per_mm;
}

double GCodePathConfig::getSpeed() const
{
    return current_config.speed;
}

double GCodePathConfig::getAcceleration() const
{
    return current_config.acceleration;
}

double GCodePathConfig::getJerk() const
{
    return current_config.jerk;
}

int GCodePathConfig::getLineWidth() const
{
    return current_config.line_width;
}

bool GCodePathConfig::isTravelPath() const
{
    return current_config.line_width == 0;
}

double GCodePathConfig::getFlowPercentage() const
{
    return current_config.flow;
}

void GCodePathConfig::calculateExtrusion()
{
    extrusion_mm3_per_mm = INT2MM(current_config.line_width) * INT2MM(layer_thickness) * double(current_config.flow) / 100.0;
}


}//namespace cura
