/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#include "utils/intpoint.h" // INT2MM
#include "GCodePathConfig.h"

namespace cura 
{

GCodePathConfig::BasicConfig::BasicConfig()
: speed(0)
, line_width(0)
, flow(100)
{
}


GCodePathConfig::BasicConfig::BasicConfig(double speed, int line_width, double flow)
: speed(speed)
, line_width(line_width)
, flow(flow)
{
}

void GCodePathConfig::BasicConfig::set(double speed, int line_width, double flow)
{
    this->speed = speed;
    this->line_width = line_width;
    this->flow = flow;
}


GCodePathConfig::GCodePathConfig(RetractionConfig* retraction_config, PrintFeatureType type)
: extrusion_mm3_per_mm(0.0)
, type(type)
, spiralize(false)
, retraction_config(retraction_config)
{
}

void GCodePathConfig::init(double speed, int line_width, double flow)
{
    iconic_config.set(speed, line_width, flow);
    current_config = iconic_config;
}

void GCodePathConfig::setLayerHeight(int layer_height)
{
    this->layer_thickness = layer_height;
    calculateExtrusion();
}

void GCodePathConfig::smoothSpeed(GCodePathConfig::BasicConfig first_layer_config, int layer_nr, double max_speed_layer) 
{
    current_config.speed = (iconic_config.speed*layer_nr)/max_speed_layer + (first_layer_config.speed*(max_speed_layer-layer_nr)/max_speed_layer);
}

void GCodePathConfig::setSpeedIconic()
{
    current_config.speed = iconic_config.speed;
}

double GCodePathConfig::getExtrusionMM3perMM()
{
    return extrusion_mm3_per_mm;
}

double GCodePathConfig::getSpeed()
{
    return current_config.speed;
}

int GCodePathConfig::getLineWidth()
{
    return current_config.line_width;
}

bool GCodePathConfig::isTravelPath()
{
    return current_config.line_width == 0;
}

double GCodePathConfig::getFlowPercentage()
{
    return current_config.flow;
}

void GCodePathConfig::calculateExtrusion()
{
    extrusion_mm3_per_mm = INT2MM(current_config.line_width) * INT2MM(layer_thickness) * double(current_config.flow) / 100.0;
}


}//namespace cura
