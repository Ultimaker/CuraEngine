/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#include "utils/intpoint.h"
#include "GCodePathConfig.h"

namespace cura 
{


GCodePathConfig::GCodePathConfig(RetractionConfig* retraction_config, PrintFeatureType type)
: speed_iconic(0)
, speed(0)
, line_width(0)
, extrusion_mm3_per_mm(0.0)
, type(type)
, spiralize(false)
, retraction_config(retraction_config)
{
}

void GCodePathConfig::init(double speed, int line_width, double flow)
{
    speed_iconic = speed;
    this->speed = speed;
    this->line_width = line_width;
    this->flow = flow;
}

void GCodePathConfig::setLayerHeight(int layer_height)
{
    this->layer_thickness = layer_height;
    calculateExtrusion();
}

void GCodePathConfig::smoothSpeed(double min_speed, int layer_nr, double max_speed_layer) 
{
    speed = (speed_iconic*layer_nr)/max_speed_layer + (min_speed*(max_speed_layer-layer_nr)/max_speed_layer);
}

void GCodePathConfig::setSpeedIconic()
{
    speed = speed_iconic;
}

double GCodePathConfig::getExtrusionMM3perMM()
{
    return extrusion_mm3_per_mm;
}

double GCodePathConfig::getSpeed()
{
    return speed;
}

int GCodePathConfig::getLineWidth()
{
    return line_width;
}

bool GCodePathConfig::isTravelPath()
{
    return line_width == 0;
}

double GCodePathConfig::getFlowPercentage()
{
    return flow;
}

void GCodePathConfig::calculateExtrusion()
{
    extrusion_mm3_per_mm = INT2MM(line_width) * INT2MM(layer_thickness) * double(flow) / 100.0;
}


}//namespace cura
