//Copyright (C) 2016 Ultimaker
//Released under terms of the AGPLv3 License

#include "GCodePath.h"

namespace cura
{

GCodePath::GCodePath(const GCodePathConfig& config, SpaceFillType space_fill_type, float flow, bool spiralize, double speed_factor) :
config(&config),
space_fill_type(space_fill_type),
flow(flow),
speed_factor(speed_factor),
spiralize(spiralize),
fan_speed(GCodePathConfig::FAN_SPEED_DEFAULT)
{
    retract = false;
    perform_z_hop = false;
    perform_prime = false;
    points = std::vector<Point>();
    done = false;
    estimates = TimeMaterialEstimates();
}

bool GCodePath::isTravelPath()
{
    return config->isTravelPath();
}

double GCodePath::getExtrusionMM3perMM()
{
    return flow * config->getExtrusionMM3perMM();
}

int GCodePath::getLineWidthForLayerView()
{
    return flow * config->getLineWidth() * config->getFlowPercentage() / 100.0;
}

void GCodePath::setFanSpeed(double fan_speed)
{
    this->fan_speed = fan_speed;
}

float GCodePath::getFanSpeed() const
{
    return (fan_speed >= 0 && fan_speed <= 100) ? fan_speed : config->getFanSpeed();
}

}
