//Copyright (C) 2016 Ultimaker
//Released under terms of the AGPLv3 License

#include "GCodePath.h"

namespace cura
{

GCodePath::GCodePath()
    : speed_factor(1.0)
{
}

bool GCodePath::isTravelPath()
{
    return config->isTravelPath();
}

double GCodePath::getExtrusionMM3perMM()
{
    return flow * config->getExtrusionMM3perMM();
}

int GCodePath::getLineWidth()
{
    return flow * config->getLineWidth() * config->getFlowPercentage() / 100.0;
}

}