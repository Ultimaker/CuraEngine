// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "pathPlanning/GCodePath.h"

#include "GCodePathConfig.h"

namespace cura
{
GCodePath::GCodePath(
    const GCodePathConfig& config,
    const SliceMeshStorage* mesh,
    const SpaceFillType space_fill_type,
    const Ratio flow,
    const Ratio width_factor,
    const bool spiralize,
    const Ratio speed_factor)
    : config(&config)
    , mesh(mesh)
    , space_fill_type(space_fill_type)
    , flow(flow)
    , width_factor(width_factor)
    , speed_factor(speed_factor)
    , speed_back_pressure_factor(1.0)
    , retract(false)
    , unretract_before_last_travel_move(false)
    , perform_z_hop(false)
    , perform_prime(false)
    , skip_agressive_merge_hint(false)
    , points(std::vector<Point>())
    , done(false)
    , spiralize(spiralize)
    , fan_speed(GCodePathConfig::FAN_SPEED_DEFAULT)
    , estimates(TimeMaterialEstimates())
{
}

bool GCodePath::isTravelPath() const
{
    return config->isTravelPath();
}

double GCodePath::getExtrusionMM3perMM() const
{
    return flow * width_factor * config->getExtrusionMM3perMM();
}

coord_t GCodePath::getLineWidthForLayerView() const
{
    return flow * width_factor * config->getLineWidth() * config->getFlowRatio();
}

void GCodePath::setFanSpeed(double fan_speed)
{
    this->fan_speed = fan_speed;
}

double GCodePath::getFanSpeed() const
{
    return (fan_speed >= 0 && fan_speed <= 100) ? fan_speed : config->getFanSpeed();
}

} // namespace cura
