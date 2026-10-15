// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ExtruderTrain.h"

namespace cura
{

ExtruderTrain::ExtruderTrain(const size_t extruder_nr, Settings* parent_settings)
    : extruder_nr_(extruder_nr)
{
    settings_.setParent(parent_settings);
}

void ExtruderTrain::initializeMotionSettings()
{
    motion_settings_ = MotionSettings{ { settings_.get<Velocity>("speed_travel"),
                                         settings_.get<Acceleration>(settings_.get<bool>("acceleration_travel_enabled") ? "acceleration_travel" : "acceleration_print"),
                                         settings_.get<Velocity>(settings_.get<bool>("jerk_travel_enabled") ? "jerk_travel" : "jerk_print") },
                                       settings_.get<Velocity>("speed_z_hop"),
                                       settings_.get<double>("material_max_flowrate") };
}

const ExtruderTrain::MotionSettings& ExtruderTrain::getMotionSettings() const
{
    return motion_settings_.value();
}

} // namespace cura
