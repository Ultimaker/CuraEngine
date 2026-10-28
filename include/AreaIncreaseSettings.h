// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "AvoidanceType.h"
#include "utils/Coord_t.h"

namespace curaengine
{
struct AreaIncreaseSettings
{
    AreaIncreaseSettings() = default;

    AreaIncreaseSettings(const AvoidanceType type, const coord_t increase_speed, const bool increase_radius, const bool simplify, const bool use_min_distance, const bool move)
        : type_(type)
        , increase_speed_(increase_speed)
        , increase_radius_(increase_radius)
        , no_error_(simplify)
        , use_min_distance_(use_min_distance)
        , move_(move)
    {
    }

    AvoidanceType type_{ AvoidanceType::FAST };
    coord_t increase_speed_{ 0 };
    bool increase_radius_{ false };
    bool no_error_{ false };
    bool use_min_distance_{ false };
    bool move_{ false };

    bool operator==(const AreaIncreaseSettings& other) const = default;
};

} // namespace curaengine
