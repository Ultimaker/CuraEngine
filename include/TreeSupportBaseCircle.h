// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <cmath>
#include <mutex>

#include <range/v3/view/iota.hpp>

#include "types/AngleRadians.h"
#include "types/Coord_t.h"
#include "types/geometry/Polygon.h"

namespace curaengine
{

class TreeSupportBaseCircle
{
protected:
    inline static types::geometry::Polygon base_circle;
    inline static std::once_flag flag;

public:
    inline static constexpr int64_t base_radius = 50;

    static types::geometry::Polygon getBaseCircle()
    {
        std::call_once(
            flag,
            [&]()
            {
                constexpr auto support_tree_circle_resolution = 12; // The number of vertices in each circle.
                types::geometry::Polygon circle;
                for (const uint64_t i : ranges::views::iota(0, support_tree_circle_resolution))
                {
                    const types::AngleRadians angle = static_cast<double>(i) / support_tree_circle_resolution * utils::tau;
                    circle.emplace_back(static_cast<types::coord_t>(std::cos(angle) * base_radius), static_cast<types::coord_t>(std::sin(angle) * base_radius));
                }
                base_circle = types::geometry::Polygon(circle);
            });
        return base_circle;
    }
};

} // namespace curaengine
