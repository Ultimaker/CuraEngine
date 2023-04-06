//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TREESUPPORTCIRCLE_H
#define TREESUPPORTCIRCLE_H

#include "utils/Coord_t.h"
#include "utils/polygon.h"

#include <cmath>
#include <range/v3/view/iota.hpp>

namespace cura
{

class TreeSupportBaseCircle
{
protected:
    inline static Polygon base_circle;
    inline static bool circle_generated = false;

public:
    inline static constexpr int64_t base_radius = 50;

    static Polygon getBaseCircle()
    {
        if (circle_generated)
        {
            return base_circle;
        }
        else
        {
            std::mutex critical_sections;
            std::lock_guard<std::mutex> critical_section_progress(critical_sections);
            if (circle_generated)
            {
                return base_circle;
            }

            constexpr auto support_tree_circle_resolution = 12; // The number of vertices in each circle.
            Polygon circle;
            for (const uint64_t i : ranges::views::iota(0, support_tree_circle_resolution))
            {
                const AngleRadians angle = static_cast<double>(i) / support_tree_circle_resolution * TAU;
                circle.emplace_back(static_cast<coord_t>(std::cos(angle) * base_radius), static_cast<coord_t>(std::sin(angle) * base_radius));
            }
            base_circle = Polygon(circle);
            circle_generated = true;
            return base_circle;
        }
    }
};

} // namespace cura

#endif // TREESUPPORTCIRCLE_H
