//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TREESUPPORTCIRCLE_H
#define TREESUPPORTCIRCLE_H

#include "utils/Coord_t.h"
#include "utils/polygon.h"
#define SUPPORT_TREE_CIRCLE_RESOLUTION 25 // The number of vertices in each circle.
namespace cura
{

class TreeSupportBaseCircle
{
    inline static Polygon base_circle;
    inline static bool circle_generated = false;

  public:
    static const int32_t base_radius = 50;
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
            Polygon circle;
            for (unsigned int i = 0; i < SUPPORT_TREE_CIRCLE_RESOLUTION; i++)
            {
                const AngleRadians angle = static_cast<double>(i) / SUPPORT_TREE_CIRCLE_RESOLUTION * TAU;
                circle.emplace_back(coord_t(cos(angle) * base_radius), coord_t(sin(angle) * base_radius));
            }
            base_circle = Polygon(circle);
            circle_generated = true;
            return base_circle;
        }
    }
};

} // namespace cura

#endif // TREESUPPORTCIRCLE_H
