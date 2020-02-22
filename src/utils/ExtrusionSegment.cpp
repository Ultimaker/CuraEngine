/** Copyright (C) 2019 Ultimaker */
#include "ExtrusionSegment.h"

#include "utils/logoutput.h"

namespace arachne
{
    
Polygons ExtrusionSegment::toPolygons()
{
    return toPolygons(is_reduced);
}
Polygons ExtrusionSegment::toPolygons(bool reduced)
{
    Polygons ret;
    Point vec = to.p - from.p;
    coord_t vec_length = vSize( vec );

    if (vec_length <= 0)
    {
        return ret;
    }

    PolygonRef poly = ret.newPoly();
    float delta_r = 0.5f * std::abs(from.w - to.w);
    float vec_length_fixed = std::max(delta_r, static_cast<float>(vec_length));
    float alpha = std::acos(delta_r / vec_length_fixed);
    if (to.w > from.w)
    {
        alpha = M_PI - alpha;
    }
    assert(alpha > - M_PI - 0.0001);
    assert(alpha < M_PI + 0.0001);
    
    float dir = std::atan(vec.Y / static_cast<float>(vec.X));
    if (vec.X < 0)
    {
        dir += M_PI;
    }
    
    {
        poly.emplace_back(from.p + Point(from.w / 2 * cos(alpha + dir), from.w / 2 * sin(alpha + dir)));
        float start_a = 2 * M_PI; while (start_a > alpha + dir) start_a -= a_step;
        start_a += a_step;
        float end_a = -2 * M_PI; while (end_a < 2 * M_PI - alpha + dir) end_a += a_step;
        for (float a = start_a; a <= end_a; a += a_step)
        {
            poly.emplace_back(from.p + Point(from.w / 2 * cos(a), from.w / 2 * sin(a)));
        }
        poly.emplace_back(from.p + Point(from.w / 2 * cos(2 * M_PI - alpha + dir), from.w / 2 * sin(2 * M_PI - alpha + dir)));
    }
    {
        poly.emplace_back(to.p + Point(to.w / 2 * cos(2 * M_PI - alpha + dir), to.w / 2 * sin(2 * M_PI - alpha + dir)));
        float start_a = 2 * M_PI; while (start_a > alpha + dir) start_a -= a_step;
        if (reduced)
        {
            start_a += a_step;
        }
        float end_a = -2 * M_PI; while (end_a < 2 * M_PI - alpha + dir) end_a += a_step;
        if (reduced)
        {
            end_a -= a_step;
        }
        else
        {
            end_a -= 2 * M_PI;
        }
        if (reduced)
        {
            for (float a = end_a; a >= start_a; a -= a_step)
            {
                poly.emplace_back(to.p + Point(to.w / 2 * cos(a), to.w / 2 * sin(a)));
            }
        }
        else
        {
            for (float a = end_a; a <= start_a; a += a_step)
            {
                poly.emplace_back(to.p + Point(to.w / 2 * cos(a), to.w / 2 * sin(a)));
            }
        }
        poly.emplace_back(to.p + Point(to.w / 2 * cos(alpha + dir), to.w / 2 * sin(alpha + dir)));
    }
    
    for (Point p : poly)
    {
        assert(p.X < 0x3FFFFFFFFFFFFFFFLL);
        assert(p.Y < 0x3FFFFFFFFFFFFFFFLL);
    }

    return ret;
}

}//namespace cura
