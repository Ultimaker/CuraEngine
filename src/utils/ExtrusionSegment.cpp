/** Copyright (C) 2019 Ultimaker */
#include "ExtrusionSegment.h"

namespace arachne
{
    
Polygons ExtrusionSegment::toPolygons()
{
    Polygons ret;
    Point vec = to.p - from.p;
    coord_t vec_length = vSize( vec );

    if (vec_length <= 0)
    {
        return ret;
    }

    PolygonRef poly = ret.newPoly();
    float alpha = std::acos(std::abs(from.w - to.w) / 2.0f / static_cast<float>( vec_length ));
    if (to.w > from.w)
    {
        alpha = M_PI - alpha;
    }
    assert(alpha > - M_PI);
    assert(alpha < M_PI);
    
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
//         end_a -= a_step;
        for (float a = start_a; a <= end_a; a += a_step)
        {
            poly.emplace_back(from.p + Point(from.w / 2 * cos(a), from.w / 2 * sin(a)));
        }
        poly.emplace_back(from.p + Point(from.w / 2 * cos(2 * M_PI - alpha + dir), from.w / 2 * sin(2 * M_PI - alpha + dir)));
    }
    {
        poly.emplace_back(to.p + Point(to.w / 2 * cos(2 * M_PI - alpha + dir), to.w / 2 * sin(2 * M_PI - alpha + dir)));
        float start_a = 2 * M_PI; while (start_a > alpha + dir) start_a -= a_step;
//         start_a += a_step;
        float end_a = -2 * M_PI; while (end_a < 2 * M_PI - alpha + dir) end_a += a_step;
//         end_a -= a_step;
        end_a -= 2 * M_PI;
        for (float a = end_a; a <= start_a; a += a_step)
        {
            poly.emplace_back(to.p + Point(to.w / 2 * cos(a), to.w / 2 * sin(a)));
        }
        poly.emplace_back(to.p + Point(to.w / 2 * cos(alpha + dir), to.w / 2 * sin(alpha + dir)));
        assert(shorterThen(poly.back(), 100000));
    }
    

    return ret;
}
    
Polygons ExtrusionSegment::toReducedPolygons()
{
    Polygons ret;
    Point vec = to.p - from.p;
    coord_t vec_length = vSize( vec );

    if (vec_length <= 0)
    {
        return ret;
    }

    PolygonRef poly = ret.newPoly();
    float alpha = std::acos(std::abs(from.w - to.w) / 2.0f / static_cast<float>( vec_length ));
    if (to.w > from.w)
    {
        alpha = M_PI - alpha;
    }
    assert(alpha > - M_PI);
    assert(alpha < M_PI);
    
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
        end_a -= a_step;
        for (float a = start_a; a <= end_a; a += a_step)
        {
            poly.emplace_back(from.p + Point(from.w / 2 * cos(a), from.w / 2 * sin(a)));
        }
        poly.emplace_back(from.p + Point(from.w / 2 * cos(2 * M_PI - alpha + dir), from.w / 2 * sin(2 * M_PI - alpha + dir)));
    }
    {
        poly.emplace_back(to.p + Point(to.w / 2 * cos(2 * M_PI - alpha + dir), to.w / 2 * sin(2 * M_PI - alpha + dir)));
        float start_a = 2 * M_PI; while (start_a > alpha + dir) start_a -= a_step;
        start_a += a_step;
        float end_a = -2 * M_PI; while (end_a < 2 * M_PI - alpha + dir) end_a += a_step;
        end_a -= a_step;
        for (float a = end_a; a >= start_a; a -= a_step)
        {
            poly.emplace_back(to.p + Point(to.w / 2 * cos(a), to.w / 2 * sin(a)));
        }
        poly.emplace_back(to.p + Point(to.w / 2 * cos(alpha + dir), to.w / 2 * sin(alpha + dir)));
    }
    

    return ret;
}

}//namespace cura
