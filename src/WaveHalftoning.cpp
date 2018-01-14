/** Copyright (C) 2017 Ultimaker- Released under terms of the AGPLv3 License */
#include "WaveHalftoning.h"

#include "utils/SVG.h" // TODO remove debug stuff

// The length of a normalized vector. Cannot be 1 because points use integer logic.
#define NORMAL_LENGTH 10000

namespace cura 
{

WaveHalftoning::WaveHalftoning(const SliceMeshStorage& mesh)
: settings(&mesh)
{
    if (mesh.getSettingBoolean("wave_halftoning_map_enabled"))
    {
        assert(mesh.texture_proximity_processor && "texture_proximity_processor should have been initialized");
        getAmplitude = [&mesh, this](const unsigned int layer_nr, const Point p)
        {
            assert(mesh.texture_proximity_processor && "When wave_halftoning_map_enabled there has to be a texture proximity processor!");
            TextureProximityProcessor& texture_proximity_processor = *mesh.texture_proximity_processor;
            float color = texture_proximity_processor.getColor(p, layer_nr, settings.color_usage, 0.0); // TODO change default 0.0
            if (settings.inverse_color_usage)
            {
                color = 1.0 - color;
            }
            coord_t ret = color * settings.max_amplitude;
            return ret;
        };
    }
    else
    {
        getAmplitude = [this](const unsigned int, const Point)
        {
            return settings.max_amplitude;
        };
    }
}

Polygons WaveHalftoning::makeHalftoned(const unsigned int layer_nr, const Polygons& in)
{
    Polygons results;
    if (in.size() == 0)
    {
        return results;
    }

    for (ConstPolygonRef poly : const_cast<Polygons&>(in))
    {
        assert(poly.size() >= 3);
        // generate points in between p0 and p1
        PolygonRef result = results.newPoly();

        Point p0 = poly[poly.size() - 2];
        Point p1 = poly.back();
        for (int p0_idx = poly.size() - 2; p0_idx >= 0 && p0 == p1; p0_idx--)
        { // p0 is the last point before p1 which is different from p1
            p0 = poly[p0_idx];
        }
        CarryOver carry_over;
        carry_over.dist_left_over = 0;
        carry_over.last_amplitude = 0.0; // unused in the first iteration since dist_between_points = carry_over.dist_left_over; see makeCornerHalftoned
        carry_over.p0p1_perp = turn90CCW(p1 - p0);
        carry_over.direction = 1;
        if ((layer_nr / 2) % 2 == 0)
        {
            carry_over.direction *= -1;
        }
        for (Point p2 : poly)
        {
            if (p2 == p1)
            {
                continue;
            }
            makeCornerHalftoned(layer_nr, p0, p1, p2, carry_over, result);
            makeSegmentHalftoned(layer_nr, p1, p2, result, carry_over);
            p0 = p1;
            p1 = p2;
        }
        while (result.size() < 3)
        {
            unsigned int point_idx = poly.size() - 2;
            result.add(poly[point_idx]);
            if (point_idx == 0)
            {
                break;
            }
            point_idx--;
        }
        if (result.size() < 3)
        {
            result.clear();
            for (const Point& p : poly)
            {
                result.add(p);
            }
        }
    }
    return results;
}

void WaveHalftoning::makeCornerHalftoned(const unsigned int layer_nr, const Point /*p0*/, const Point p1, const Point p2, const CarryOver carry_over, PolygonRef result)
{
    const Point p0p1_perp = carry_over.p0p1_perp;
    const Point p1p2 = p2 - p1;
    const Point p1p2_perp = turn90CCW(p1p2);
    const Point corner_normal = normal(p0p1_perp, NORMAL_LENGTH) + normal(p1p2_perp, NORMAL_LENGTH);

    // x is the last point which was offsetted
    // a is the next point to be offsetted
    //
    //         dist_between_points
    //        ^^^^^^^^^^^^^^^^^^^^
    //                   p1pa_dist
    //        pxp1_dist  ^^^^^^^^^
    //        ^^^^^^^^^^
    //                  ┬                    > amplitudes
    //                  |
    //                  |
    //        ┬         |
    //        ┥         |                    > previous random offset within amplitude
    //        |         ┥pr       ┬          > corner offset computed by weighted average based on pxp0_dist, p0pa_dist and the amplitudes
    //        |         |         |
    // -------x---------p1--------a-------
    //        |         |         ┥          > next random offset within amplitude
    //        |         |         ┴
    //        |         |
    //        ┴         |
    //                  |
    //                  |
    //                  ┴
    //
    // assuming all amplitudes are the same and x, p1, a are on a straight line, pr will also be on a straight line between the previous and next offsetted points

    //const coord_t corner_amplitude = getAmplitude(layer_nr, p1);
    // TODO use above schema
    
    // randFloat = offset / amplitude
    // offset weighted by relative amplitudes and distance to p0
    
    
    coord_t offset_amount = -carry_over.last_amplitude + 2 * carry_over.last_amplitude * carry_over.dist_left_over / settings.dist_between_points;
    Point offset = normal(corner_normal, offset_amount);
    assert(vSizeMM(offset) < 2.0); // TODO: remove debug
    Point pr = p1 + offset;
    result.add(pr);
}

void WaveHalftoning::makeSegmentHalftoned(const unsigned int layer_nr, const Point p0, const Point p1, PolygonRef result, CarryOver& carry_over)
{
    // 'a' is the (next) new point between p0 and p1, offsetted from the point
    // 'x', which is on the line segment p0p1
    const Point p0p1 = p1 - p0;
    carry_over.p0p1_perp = turn90CCW(p0p1);
    const int64_t p0p1_size = vSize(p0p1);
    coord_t dist_to_prev_point = carry_over.dist_left_over; // distance from the last introduced point to the newly introduced one
    int64_t dist_last_point = carry_over.dist_left_over - settings.dist_between_points; // so that 'dist_between_points - (p0p1_size - dist_last_point)' evaulates to 'dist_left_over - p0p1_size'
    for (int64_t p0pa_dist = carry_over.dist_left_over; p0pa_dist < p0p1_size; p0pa_dist += settings.dist_between_points)
    {
        const Point px = p0 + normal(p0p1, p0pa_dist);
        coord_t amplitude = getAmplitude(layer_nr, px);
        carry_over.last_amplitude = amplitude;
        const coord_t offset_amount = carry_over.direction * amplitude;
        Point offset = normal(carry_over.p0p1_perp, offset_amount);
        Point pa = px + offset;
        assert(vSizeMM(offset) < 2.0); // TODO: remove debug
        result.add(pa);
        dist_last_point = p0pa_dist;
        dist_to_prev_point = settings.dist_between_points;
        carry_over.direction *= -1;
    }
    carry_over.dist_left_over = settings.dist_between_points - (p0p1_size - dist_last_point);
    assert(carry_over.dist_left_over >= 0);
    assert(carry_over.dist_left_over < settings.dist_between_points);
}


}//namespace cura
