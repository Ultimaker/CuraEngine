/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "FuzzyWalls.h"

#define NORMAL_LENGTH 10000

namespace cura 
{

FuzzyWalls::FuzzyWalls(const SliceMeshStorage& mesh)
: settings(&mesh)
{
    if (mesh.getSettingBoolean("fuzz_map_enabled"))
    {
        assert(mesh.texture_proximity_processor && "texture_proximity_processor should have been initialized");
        getAmplitude = [&mesh, this](const unsigned int layer_nr, const Point p)
        {
            assert(mesh.texture_proximity_processor && "When fuzz_map_enabled there has to be a texture proximity processor!");
            TextureProximityProcessor& texture_proximity_processor = *mesh.texture_proximity_processor;
            float color = texture_proximity_processor.getColor(p, layer_nr, settings.color_usage, 0.0); // TODO change default 0.0
            coord_t ret = color * settings.max_amplitude;
            return ret;
        };
    }
    else
    {
        getAmplitude = [this](const unsigned int layer_nr, const Point p)
        {
            return settings.max_amplitude;
        };
    }
}

Polygons FuzzyWalls::makeFuzzy(const SliceMeshStorage& mesh, const unsigned int layer_nr, const Polygons& in)
{
    Polygons results;
    if (in.size() == 0)
    {
        return results;
    }

    flows.reserve(in.size());
    for (ConstPolygonRef poly : const_cast<Polygons&>(in))
    {
        assert(poly.size() >= 3);
        // generate points in between p0 and p1
        PolygonRef result = results.newPoly();
        flows.emplace_back(); // keep flows aligned with the result
        flows.back().reserve(poly.size());

        Point p0 = poly[poly.size() - 2];
        Point p1 = poly.back();
        for (int p0_idx = poly.size() - 2; p0_idx >= 0; p0_idx--)
        { // p0 is the last point before p1 which is different from p1
            p0 = poly[p0_idx];
        }
        CarryOver carry_over;
        carry_over.dist_left_over = (settings.min_dist_between_points + rand() % settings.range_random_point_dist) / 2;
        carry_over.step_size = carry_over.dist_left_over;
        carry_over.offset_random = 0.0; // unused in the first iteration since carry_over.step_size = carry_over.dist_left_over; see makeCornerFuzzy
        carry_over.next_offset_random = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2.0 - 1.0;
        carry_over.p0p1_perp = turn90CCW(p1 - p0);
        // 'x' is the previous location from where a randomly offsetted new point between p-1 and p0 was created
        for (Point p2 : poly)
        {
            if (p2 == p1)
            {
                continue;
            }
            makeCornerFuzzy(layer_nr, p0, p1, p2, carry_over, result);
            makeSegmentFuzzy(layer_nr, p1, p2, result, carry_over);
            p0 = p1;
            p1 = p2;
        }
        while (result.size() < 3 )
        {
            unsigned int point_idx = poly.size() - 2;
            result.add(poly[point_idx]);
            flows.back().push_back(1.0);
            if (point_idx == 0)
            {
                break;
            }
            point_idx--;
        }
        if (result.size() > 0)
        { // compute flow of the newly introduced segment
            const Point p0 = result.back();
            const Point p1 = result.back();
            const coord_t length = vSize(p1 - p0);
            const float flow_here = (length == 0)? 0.0 : (float) (carry_over.step_size - carry_over.dist_left_over) / (float) length;
            flows.back().push_back(flow_here);
        }
        if (result.size() < 3)
        {
            result.clear();
            flows.back().clear();
            for (const Point& p : poly)
            {
                result.add(p);
                flows.back().push_back(1.0);
            }
        }
        assert(result.size() == flows.back().size());
    }
    return results;
}

void FuzzyWalls::makeCornerFuzzy(const unsigned int layer_nr, const Point p0, const Point p1, const Point p2, const CarryOver carry_over, PolygonRef result)
{
    const Point p0p1_perp = carry_over.p0p1_perp;
    const Point p1p2 = p2 - p1;
    const Point p1p2_perp = turn90CCW(p1p2);
    const Point corner_normal = normal(p0p1_perp, NORMAL_LENGTH) + normal(p1p2_perp, NORMAL_LENGTH);

    // x is the last point which was offsetted
    // a is the next point to be offsetted
    //
    //             step_size
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

    const coord_t corner_amplitude = getAmplitude(layer_nr, p1);
    // randFloat = offset / amplitude
    // offset weighted by relative amplitudes and distance to p0
    assert(carry_over.step_size > 0);
    const coord_t pxp1_dist = (carry_over.step_size - carry_over.dist_left_over);
    assert(pxp1_dist >= 0);
    const coord_t p1pa_dist = carry_over.dist_left_over;
    const coord_t offset_contribution_0 = corner_amplitude * pxp1_dist * carry_over.offset_random;
    const coord_t offset_contribution_2 = corner_amplitude * p1pa_dist * carry_over.next_offset_random;
    const coord_t offset = (offset_contribution_0 + offset_contribution_2) / carry_over.step_size;

    Point fuzz = normal(corner_normal, offset);
    Point pr = p1 + fuzz;
    if (result.size() > 0)
    { // compute flow of the newly introduced segment
        const Point last = result.back();
        const coord_t length = vSize(last - pr);
        const float flow_here = (length == 0)? 0.0 : (float) pxp1_dist / (float) length;
        flows.back().push_back(flow_here);
    }
    result.add(pr);
}

void FuzzyWalls::makeSegmentFuzzy(const unsigned int layer_nr, const Point p0, const Point p1, PolygonRef result, CarryOver& carry_over)
{
    // 'a' is the (next) new point between p0 and p1, offsetted from the point
    // 'x', which is on the line segment p0p1
    const Point p0p1 = p1 - p0;
    carry_over.p0p1_perp = turn90CCW(p0p1);
    const int64_t p0p1_size = vSize(p0p1);
    coord_t dist_to_prev_point = carry_over.dist_left_over; // distance from the last introduced point to the newly introduced one
    int64_t dist_last_point = carry_over.dist_left_over - carry_over.step_size; // so that 'carry_over.step_size - (p0p1_size - dist_last_point)' evaulates to 'dist_left_over - p0p1_size'
    for (int64_t p0pa_dist = carry_over.dist_left_over; p0pa_dist < p0p1_size; p0pa_dist += carry_over.step_size)
    {
        const Point px = p0 + normal(p0p1, p0pa_dist);
        coord_t amplitude = getAmplitude(layer_nr, px);
        if (amplitude == 0)
        {
            amplitude = 1;
        }
        carry_over.offset_random = carry_over.next_offset_random;
        carry_over.next_offset_random = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2.0 - 1.0;
        const coord_t offset = carry_over.offset_random * amplitude;
        Point fuzz = normal(carry_over.p0p1_perp, offset);
        Point pa = px + fuzz;
        if (result.size() > 0)
        { // compute flow of the newly introduced segment
            const Point last = result.back();
            const coord_t length = vSize(last - pa);
            const float flow_here = (length == 0)? 0.0 : (float) dist_to_prev_point / (float) length;
            flows.back().push_back(flow_here);
        }
        result.add(pa);
        dist_last_point = p0pa_dist;
        carry_over.step_size = settings.min_dist_between_points + rand() % settings.range_random_point_dist;
        dist_to_prev_point = carry_over.step_size;
    }
    carry_over.dist_left_over = carry_over.step_size - (p0p1_size - dist_last_point);
    assert(carry_over.dist_left_over >= 0);
    assert(carry_over.dist_left_over < carry_over.step_size);
}

float FuzzyWalls::getFlow(const Polygons& from, unsigned int poly_idx, unsigned int from_point_idx, unsigned int to_point_idx)
{
    assert(from.size() == flows.size());
    assert(poly_idx < flows.size());
    assert(from[poly_idx].size() == flows[poly_idx].size());
    assert((from_point_idx + 1) % flows[poly_idx].size() == to_point_idx);
    return flows[poly_idx][from_point_idx];
}



}//namespace cura