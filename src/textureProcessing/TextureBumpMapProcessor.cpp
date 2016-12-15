#include "TextureBumpMapProcessor.h"

#include <algorithm> // swap

#include "../utils/optional.h"
#include "../slicer/SlicerSegment.h"

namespace cura 
{

#define POINT_DIST 400
#define AMPLITUDE 3000
#define EXTRA_OFFSET 3000

/*
void TextureBumpMapProcessor::process(std::vector< Slicer* >& slicer_list)
{
    for (Slicer* slicer : slicer_list)
    {
        for (SlicerLayer& layer : slicer->layers)
        {
            process(slicer->mesh, layer);
        }
    }
}
*/

void TextureBumpMapProcessor::processSegmentBumpMap(const TexturedMesh* mesh, const SlicerSegment& slicer_segment, const MatSegment& mat, const Point p0, const Point p1, coord_t& dist_left_over, PolygonRef result)
{

    MatCoord mat_start = mat.start;
    MatCoord mat_end = mat.end;
    if (vSize2(slicer_segment.start - p0) > vSize2(slicer_segment.start - p1))
    {
        std::swap(mat_start, mat_end);
    }
    Point p0p1 = p1 - p0;
    int64_t p0p1_size = vSize(p0p1);
    if (dist_left_over >= p0p1_size)
    {
        dist_left_over -= p0p1_size;
        return;
    }
    Point perp_to_p0p1 = turn90CCW(p0p1);
    int64_t dist_last_point = -1; // p0p1_size * 2 - dist_left_over; // so that p0p1_size - dist_last_point evaulates to dist_left_over - p0p1_size
    // TODO: move start point (which was already moved last iteration
    for (int64_t p0pa_dist = dist_left_over; p0pa_dist < p0p1_size; p0pa_dist += POINT_DIST)
    {
        assert(p0pa_dist >= 0);
        assert(p0pa_dist <= p0p1_size);
        MatCoord mat_coord_now = mat_start;
        mat_coord_now.coords = mat_start.coords + (mat_end.coords - mat_start.coords) * p0pa_dist / p0p1_size;
        float val = mesh->getColor(mat_coord_now, ColourUsage::GREY);
        int offset = val * (AMPLITUDE * 2) - AMPLITUDE + EXTRA_OFFSET;
        Point fuzz = normal(perp_to_p0p1, offset);
        Point pa = p0 + normal(p0p1, p0pa_dist) - fuzz;
        result.add(pa);
        dist_last_point = p0pa_dist;
    }
    // TODO: move end point as well
    float val = mesh->getColor(mat_end, ColourUsage::GREY);
    int r = val * (AMPLITUDE * 2) - AMPLITUDE + EXTRA_OFFSET;
    Point fuzz = normal(perp_to_p0p1, r);
    result.emplace_back(p1 - fuzz);
    assert(dist_last_point >= 0 && "above loop should have run at least once!");
    assert(p0p1_size > dist_last_point);
    dist_left_over = p0p1_size - dist_last_point;
    assert(dist_left_over <= POINT_DIST);
}


void TextureBumpMapProcessor::processBumpMap(const TexturedMesh* mesh, Polygons& layer_polygons)
{
    Polygons results;
    for (PolygonRef poly : layer_polygons)
    {
        // generate points in between p0 and p1
        PolygonRef result = results.newPoly();
        
        coord_t dist_left_over = (POINT_DIST / 2); // the distance to be traversed on the line before making the first new point
        Point* p0 = &poly.back();
        for (Point& p1 : poly)
        { // 'a' is the (next) new point between p0 and p1
            if (*p0 == p1)
            {
                continue;
            }
            SlicerSegment segment(*p0, p1);
            std::optional<std::pair<SlicerSegment, MatSegment>> best_mat_segment_it;
            coord_t best_dist_score = std::numeric_limits<coord_t>::max();
            for (std::unordered_map<SlicerSegment, MatSegment>::iterator it = segment_to_material_segment.begin(); it != segment_to_material_segment.end(); ++it)
            {
                const SlicerSegment& sliced_segment = it->first;
                coord_t dist_score = std::min(
                        vSize2(sliced_segment.start - segment.start) + vSize2(sliced_segment.end - segment.end)
                        , vSize2(sliced_segment.end - segment.start) + vSize2(sliced_segment.start - segment.end)
                    );
                if (dist_score < best_dist_score)
                {
                    best_dist_score = dist_score;
                    best_mat_segment_it = *it;
                }
            }
            if (best_dist_score < 30 * 30) // TODO: magic value of 0.03mm for total stitching distance > should be something like SlicerLayer.cpp::largest_neglected_gap_second_phase (?)
            {
                assert(best_mat_segment_it);
                processSegmentBumpMap(mesh, best_mat_segment_it->first, best_mat_segment_it->second, *p0, p1, dist_left_over, result);
            }
            else
            {
                result.emplace_back(p1);
            }
            p0 = &p1;
        }
        while (result.size() < 3 )
        {
            unsigned int point_idx = poly.size() - 2;
            result.add(poly[point_idx]);
            if (point_idx == 0) { break; }
            point_idx--;
        }
        if (result.size() < 3)
        {
            result.clear();
            for (Point& p : poly)
                result.add(p);
        }
    }
    // a negative offset on two sides of a corner, may introduce complexities in the model which should be removed:
    //         ^↘
    //         ^  ↘
    // <<<<<<<<^<<<<   should become  <<<<<<<<
    //         ^                              ^
    //         ^                              ^
    //         ^                              ^
    layer_polygons = results.removeComplexParts();
}



}//namespace cura
