#include "TextureBumpMapProcessor.h"

#include <algorithm> // swap

#include "../utils/optional.h"
#include "../slicer/SlicerSegment.h"

namespace cura 
{

#define SLICE_SEGMENT_SNAP_GAP 20

TextureBumpMapProcessor::TextureBumpMapProcessor(const TextureBumpMapProcessor::Settings settings)
: settings(settings)
, loc_to_slice(SLICE_SEGMENT_SNAP_GAP)
{

}


void TextureBumpMapProcessor::registerTexturedFaceSlice(SlicerSegment face_segment, MatSegment texture_segment)
{
    TexturedFaceSlice slice{face_segment, texture_segment};
    loc_to_slice.insert(face_segment.start, slice);
    loc_to_slice.insert(face_segment.end, slice);
}

std::optional<TextureBumpMapProcessor::TexturedFaceSlice> TextureBumpMapProcessor::getTexturedFaceSlice(Point p0, Point p1)
{
    std::vector<TexturedFaceSlice> nearby_slices = loc_to_slice.getNearby(p0, SLICE_SEGMENT_SNAP_GAP);
    std::optional<TexturedFaceSlice> best;
    coord_t best_dist_score = std::numeric_limits<coord_t>::max();

    for (TexturedFaceSlice& slice : nearby_slices)
    {
        coord_t dist_score = std::min(
                vSize2(slice.face_segment.start - p0) + vSize2(slice.face_segment.end - p1)
                , vSize2(slice.face_segment.end - p0) + vSize2(slice.face_segment.start - p1)
            );
        if (dist_score < best_dist_score)
        {
            best = slice;
            best_dist_score = dist_score;
        }
    }
    if (best_dist_score > SLICE_SEGMENT_SNAP_GAP * SLICE_SEGMENT_SNAP_GAP * 4) // TODO: this condition doesn't follow exactly from using SLICE_SEGMENT_SNAP_GAP and the quadratic dist score
    {
        return std::optional<TextureBumpMapProcessor::TexturedFaceSlice>();
    }
    return best;
}


void TextureBumpMapProcessor::processSegmentBumpMap(unsigned int layer_nr, const SlicerSegment& slicer_segment, const MatSegment& mat, const Point p0, const Point p1, coord_t& dist_left_over, PolygonRef result)
{
    MatCoord mat_start = mat.start;
    MatCoord mat_end = mat.end;
    assert(mat_start.mat == mat_end.mat && "texture across face must be from one material!");
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
    for (int64_t p0pa_dist = dist_left_over; p0pa_dist < p0p1_size; p0pa_dist += settings.point_distance)
    {
        assert(p0pa_dist >= 0);
        assert(p0pa_dist <= p0p1_size);
        MatCoord mat_coord_now = mat_start;
        mat_coord_now.coords = mat_start.coords + (mat_end.coords - mat_start.coords) * p0pa_dist / p0p1_size;
        float val = mat_coord_now.getColor(ColourUsage::GREY);
        int offset = val * (settings.amplitude * 2) - settings.amplitude + settings.offset;
        Point fuzz = normal(perp_to_p0p1, offset);
        Point pa = p0 + normal(p0p1, p0pa_dist) - fuzz;
        result.add(pa);
        dist_last_point = p0pa_dist;
    }
    // TODO: move end point as well
    float val = mat_end.getColor(ColourUsage::GREY);
    int r = val * (settings.amplitude * 2) - settings.amplitude + settings.offset;
    Point fuzz = normal(perp_to_p0p1, r);
    result.emplace_back(p1 - fuzz);
    assert(dist_last_point >= 0 && "above loop should have run at least once!");
    assert(p0p1_size > dist_last_point);
    dist_left_over = p0p1_size - dist_last_point;
    assert(dist_left_over <= settings.point_distance);
}


void TextureBumpMapProcessor::processBumpMap(Polygons& layer_polygons, unsigned int layer_nr)
{
    Polygons results;
    for (PolygonRef poly : layer_polygons)
    {
        // generate points in between p0 and p1
        PolygonRef result = results.newPoly();
        
        coord_t dist_left_over = (settings.point_distance / 2); // the distance to be traversed on the line before making the first new point
        Point* p0 = &poly.back();
        for (Point& p1 : poly)
        { // 'a' is the (next) new point between p0 and p1
            if (*p0 == p1)
            {
                continue;
            }
            std::optional<TexturedFaceSlice> textured_face_slice = getTexturedFaceSlice(*p0, p1);
            if (textured_face_slice)
            {
                processSegmentBumpMap(layer_nr, textured_face_slice->face_segment, textured_face_slice->mat_segment, *p0, p1, dist_left_over, result);
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
