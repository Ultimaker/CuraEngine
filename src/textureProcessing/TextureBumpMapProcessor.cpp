#include "TextureBumpMapProcessor.h"

#include <algorithm> // swap
#include <cmath> // fabs

#include "../utils/optional.h"
#include "../utils/linearAlg2D.h"
#include "../slicer/SlicerSegment.h"

namespace cura 
{

#define SLICE_SEGMENT_SNAP_GAP 20

TextureBumpMapProcessor::TextureBumpMapProcessor(TexturedMesh* mesh, const TextureBumpMapProcessor::Settings settings, FaceNormalStorage* face_normal_storage)
: mesh(mesh)
, settings(settings)
, face_normal_storage(face_normal_storage)
, loc_to_slice(SLICE_SEGMENT_SNAP_GAP)
{

}


void TextureBumpMapProcessor::registerTexturedFaceSlice(SlicerSegment face_segment, MatSegment texture_segment)
{
    assert(face_segment.faceIndex >= 0);
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
    if (vSize2(best->face_segment.start - p0) > vSize2(best->face_segment.start - p1))
    {
        std::swap(best->face_segment.start, best->face_segment.end);
    }
    assert(best->face_segment.faceIndex >= 0);
    return best;
}

coord_t TextureBumpMapProcessor::getOffset(const float color, const int face_idx)
{
    coord_t extra_offset = 0;
    if (face_normal_storage && face_idx >= 0)
    {
        assert(face_idx >= 0);
        float tan_angle = face_normal_storage->getFaceTanAngle(face_idx);
        float abs_tan_angle = std::fabs(tan_angle);
        abs_tan_angle = std::min(abs_tan_angle, settings.max_tan_correction_angle);
        extra_offset = settings.face_angle_correction * (color - 0.5) * abs_tan_angle * settings.layer_height;
        // (color - 0.5) so that the color causes either an outset or an inset which is
        // within the range [-0.5, 0.5] so that when at max it will coincide with the min on the previous layer
    }
    return color * (settings.amplitude * 2) - settings.amplitude + settings.offset + extra_offset;
}


coord_t TextureBumpMapProcessor::getCornerDisregard(Point p0, Point p1, Point p2, std::optional<TextureBumpMapProcessor::TexturedFaceSlice>& textured_face_slice, std::optional<TextureBumpMapProcessor::TexturedFaceSlice>& next_textured_face_slice)
{
    float color0 = (textured_face_slice)? textured_face_slice->mat_segment.end.getColor(ColourUsage::GREY) : 0.0; // TODO default color for non textured should be settable
    float color1 = (next_textured_face_slice)? next_textured_face_slice->mat_segment.start.getColor(ColourUsage::GREY) : 0.0; // TODO default color for non textured should be settable
    const int face_0_idx = (textured_face_slice)? textured_face_slice->face_segment.faceIndex : -1;
    const int face_1_idx = (next_textured_face_slice)? next_textured_face_slice->face_segment.faceIndex : -1;
    coord_t offset = (getOffset(color0, face_0_idx) + getOffset(color1, face_1_idx)) / 2;
    if ((LinearAlg2D::pointIsLeftOfLine(p1, p0, p2) < 0) == (offset > 0))
    {
        return 0;
    }
    Point v01 = p1 - p0;
    Point v12 = p2 - p1;
    assert(p0 != p1 && "Code below depends on v01 not being of zer o size");
    assert(p1 != p2 && "This function assumes the three points are different");
    Point n01 = normal(turn90CCW(v01), -1000);
    Point n12 = normal(turn90CCW(v12), -1000);
    Point corner_normal = n01 + n12;
    coord_t corner_normal_size2 = vSize2(corner_normal);
    coord_t normal_aspect = dot(corner_normal, v01) / vSize(v01); // The aspect of the corner normal along v01 (might be negative)
    coord_t dist_aspect = sqrt(std::max((coord_t)1, corner_normal_size2 - normal_aspect * normal_aspect)); // The distance of the end of the normal vector to v01 or v12
    // ^ due to rounding errors 'corner_normal_size2 - normal_aspect^2' may be smaller than zero; because of division on line below should be at least 1
    coord_t disregard = std::abs(offset * normal_aspect) / dist_aspect;
    assert(disregard >= 0);
    return disregard;
}


void TextureBumpMapProcessor::processSegmentBumpMap(unsigned int layer_nr, const SlicerSegment& slicer_segment, const MatSegment& mat, const Point p0, const Point p1, coord_t& dist_left_over, coord_t corner_disregard_p1, PolygonRef result)
{
    assert(mat.start.mat == mat.end.mat && "texture across face must be from one material!");

    Point p0p1 = p1 - p0;
    int64_t p0p1_size = vSize(p0p1);
    if (dist_left_over >= p0p1_size - corner_disregard_p1)
    {
        dist_left_over -= p0p1_size;
        return;
    }

    Point perp_to_p0p1 = turn90CCW(p0p1);
    int64_t dist_last_point = -1; // p0p1_size * 2 - dist_left_over; // so that p0p1_size - dist_last_point evaulates to dist_left_over - p0p1_size
    for (int64_t p0pa_dist = dist_left_over; p0pa_dist < p0p1_size - corner_disregard_p1; p0pa_dist += settings.point_distance)
    {
        assert(p0pa_dist >= 0);
        assert(p0pa_dist <= p0p1_size);
        MatCoord mat_coord_now = mat.start;
        mat_coord_now.coords = mat.start.coords + (mat.end.coords - mat.start.coords) * p0pa_dist / p0p1_size;
        float val = mat_coord_now.getColor(ColourUsage::GREY);
        int offset = getOffset(val, slicer_segment.faceIndex);
        Point fuzz = normal(perp_to_p0p1, offset);
        Point pa = p0 + normal(p0p1, p0pa_dist) - fuzz;
        result.add(pa);
        dist_last_point = p0pa_dist;
    }
    assert(dist_last_point >= 0 && "above loop should have run at least once!");
    assert(p0p1_size > dist_last_point);
    dist_left_over = p0p1_size - dist_last_point;
    assert(dist_left_over <= settings.point_distance + corner_disregard_p1);
}


void TextureBumpMapProcessor::processBumpMap(Polygons& layer_polygons, unsigned int layer_nr)
{
    if (layer_polygons.size() == 0)
    {
        return;
    }

    Polygons preprocessed;
    for (PolygonRef poly : layer_polygons)
    { // remove duplicate points
        PolygonRef preprocessed_poly = preprocessed.newPoly();
        Point p0 = poly.back();
        for (const Point p1 : poly)
        {
            if (p1 == p0)
                continue;
            preprocessed_poly.add(p1);
            p0 = p1;
        }
    }

    Polygons results;
    for (PolygonRef poly : preprocessed)
    {
        if (poly.size() < 3)
        {
            results.add(poly);
            continue;
        }
        PolygonRef result = results.newPoly();

        std::vector<std::optional<TexturedFaceSlice>> texture_poly;
        {
            Point p0 = poly.back();
            for (Point& p1 : poly)
            {
                texture_poly.emplace_back(getTexturedFaceSlice(p0, p1));
                p0 = p1;
            }
        }

        coord_t corner_disregard_p0 = getCornerDisregard(poly[poly.size() - 2], poly.back(), poly[0], texture_poly.back(), texture_poly[0]);; // TODO
        coord_t dist_left_over = (settings.point_distance / 2); // the distance to be traversed on the line before making the first new point
        Point* p0 = &poly.back();
        for (unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
        { // 'a' is the (next) new point between p0 and p1
            Point& p1 = poly[point_idx];
            unsigned int next_point_idx = (point_idx + 1 == poly.size())? 0 : point_idx + 1;
            Point& p2 = poly[next_point_idx];
            if (*p0 == p1)
            {
                continue;
            }
            std::optional<TexturedFaceSlice>& textured_face_slice = texture_poly[point_idx];
            std::optional<TexturedFaceSlice>& next_textured_face_slice = texture_poly[next_point_idx];

            coord_t corner_disregard_p1 = getCornerDisregard(*p0, p1, p2, textured_face_slice, next_textured_face_slice); // TODO
            if (dist_left_over < corner_disregard_p0)
            {
                dist_left_over = corner_disregard_p0;
            }

            if (textured_face_slice)
            {
                processSegmentBumpMap(layer_nr, textured_face_slice->face_segment, textured_face_slice->mat_segment, *p0, p1, dist_left_over, corner_disregard_p1, result);
            }
            else
            {
                coord_t p0p1_size2 = vSize2(p1 - *p0);
                if (p0p1_size2 < dist_left_over * dist_left_over)
                {
                    dist_left_over -= sqrt(p0p1_size2);
                }
                else
                {
                    result.emplace_back(*p0);
                    result.emplace_back(p1);
                    dist_left_over = settings.point_distance;
                }
            }

            if (corner_disregard_p1 == 0
                && (textured_face_slice || next_textured_face_slice)
                && (textured_face_slice || !shorterThen(p1 - *p0, SLICE_SEGMENT_SNAP_GAP)) // don't introduce corner points for gap closer poly segments
                && (next_textured_face_slice || !shorterThen(p2 - p1, SLICE_SEGMENT_SNAP_GAP)) // don't introduce corner points for gap closer poly segments
                )
            { // add point for outward corner
                // TODO: remove code duplication with getCornerDisregard
                float color0 = (textured_face_slice)? textured_face_slice->mat_segment.end.getColor(ColourUsage::GREY) : 0.0; // TODO default color for non textured should be settable
                float color1 = (next_textured_face_slice)? next_textured_face_slice->mat_segment.start.getColor(ColourUsage::GREY) : 0.0; // TODO default color for non textured should be settable
                const int face_0_idx = (textured_face_slice)? textured_face_slice->face_segment.faceIndex : -1;
                const int face_1_idx = (next_textured_face_slice)? next_textured_face_slice->face_segment.faceIndex : -1;
                coord_t offset = (getOffset(color0, face_0_idx) + getOffset(color1, face_1_idx)) / 2;
                Point v01 = p1 - *p0;
                Point v12 = p2 - p1;
                Point n01 = normal(turn90CCW(v01), -1000);
                Point n12 = normal(turn90CCW(v12), -1000);
                Point corner_normal = normal(n01 + n12, offset);
                result.add(p1 + corner_normal);
            }
            p0 = &p1;
            corner_disregard_p0 = corner_disregard_p1;
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
