//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <map>
#include "pathOrderOptimizer.h"
#include "utils/logoutput.h"
#include "utils/SparsePointGridInclusive.h"
#include "utils/linearAlg2D.h"
#include "pathPlanning/LinePolygonsCrossings.h"
#include "pathPlanning/CombPath.h"

#define INLINE static inline

namespace cura {

constexpr coord_t COINCIDENT_POINT_DISTANCE = 5; // In uM. Points closer than this may be considered overlapping / at the same place
constexpr coord_t SQUARED_COINCIDENT_POINT_DISTANCE = COINCIDENT_POINT_DISTANCE * COINCIDENT_POINT_DISTANCE;


ZSeamConfig::ZSeamConfig()
: type(EZSeamType::SHORTEST)
, pos(Point(0, 0))
, corner_pref(EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE)
{
}

ZSeamConfig::ZSeamConfig(const EZSeamType type, const Point pos, const EZSeamCornerPrefType corner_pref)
: type(type)
, pos(pos)
, corner_pref(corner_pref)
{
}


PathOrderOptimizer::Path::Path(const ConstPolygonPointer vertices, const bool is_closed, const size_t start_vertex, const bool backwards)
: vertices(vertices)
, start_vertex(start_vertex)
, is_closed(is_closed)
, backwards(backwards)
{
}

PathOrderOptimizer::PathOrderOptimizer(const Point start_point, const ZSeamConfig config, const Polygons* combing_boundary)
: start_point(start_point)
, config(config)
, combing_boundary((combing_boundary != nullptr && combing_boundary->size() > 0) ? combing_boundary : nullptr)
{
}

void PathOrderOptimizer::addPolygon(const PolygonRef& polygon)
{
    constexpr bool is_closed = true;
    paths.emplace_back(polygon, is_closed);
}

void PathOrderOptimizer::addPolygon(const ConstPolygonRef& polygon)
{
    constexpr bool is_closed = true;
    paths.emplace_back(polygon, is_closed);
}

void PathOrderOptimizer::addPolygons(const Polygons& polygons)
{
    constexpr bool is_closed = true; //All of these are polygons, not polylines.
    for(ConstPolygonRef polygon : polygons)
    {
        paths.emplace_back(polygon, is_closed);
    }
}

void PathOrderOptimizer::addPolyline(const PolygonRef& polyline)
{
    paths.emplace_back(polyline);
}

void PathOrderOptimizer::addPolyline(const ConstPolygonRef& polyline)
{
    paths.emplace_back(polyline);
}

void PathOrderOptimizer::addPolylines(const Polygons& polylines)
{
    for(ConstPolygonRef polyline : polylines)
    {
        paths.emplace_back(polyline);
    }
}

void PathOrderOptimizer::optimize()
{
    // NOTE: Keep this vector fixed-size, it replaces an (non-standard, sized at runtime) array:
    std::vector<bool> picked(polygons.size(), false);
    loc_to_line = nullptr;

    for (unsigned poly_idx = 0; poly_idx < polygons.size(); ++poly_idx) /// find closest point to initial starting point within each polygon +initialize picked
    {
        const ConstPolygonRef poly = *polygons[poly_idx];
        switch (config.type)
        {
            case EZSeamType::USER_SPECIFIED:
                poly_start.push_back(getClosestPointInPolygon(config.pos, poly_idx));
                break;
            case EZSeamType::RANDOM:
                poly_start.push_back(getRandomPointInPolygon(poly_idx));
                break;
            case EZSeamType::SHARPEST_CORNER:
            case EZSeamType::SHORTEST:
            default:
                poly_start.push_back(getClosestPointInPolygon(start_point, poly_idx));
                break;
        }
        assert(poly.size() != 2);
    }


    Point prev_point;
    switch (config.type)
    {
        case EZSeamType::USER_SPECIFIED:
            prev_point = config.pos;
            break;
        case EZSeamType::RANDOM: //TODO: Starting position of the first polygon isn't random.
        case EZSeamType::SHARPEST_CORNER:
        case EZSeamType::SHORTEST:
        default:
            prev_point = start_point;
    }
    for (unsigned int poly_order_idx = 0; poly_order_idx < polygons.size(); poly_order_idx++) /// actual path order optimizer
    {
        int best_poly_idx = -1;
        float bestDist2 = std::numeric_limits<float>::infinity();


        for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
        {
            if (picked[poly_idx] || polygons[poly_idx]->size() < 1) /// skip single-point-polygons
            {
                continue;
            }

            assert (polygons[poly_idx]->size() != 2);

            const Point& p = (*polygons[poly_idx])[poly_start[poly_idx]];
            float dist2 = vSize2f(p - prev_point);
            if (dist2 < bestDist2 && combing_boundary)
            {
                // using direct routing, this poly is the closest so far but as the combing boundary
                // is available see if the travel would cross the combing boundary and, if so, either get
                // the combed distance and use that instead or increase the distance to make it less attractive
                if (PolygonUtils::polygonCollidesWithLineSegment(*combing_boundary, p, prev_point))
                {
                    if ((polygons.size() - poly_order_idx) > 100)
                    {
                        // calculating the combing distance for lots of polygons is too time consuming so, instead,
                        // just increase the distance to penalise travels that hit the combing boundary
                        dist2 *= 5;
                    }
                    else
                    {
                        if (!loc_to_line)
                        {
                            // the combing boundary has been provided so do the initialisation
                            // required to be able to calculate realistic travel distances to the start of new paths
                            const int travel_avoid_distance = 2000; // assume 2mm - not really critical for our purposes
                            loc_to_line = PolygonUtils::createLocToLineGrid(*combing_boundary, travel_avoid_distance);
                        }
                        CombPath comb_path;
                        if (LinePolygonsCrossings::comb(*combing_boundary, *loc_to_line, p, prev_point, comb_path, -40, 0, false))
                        {
                            float dist = 0;
                            Point last_point = p;
                            for (const Point& comb_point : comb_path)
                            {
                                dist += vSize(comb_point - last_point);
                                last_point = comb_point;
                            }
                            dist2 = dist * dist;
                        }
                    }
                }
            }
            if (dist2 < bestDist2)
            {
                best_poly_idx = poly_idx;
                bestDist2 = dist2;
            }

        }


        if (best_poly_idx > -1) /// should always be true; we should have been able to identify the best next polygon
        {
            assert(polygons[best_poly_idx]->size() != 2);

            prev_point = (*polygons[best_poly_idx])[poly_start[best_poly_idx]];

            picked[best_poly_idx] = true;
            poly_order.push_back(best_poly_idx);
        }
        else
        {
            logError("Failed to find next closest polygon.\n");
        }
    }

    if (loc_to_line != nullptr)
    {
        delete loc_to_line;
    }
}

size_t PathOrderOptimizer::getClosestPointInPolygon(const Point prev_point, const size_t poly_idx) const
{
    ConstPolygonRef poly = *polygons[poly_idx];

    int best_point_idx = -1;
    float best_point_score = std::numeric_limits<float>::infinity();
    Point p0 = poly.back();
    for (unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
    {
        const Point& p1 = poly[point_idx];
        const Point& p2 = poly[(point_idx + 1) % poly.size()];
        // when type is SHARPEST_CORNER, actual distance is ignored, we use a fixed distance and decision is based on curvature only
        float dist_score = (config.type == EZSeamType::SHARPEST_CORNER && config.corner_pref != EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE)? 10000 : vSize2(p1 - prev_point);
        const float corner_angle = LinearAlg2D::getAngleLeft(p0, p1, p2) / M_PI; // 0 -> 2
        float corner_shift;
        if (config.type == EZSeamType::SHORTEST)
        {
            // the more a corner satisfies our criteria, the closer it appears to be
            // shift 10mm for a very acute corner
            corner_shift = 10000 * 10000;
        }
        else
        {
            // the larger the distance from prev_point to p1, the more a corner will "attract" the seam
            // so the user has some control over where the seam will lie.

            // the divisor here may need adjusting to obtain the best results (TBD)
            corner_shift = dist_score / 10;
        }
        switch (config.corner_pref)
        {
            case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_INNER:
                if (corner_angle > 1)
                {
                    // p1 lies on a concave curve so reduce the distance to favour it
                    // the more concave the curve, the more we reduce the distance
                    dist_score -= (corner_angle - 1) * corner_shift;
                }
                break;
            case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_OUTER:
                if (corner_angle < 1)
                {
                    // p1 lies on a convex curve so reduce the distance to favour it
                    // the more convex the curve, the more we reduce the distance
                    dist_score -= (1 - corner_angle) * corner_shift;
                }
                break;
            case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_ANY:
                // the more curved the region, the more we reduce the distance
                dist_score -= fabs(corner_angle - 1) * corner_shift;
                break;
            case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_WEIGHTED:
            {
                //More curve is better score (reduced distance), but slightly in favour of concave curves.
                float dist_score_corner = fabs(corner_angle - 1) * corner_shift;
                if (corner_angle < 1)
                {
                    dist_score_corner *= 2;
                }
                dist_score -= dist_score_corner;
                break;
            }
            case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE:
            default:
                // do nothing
                break;
        }
        if (dist_score < best_point_score)
        {
            best_point_idx = point_idx;
            best_point_score = dist_score;
        }
        p0 = p1;
    }
    return best_point_idx;
}

size_t PathOrderOptimizer::getRandomPointInPolygon(const size_t poly_idx) const
{
    return rand() % polygons[poly_idx]->size();
}

}//namespace cura
