//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <map>
#include "OrderOptimizer.h"
#include "utils/logoutput.h"
#include "utils/SparsePointGridInclusive.h"
#include "utils/linearAlg2D.h"

#define INLINE static inline

namespace arachne {

/**
*
*/
void OrderOptimizer::optimize()
{
    // NOTE: Keep this vector fixed-size, it replaces an (non-standard, sized at runtime) array:
    std::vector<bool> picked(polys.size(), false);
    loc_to_line = nullptr;

    for (unsigned poly_idx = 0; poly_idx < polys.size(); ++poly_idx) /// find closest point to initial starting point within each poly +initialize picked
    {
        const ConstPolygonRef poly = *polys[poly_idx].poly;
        switch (config.type)
        {
            case EZSeamType::USER_SPECIFIED:
                polyStart.push_back(getClosestPointInPoly(config.pos, poly_idx));
                break;
            case EZSeamType::RANDOM:
                polyStart.push_back(getRandomPointInPoly(poly_idx));
                break;
            case EZSeamType::SHARPEST_CORNER:
            case EZSeamType::SHORTEST:
            default:
                polyStart.push_back(getClosestPointInPoly(startPoint, poly_idx));
                break;
        }
//         assert(poly.size() != 2);
    }


    Point prev_point;
    switch (config.type)
    {
        case EZSeamType::USER_SPECIFIED:
            prev_point = config.pos;
            break;
        case EZSeamType::RANDOM: //TODO: Starting position of the first poly isn't random.
        case EZSeamType::SHARPEST_CORNER:
        case EZSeamType::SHORTEST:
        default:
            prev_point = startPoint;
    }
    for (unsigned int poly_order_idx = 0; poly_order_idx < polys.size(); poly_order_idx++) /// actual path order optimizer
    {
        int best_poly_idx = -1;
        float bestDist2 = std::numeric_limits<float>::infinity();


        for (unsigned int poly_idx = 0; poly_idx < polys.size(); poly_idx++)
        {
            if (picked[poly_idx] || polys[poly_idx].poly->size() < 1) /// skip single-point-polys
            {
                continue;
            }

//             assert (polys[poly_idx]->size() != 2);

            size_t start_idx = getClosestPointInPoly(prev_point, poly_idx);
            polyStart[poly_idx] = start_idx;
            const Point& p = (*polys[poly_idx].poly)[start_idx];
            float dist2 = vSize2f(p - prev_point);
            if (dist2 < bestDist2)
            {
                best_poly_idx = poly_idx;
                bestDist2 = dist2;
            }

        }


        if (best_poly_idx > -1) /// should always be true; we should have been able to identify the best next poly
        {
//             assert(polys[best_poly_idx]->size() != 2);
            if (polys[best_poly_idx].is_closed)
            {
                prev_point = (*polys[best_poly_idx].poly)[polyStart[best_poly_idx]];
            }
            else
            {
                if (polyStart[best_poly_idx] == 0)
                    prev_point = polys[best_poly_idx].poly->back();
                else
                    prev_point = polys[best_poly_idx].poly->front();
            }

            picked[best_poly_idx] = true;
            polyOrder.push_back(best_poly_idx);
        }
        else
        {
            logError("Failed to find next closest poly.\n");
        }
    }

    if (loc_to_line != nullptr)
    {
        delete loc_to_line;
    }
}

int OrderOptimizer::getClosestPointInPoly(Point prev_point, int poly_idx)
{
    ConstPolygonRef poly = *polys[poly_idx].poly;
    if (polys[poly_idx].is_closed)
    {
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
                    //More curve is better score (reduced distance), but slightly in favour of concave curves.
                    dist_score -= fabs(corner_angle - 0.8) * corner_shift;
                    break;
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
    else
    {
        Point start = poly.front();
        Point end = poly.back();
        if (vSize2(start - prev_point) < vSize2(end - prev_point))
        {
            return 0;
        }
        else
        {
            return poly.size() - 1;
        }
    }
}

int OrderOptimizer::getRandomPointInPoly(int poly_idx)
{
    ConstPolygonRef poly = *polys[poly_idx].poly;

    if (polys[poly_idx].is_closed)
    {
        return rand() % poly.size();
    }
    else
    {
        int pos = rand() % 2;
        if (pos)
        {
            return poly.size() - 1;
        }
        else
        {
            return 0;
        }
    }
}

static inline bool pointsAreCoincident(const Point& a, const Point& b)
{
    return vSize2(a - b) < 25; // points are closer than 5uM, consider them coincident
}


}//namespace cura
