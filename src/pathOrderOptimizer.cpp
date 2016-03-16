/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "pathOrderOptimizer.h"
#include "utils/logoutput.h"
#include "utils/BucketGrid2D.h"

#define INLINE static inline

namespace cura {

/**
*
*/
void PathOrderOptimizer::optimize()
{
    bool picked[polygons.size()];
    memset(picked, false, sizeof(bool) * polygons.size());/// initialized as falses
    
    for (PolygonRef poly : polygons) /// find closest point to initial starting point within each polygon +initialize picked
    {
        int best = -1;
        float bestDist = std::numeric_limits<float>::infinity();
        for (unsigned int point_idx = 0; point_idx < poly.size(); point_idx++) /// get closest point in polygon
        {
            float dist = vSize2f(poly[point_idx] - startPoint);
            if (dist < bestDist)
            {
                best = point_idx;
                bestDist = dist;
            }
        }
        polyStart.push_back(best);
        //picked.push_back(false); /// initialize all picked values as false

        assert(poly.size() != 2);
    }


    Point prev_point = startPoint;
    for (unsigned int poly_order_idx = 0; poly_order_idx < polygons.size(); poly_order_idx++) /// actual path order optimizer
    {
        int best_poly_idx = -1;
        float bestDist = std::numeric_limits<float>::infinity();


        for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
        {
            if (picked[poly_idx] || polygons[poly_idx].size() < 1) /// skip single-point-polygons
            {
                continue;
            }

            assert (polygons[poly_idx].size() != 2);

            float dist = vSize2f(polygons[poly_idx][polyStart[poly_idx]] - prev_point);
            if (dist < bestDist)
            {
                best_poly_idx = poly_idx;
                bestDist = dist;
            }

        }


        if (best_poly_idx > -1) /// should always be true; we should have been able to identify the best next polygon
        {
            assert(polygons[best_poly_idx].size() != 2);

            prev_point = polygons[best_poly_idx][polyStart[best_poly_idx]];

            picked[best_poly_idx] = true;
            polyOrder.push_back(best_poly_idx);
        }
        else
        {
            logError("Failed to find next closest polygon.\n");
        }
    }

    prev_point = startPoint;
    for (unsigned int poly_idx = 0; poly_idx < polyOrder.size(); poly_idx++) /// decide final starting points in each polygon
    {
        int ordered_poly_idx = polyOrder[poly_idx];
        int point_idx = getPolyStart(prev_point, ordered_poly_idx);
        polyStart[poly_idx] = point_idx;
        prev_point = polygons[ordered_poly_idx][point_idx];

    }
}

int PathOrderOptimizer::getPolyStart(Point prev_point, int poly_idx)
{
    switch (type)
    {
        case EZSeamType::BACK:      return getFarthestPointInPolygon(poly_idx); 
        case EZSeamType::RANDOM:    return getRandomPointInPolygon(poly_idx); 
        case EZSeamType::SHORTEST:  return getClosestPointInPolygon(prev_point, poly_idx);
        default:                    return getClosestPointInPolygon(prev_point, poly_idx);
    }
}


int PathOrderOptimizer::getClosestPointInPolygon(Point prev_point, int poly_idx)
{
    PolygonRef poly = polygons[poly_idx];
    int best_point_idx = -1;
    float bestDist = std::numeric_limits<float>::infinity();
    bool orientation = poly.orientation();
    for(unsigned int i_point=0 ; i_point<poly.size() ; i_point++)
    {
        float dist = vSize2f(poly[i_point] - prev_point);
        Point n0 = normal(poly[(i_point-1+poly.size())%poly.size()] - poly[i_point], 2000);
        Point n1 = normal(poly[i_point] - poly[(i_point + 1) % poly.size()], 2000);
        float dot_score = dot(n0, n1) - dot(crossZ(n0), n1); /// prefer binnenbocht
        if (orientation)
            dot_score = -dot_score;
        if (dist + dot_score < bestDist)
        {
            best_point_idx = i_point;
            bestDist = dist;
        }
    }
    return best_point_idx;
}

int PathOrderOptimizer::getRandomPointInPolygon(int poly_idx)
{
    return rand() % polygons[poly_idx].size();
}


int PathOrderOptimizer::getFarthestPointInPolygon(int poly_idx)
{
    PolygonRef poly = polygons[poly_idx];
    int best_point_idx = -1;
    float best_y = std::numeric_limits<float>::min();
    for(unsigned int point_idx=0 ; point_idx<poly.size() ; point_idx++)
    {
        if (poly[point_idx].Y > best_y)
        {
            best_point_idx = point_idx;
            best_y = poly[point_idx].Y;
        }
    }
    return best_point_idx;
}


/**
*
*/
void LineOrderOptimizer::optimize()
{
    int gridSize = 5000; // the size of the cells in the hash grid.
    BucketGrid2D<unsigned int> line_bucket_grid(gridSize);
    bool picked[polygons.size()];
    memset(picked, false, sizeof(bool) * polygons.size());/// initialized as falses
    
    for(unsigned int i_polygon=0 ; i_polygon<polygons.size() ; i_polygon++) /// find closest point to initial starting point within each polygon +initialize picked
    {
        int best = -1;
        float bestDist = std::numeric_limits<float>::infinity();
        PolygonRef poly = polygons[i_polygon];
        for(unsigned int i_point=0; i_point<poly.size(); i_point++) /// get closest point from polygon
        {
            float dist = vSize2f(poly[i_point] - startPoint);
            if (dist < bestDist)
            {
                best = i_point;
                bestDist = dist;
            }
        }
        polyStart.push_back(best);

        assert(poly.size() == 2);

        line_bucket_grid.insert(poly[0], i_polygon);
        line_bucket_grid.insert(poly[1], i_polygon);

    }


    Point incommingPerpundicularNormal(0, 0);
    Point prev_point = startPoint;
    for(unsigned int i_polygon=0 ; i_polygon<polygons.size() ; i_polygon++) /// actual path order optimizer
    {
        int best = -1;
        float bestDist = std::numeric_limits<float>::infinity();

        for(unsigned int i_close_line_polygon :  line_bucket_grid.findNearbyObjects(prev_point)) /// check if single-line-polygon is close to last point
        {
            if (picked[i_close_line_polygon] || polygons[i_close_line_polygon].size() < 1)
                continue;


            checkIfLineIsBest(i_close_line_polygon, best, bestDist, prev_point, incommingPerpundicularNormal);

        }

        if (best == -1) /// if single-line-polygon hasn't been found yet
        {
           for(unsigned int i_polygon=0 ; i_polygon<polygons.size() ; i_polygon++)
            {
                if (picked[i_polygon] || polygons[i_polygon].size() < 1) /// skip single-point-polygons
                    continue;
                assert(polygons[i_polygon].size() == 2);

                checkIfLineIsBest(i_polygon, best, bestDist, prev_point, incommingPerpundicularNormal);

            }
        }

        if (best > -1) /// should always be true; we should have been able to identify the best next polygon
        {
            assert(polygons[best].size() == 2);

            int endIdx = polyStart[best] * -1 + 1; /// 1 -> 0 , 0 -> 1
            prev_point = polygons[best][endIdx];
            incommingPerpundicularNormal = crossZ(normal(polygons[best][endIdx] - polygons[best][polyStart[best]], 1000));

            picked[best] = true;
            polyOrder.push_back(best);
        }
        else
            logError("Failed to find next closest line.\n");
    }

    prev_point = startPoint;
    for(unsigned int n=0; n<polyOrder.size(); n++) /// decide final starting points in each polygon
    {
        int nr = polyOrder[n];
        PolygonRef poly = polygons[nr];
        int best = -1;
        float bestDist = std::numeric_limits<float>::infinity();
        bool orientation = poly.orientation();
        for(unsigned int i=0;i<poly.size(); i++)
        {
            float dist = vSize2f(polygons[nr][i] - prev_point);
            Point n0 = normal(poly[(i+poly.size()-1)%poly.size()] - poly[i], 2000);
            Point n1 = normal(poly[i] - poly[(i + 1) % poly.size()], 2000);
            float dot_score = dot(n0, n1) - dot(crossZ(n0), n1);
            if (orientation)
                dot_score = -dot_score;
            if (dist + dot_score < bestDist)
            {
                best = i;
                bestDist = dist + dot_score;
            }
        }

        polyStart[nr] = best;
        assert(poly.size() == 2);
        prev_point = poly[best *-1 + 1]; /// 1 -> 0 , 0 -> 1

    }
}

inline void LineOrderOptimizer::checkIfLineIsBest(unsigned int i_line_polygon, int& best, float& bestDist, Point& prev_point, Point& incommingPerpundicularNormal)
{
    { /// check distance to first point on line (0)
        float dist = vSize2f(polygons[i_line_polygon][0] - prev_point);
        dist += abs(dot(incommingPerpundicularNormal, normal(polygons[i_line_polygon][1] - polygons[i_line_polygon][0], 1000))) * 0.0001f; /// penalize sharp corners
        if (dist < bestDist)
        {
            best = i_line_polygon;
            bestDist = dist;
            polyStart[i_line_polygon] = 0;
        }
    }
    { /// check distance to second point on line (1)
        float dist = vSize2f(polygons[i_line_polygon][1] - prev_point);
        dist += abs(dot(incommingPerpundicularNormal, normal(polygons[i_line_polygon][0] - polygons[i_line_polygon][1], 1000) )) * 0.0001f; /// penalize sharp corners
        if (dist < bestDist)
        {
            best = i_line_polygon;
            bestDist = dist;
            polyStart[i_line_polygon] = 1;
        }
    }
}

}//namespace cura
