/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <map>

#include "pathOrderOptimizer.h"
#include "utils/logoutput.h"

#define INLINE static inline

namespace cura {

static uint32_t hashPoint(const Point& p)
{
    return (p.X / 20000) ^ (p.Y / 20000) << 8;
}

inline void PathOrderOptimizer::checkIfLineIsBest(unsigned int i_line_polygon, int& best, float& bestDist, Point& prev_point, Point& incommingPerpundicularNormal)
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

/**
*
*/
void PathOrderOptimizer::optimize()
{
    std::map<uint32_t, std::vector<unsigned int>> location_to_line_map;
    //std::vector<bool> picked; /// TODO: change to array of constant size
    bool picked [polygons.size()] {}; /// initialized as falses
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
        //picked.push_back(false); /// initialize all picked values as false

        if (poly.size() == 2) /// if polygon is single line
        {
            logError("Shouldn't be lines!");
            location_to_line_map[hashPoint(poly[0])].push_back(i_polygon);
            location_to_line_map[hashPoint(poly[1])].push_back(i_polygon);
        }
    }


    Point incommingPerpundicularNormal(0, 0);
    Point prev_point = startPoint;
    for(unsigned int i_polygon=0 ; i_polygon<polygons.size() ; i_polygon++) /// actual path order optimizer
    {
        int best = -1;
        float bestDist = std::numeric_limits<float>::infinity();

        for(unsigned int i_close_line_polygon : location_to_line_map[hashPoint(prev_point)]) /// check if single-line-polygon is close to last point
        {
            if (picked[i_close_line_polygon] || polygons[i_close_line_polygon].size() < 1)
                continue;


            checkIfLineIsBest(i_close_line_polygon, best, bestDist, prev_point, incommingPerpundicularNormal);

        }

        if (best == -1) /// if no single-line-polygon is close
        {
            for(unsigned int i_polygon=0 ; i_polygon<polygons.size() ; i_polygon++)
            {
                if (picked[i_polygon] || polygons[i_polygon].size() < 1) /// skip single-point-polygons
                    continue;
                if (polygons[i_polygon].size() == 2) /// single-line-polygon (which is not closeby, given by the location_to_line_map)
                {
                    logError("Shouldn't be lines!");

                    checkIfLineIsBest(i_polygon, best, bestDist, prev_point, incommingPerpundicularNormal);

                }else{
                    float dist = vSize2f(polygons[i_polygon][polyStart[i_polygon]] - prev_point);
                    if (dist < bestDist)
                    {
                        best = i_polygon;
                        bestDist = dist;
                    }
                }
            }
        }

        if (best > -1) /// should always be true; we should have been able to identify the best next polygon
        {
            if (polygons[best].size() == 2) /// single-line-polygon
            {
                int endIdx = polyStart[best] * -1 + 1; /// 1 -> 0 , 0 -> 1
                prev_point = polygons[best][endIdx];
                incommingPerpundicularNormal = crossZ(normal(polygons[best][endIdx] - polygons[best][polyStart[best]], 1000));
            }else{
                prev_point = polygons[best][polyStart[best]];
                incommingPerpundicularNormal = Point(0, 0);
            }
            picked[best] = true;
            polyOrder.push_back(best);
        }
        else
            logError("Failed to find next closest part.\n");
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
                bestDist = dist;
            }
        }
        polyStart[nr] = best;
        if (poly.size() <= 2)
        {
            prev_point = poly[best *-1 + 1]; /// 1 -> 0 , 0 -> 1
        }else{
            prev_point = poly[best];
        }
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

/**
*
*/
void LineOrderOptimizer::optimize()
{
    std::map<uint32_t, std::vector<unsigned int>> location_to_line_map;
    //std::vector<bool> picked; /// TODO: change to array of constant size
    bool picked [polygons.size()] {}; /// initialized as falses
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
        //picked.push_back(false); /// initialize all picked values as false

        if (poly.size() == 2) /// if polygon is single line
        {
            location_to_line_map[hashPoint(poly[0])].push_back(i_polygon);
            location_to_line_map[hashPoint(poly[1])].push_back(i_polygon);
        } else
            logError("Should only be lines!");

    }


    Point incommingPerpundicularNormal(0, 0);
    Point prev_point = startPoint;
    for(unsigned int i_polygon=0 ; i_polygon<polygons.size() ; i_polygon++) /// actual path order optimizer
    {
        int best = -1;
        float bestDist = std::numeric_limits<float>::infinity();

        for(unsigned int i_close_line_polygon : location_to_line_map[hashPoint(prev_point)]) /// check if single-line-polygon is close to last point
        {
            if (picked[i_close_line_polygon] || polygons[i_close_line_polygon].size() < 1)
                continue;


            checkIfLineIsBest(i_close_line_polygon, best, bestDist, prev_point, incommingPerpundicularNormal);

        }

        if (best == -1) /// if no single-line-polygon is close
        {
            logError("Should only be lines!");
           for(unsigned int i_polygon=0 ; i_polygon<polygons.size() ; i_polygon++)
            {
                if (picked[i_polygon] || polygons[i_polygon].size() < 1) /// skip single-point-polygons
                    continue;
                if (polygons[i_polygon].size() == 2) /// single-line-polygon (which is not closeby, given by the location_to_line_map)
                {
                    checkIfLineIsBest(i_polygon, best, bestDist, prev_point, incommingPerpundicularNormal);

                }else{
                    float dist = vSize2f(polygons[i_polygon][polyStart[i_polygon]] - prev_point);
                    if (dist < bestDist)
                    {
                        best = i_polygon;
                        bestDist = dist;
                    }
                }
            }
        }

        if (best > -1) /// should always be true; we should have been able to identify the best next polygon
        {
            if (polygons[best].size() == 2) /// single-line-polygon
            {
                int endIdx = polyStart[best] * -1 + 1; /// 1 -> 0 , 0 -> 1
                prev_point = polygons[best][endIdx];
                incommingPerpundicularNormal = crossZ(normal(polygons[best][endIdx] - polygons[best][polyStart[best]], 1000));
            }else{
                prev_point = polygons[best][polyStart[best]];
                incommingPerpundicularNormal = Point(0, 0);
            }
            picked[best] = true;
            polyOrder.push_back(best);
        }
        else
            logError("Failed to find next closest part.\n");
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
                bestDist = dist;
            }
        }
        polyStart[nr] = best;
        if (poly.size() <= 2)
        {
            prev_point = poly[best *-1 + 1]; /// 1 -> 0 , 0 -> 1
        }else{
            prev_point = poly[best];
        }
    }
}

}//namespace cura
