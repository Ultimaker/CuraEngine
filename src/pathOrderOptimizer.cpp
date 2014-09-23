/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "pathOrderOptimizer.h"

namespace cura {

void PathOrderOptimizer::optimize()
{
    std::vector<bool> picked;
    for(unsigned int i=0;i<polygons.size(); i++)
    {
        int best = -1;
        float bestDist = 0xFFFFFFFFFFFFFFFFLL;
        PolygonRef poly = polygons[i];
        for(unsigned int j=0; j<poly.size(); j++)
        {
            float dist = vSize2f(poly[j] - startPoint);
            if (dist < bestDist)
            {
                best = j;
                bestDist = dist;
            }
        }
        polyStart.push_back(best);
        picked.push_back(false);
    }

    Point incommingPerpundicularNormal(0, 0);
    Point p0 = startPoint;
    for(unsigned int n=0; n<polygons.size(); n++)
    {
        int best = -1;
        float bestDist = 0xFFFFFFFFFFFFFFFFLL;
        for(unsigned int i=0;i<polygons.size(); i++)
        {
            if (picked[i] || polygons[i].size() < 1)
                continue;
            if (polygons[i].size() == 2)
            {
                float dist = vSize2f(polygons[i][0] - p0);
                dist += abs(dot(incommingPerpundicularNormal, normal(polygons[i][1] - polygons[i][0], 1000))) * 0.0001f;
                if (dist < bestDist)
                {
                    best = i;
                    bestDist = dist;
                    polyStart[i] = 0;
                }
                dist = vSize2f(polygons[i][1] - p0);
                dist += abs(dot(incommingPerpundicularNormal, normal(polygons[i][0] - polygons[i][1], 1000))) * 0.0001f;
                if (dist < bestDist)
                {
                    best = i;
                    bestDist = dist;
                    polyStart[i] = 1;
                }
            }else{
                float dist = vSize2f(polygons[i][polyStart[i]] - p0);
                if (dist < bestDist)
                {
                    best = i;
                    bestDist = dist;
                }
            }
        }
        if (best > -1)
        {
            if (polygons[best].size() == 2)
            {
                int endIdx = (polyStart[best] + 1) % 2;
                p0 = polygons[best][endIdx];
                incommingPerpundicularNormal = crossZ(normal(polygons[best][endIdx] - polygons[best][polyStart[best]], 1000));
            }else{
                p0 = polygons[best][polyStart[best]];
                incommingPerpundicularNormal = Point(0, 0);
            }
            picked[best] = true;
            polyOrder.push_back(best);
        }
    }
    
    p0 = startPoint;
    for(unsigned int n=0; n<polyOrder.size(); n++)
    {
        int nr = polyOrder[n];
        PolygonRef poly = polygons[nr];
        int best = -1;
        float bestDist = 0xFFFFFFFFFFFFFFFFLL;
        bool orientation = poly.orientation();
        for(unsigned int i=0;i<poly.size(); i++)
        {
            float dist = vSize2f(polygons[nr][i] - p0);
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
            p0 = poly[(best + 1) % 2];
        }else{
            p0 = poly[best];
        }
    }
}

}//namespace cura
