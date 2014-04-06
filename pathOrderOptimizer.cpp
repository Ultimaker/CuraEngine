/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "pathOrderOptimizer.h"

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
                if (dist < bestDist)
                {
                    best = i;
                    bestDist = dist;
                    polyStart[i] = 0;
                }
                dist = vSize2f(polygons[i][1] - p0);
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
                p0 = polygons[best][(polyStart[best] + 1) % 2];
            }else{
                p0 = polygons[best][polyStart[best]];
            }
            picked[best] = true;
            polyOrder.push_back(best);
        }
    }
    
    p0 = startPoint;
    for(unsigned int n=0; n<polyOrder.size(); n++)
    {
        int nr = polyOrder[n];
        int best = -1;
        float bestDist = 0xFFFFFFFFFFFFFFFFLL;
        for(unsigned int i=0;i<polygons[nr].size(); i++)
        {
            float dist = vSize2f(polygons[nr][i] - p0);
            if (dist < bestDist)
            {
                best = i;
                bestDist = dist;
            }
        }
        polyStart[nr] = best;
        if (polygons[nr].size() <= 2)
        {
            p0 = polygons[nr][(best + 1) % 2];
        }else{
            p0 = polygons[nr][best];
        }
    }
}
