#ifndef PATHOPTIMIZER_H
#define PATHOPTIMIZER_H

#include <stdint.h>

float distanceSquared(const ClipperLib::IntPoint& p0, const ClipperLib::IntPoint& p1)
{
    float dx = p0.X - p1.X;
    float dy = p0.Y - p1.Y;
    return dx * dx + dy * dy;
}

class PathOptimizer
{
public:
    ClipperLib::IntPoint startPoint;
    std::vector<ClipperLib::Polygon*> polygons;
    std::vector<int> polyStart;
    std::vector<int> polyOrder;

    PathOptimizer(ClipperLib::IntPoint startPoint)
    {
        this->startPoint = startPoint;
    }

    void addPolygon(ClipperLib::Polygon& polygon)
    {
        this->polygons.push_back(&polygon);
    }
    
    void addPolygons(ClipperLib::Polygons& polygons)
    {
        for(unsigned int i=0;i<polygons.size(); i++)
            this->polygons.push_back(&polygons[i]);
    }
    
    void optimize()
    {
        std::vector<bool> picked;
        for(unsigned int i=0;i<polygons.size(); i++)
        {
            int best = -1;
            float bestDist = 0xFFFFFFFFFFFFFFFFLL;
            ClipperLib::Polygon* poly = polygons[i];
            for(unsigned int j=0; j<poly->size(); j++)
            {
                float dist = distanceSquared((*poly)[j], startPoint);
                if (dist < bestDist)
                {
                    best = j;
                    bestDist = dist;
                }
            }
            polyStart.push_back(best);
            picked.push_back(false);
        }

        ClipperLib::IntPoint p0 = startPoint;
        for(unsigned int n=0; n<polygons.size(); n++)
        {
            int best = -1;
            float bestDist = 0xFFFFFFFFFFFFFFFFLL;
            for(unsigned int i=0;i<polygons.size(); i++)
            {
                if (picked[i])
                    continue;
                float dist = distanceSquared((*polygons[i])[polyStart[i]], p0);
                if (dist < bestDist)
                {
                    best = i;
                    bestDist = dist;
                }
            }
            p0 = (*polygons[best])[polyStart[best]];
            picked[best] = true;
            polyOrder.push_back(best);
        }
        
        p0 = startPoint;
        for(unsigned int n=0; n<polygons.size(); n++)
        {
            int nr = polyOrder[n];
            int best = -1;
            float bestDist = 0xFFFFFFFFFFFFFFFFLL;
            for(unsigned int i=0;i<polygons[nr]->size(); i++)
            {
                float dist = distanceSquared((*polygons[nr])[i], p0);
                if (dist < bestDist)
                {
                    best = i;
                    bestDist = dist;
                }
            }
            polyStart[nr] = best;
            p0 = (*polygons[nr])[best];
        }
    }
};

#endif//PATHOPTIMIZER_H
