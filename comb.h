/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef COMB_H
#define COMB_H

class Comb
{
private:
    Polygons& boundery;

    int64_t* minX;
    int64_t* maxX;
    unsigned int* minIdx;
    unsigned int* maxIdx;

    PointMatrix matrix;
    Point sp;
    Point ep;

    bool preTest(Point startPoint, Point endPoint)
    {
        return collisionTest(startPoint, endPoint);
    }
    
    bool collisionTest(Point startPoint, Point endPoint)
    {
        Point diff = endPoint - startPoint;

        matrix = PointMatrix(diff);
        sp = matrix.apply(startPoint);
        ep = matrix.apply(endPoint);
        
        for(unsigned int n=0; n<boundery.size(); n++)
        {
            if (boundery[n].size() < 1)
                continue;
            Point p0 = matrix.apply(boundery[n][boundery[n].size()-1]);
            for(unsigned int i=0; i<boundery[n].size(); i++)
            {
                Point p1 = matrix.apply(boundery[n][i]);
                if ((p0.Y > sp.Y && p1.Y < sp.Y) || (p1.Y > sp.Y && p0.Y < sp.Y))
                {
                    int64_t x = p0.X + (p1.X - p0.X) * (sp.Y - p0.Y) / (p1.Y - p0.Y);
                    
                    if (x > sp.X && x < ep.X)
                        return true;
                }
                p0 = p1;
            }
        }
        return false;
    }

    void calcMinMax()
    {
        for(unsigned int n=0; n<boundery.size(); n++)
        {
            minX[n] = LLONG_MAX;
            maxX[n] = LLONG_MIN;
            Point p0 = matrix.apply(boundery[n][boundery[n].size()-1]);
            for(unsigned int i=0; i<boundery[n].size(); i++)
            {
                Point p1 = matrix.apply(boundery[n][i]);
                if ((p0.Y > sp.Y && p1.Y < sp.Y) || (p1.Y > sp.Y && p0.Y < sp.Y))
                {
                    int64_t x = p0.X + (p1.X - p0.X) * (sp.Y - p0.Y) / (p1.Y - p0.Y);
                    
                    if (x >= sp.X && x <= ep.X)
                    {
                        if (x < minX[n]) { minX[n] = x; minIdx[n] = i; }
                        if (x > maxX[n]) { maxX[n] = x; maxIdx[n] = i; }
                    }
                }
                p0 = p1;
            }
        }
    }
    
    unsigned int getPolygonAbove(int64_t x)
    {
        int64_t min = LLONG_MAX;
        unsigned int ret = UINT_MAX;
        for(unsigned int n=0; n<boundery.size(); n++)
        {
            if (minX[n] > x && minX[n] < min)
            {
                min = minX[n];
                ret = n;
            }
        }
        return ret;
    }
    
    Point getBounderyPointWithOffset(unsigned int polygonNr, unsigned int idx)
    {
        Point p0 = boundery[polygonNr][(idx > 0) ? (idx - 1) : (boundery[polygonNr].size() - 1)];
        Point p1 = boundery[polygonNr][idx];
        Point p2 = boundery[polygonNr][(idx < (boundery[polygonNr].size() - 1)) ? (idx + 1) : (0)];
        
        Point off0 = crossZ(normal(p1 - p0, 1000));
        Point off1 = crossZ(normal(p2 - p1, 1000));
        Point n = normal(off0 + off1, 200);
        
        return p1 + n;
    }
    
public:
    Comb(Polygons& _boundery)
    : boundery(_boundery)
    {
        minX = new int64_t[boundery.size()];
        maxX = new int64_t[boundery.size()];
        minIdx = new unsigned int[boundery.size()];
        maxIdx = new unsigned int[boundery.size()];
    }
    
    ~Comb()
    {
        delete[] minX;
        delete[] maxX;
        delete[] minIdx;
        delete[] maxIdx;
    }

    bool checkInside(Point p)
    {
        //Check if we are inside the comb boundary. We do this by tracing from the point towards the negative X direction,
        //  every boundary we cross increments the crossings counter. If we have an even number of crossings then we are not inside the boundary
        int crossings = 0;
        for(unsigned int n=0; n<boundery.size(); n++)
        {
            if (boundery[n].size() < 1)
                continue;
            Point p0 = boundery[n][boundery[n].size()-1];
            for(unsigned int i=0; i<boundery[n].size(); i++)
            {
                Point p1 = boundery[n][i];
                
                if ((p0.Y >= p.Y && p1.Y < p.Y) || (p1.Y > p.Y && p0.Y <= p.Y))
                {
                    int64_t x = p0.X + (p1.X - p0.X) * (p.Y - p0.Y) / (p1.Y - p0.Y);
                    if (x >= p.X)
                        crossings ++;
                }
                p0 = p1;
            }
        }
        if ((crossings % 2) == 0)
            return false;
        return true;
    }

    bool moveInside(Point& p)
    {
        Point ret = p;
        int64_t bestDist = 10000LL * 10000LL;
        for(unsigned int n=0; n<boundery.size(); n++)
        {
            if (boundery[n].size() < 1)
                continue;
            Point p0 = boundery[n][boundery[n].size()-1];
            for(unsigned int i=0; i<boundery[n].size(); i++)
            {
                Point p1 = boundery[n][i];
                
                //Q = A + Normal( B - A ) * ((( B - A ) dot ( P - A )) / VSize( A - B ));
                Point pDiff = p1 - p0;
                int64_t lineLength = vSize(pDiff);
                int64_t distOnLine = dot(pDiff, p - p0) / lineLength;
                if (distOnLine < 10)
                    distOnLine = 10;
                if (distOnLine > lineLength - 10)
                    distOnLine = lineLength - 10;
                Point q = p0 + pDiff * distOnLine / lineLength;
                
                int64_t dist = vSize2(q - p);
                if (dist < bestDist)
                {
                    bestDist = dist;
                    ret = q + crossZ(normal(p1 - p0, 100));
                }
                
                p0 = p1;
            }
        }
        if (bestDist < 10000LL * 10000LL)
        {
            p = ret;
            return true;
        }
        return false;
    }
    
    bool calc(Point startPoint, Point endPoint, vector<Point>& combPoints)
    {
        if (shorterThen(endPoint - startPoint, 1500))
            return true;
        
        bool addEndpoint = false;
        //Check if we are inside the comb boundaries
        if (!checkInside(startPoint))
        {
            if (!moveInside(startPoint))    //If we fail to move the point inside the comb boundary we need to retract.
                return false;
            combPoints.push_back(startPoint);
        }
        if (!checkInside(endPoint))
        {
            if (!moveInside(endPoint))    //If we fail to move the point inside the comb boundary we need to retract.
                return false;
            addEndpoint = true;
        }
        
        //Check if we are crossing any bounderies, and pre-calculate some values.
        if (!preTest(startPoint, endPoint))
        {
            //We're not crossing any boundaries. So skip the comb generation.
            if (!addEndpoint && combPoints.size() == 0) //Only skip if we didn't move the start and end point.
                return true;
        }
        
        //Calculate the minimum and maximum positions where we cross the comb boundary
        calcMinMax();
        
        int64_t x = sp.X;
        vector<Point> pointList;
        //Now walk trough the crossings, for every boundary we cross, find the initial cross point and the exit point. Then add all the points in between
        // to the pointList and continue with the next boundary we will cross, until there are no more boundaries to cross.
        // This gives a path from the start to finish curved around the holes that it encounters.
        while(true)
        {
            unsigned int n = getPolygonAbove(x);
            if (n == UINT_MAX) break;
            
            pointList.push_back(matrix.unapply(Point(minX[n] - 200, sp.Y)));
            if ( (minIdx[n] - maxIdx[n] + boundery[n].size()) % boundery[n].size() > (maxIdx[n] - minIdx[n] + boundery[n].size()) % boundery[n].size())
            {
                for(unsigned int i=minIdx[n]; i != maxIdx[n]; i = (i < boundery[n].size() - 1) ? (i + 1) : (0))
                {
                    pointList.push_back(getBounderyPointWithOffset(n, i));
                }
            }else{
                minIdx[n]--;
                if (minIdx[n] == UINT_MAX) minIdx[n] = boundery[n].size() - 1;
                maxIdx[n]--;
                if (maxIdx[n] == UINT_MAX) maxIdx[n] = boundery[n].size() - 1;
                for(unsigned int i=minIdx[n]; i != maxIdx[n]; i = (i > 0) ? (i - 1) : (boundery[n].size() - 1))
                {
                    pointList.push_back(getBounderyPointWithOffset(n, i));
                }
            }
            pointList.push_back(matrix.unapply(Point(maxX[n] + 200, sp.Y)));
            
            x = maxX[n];
        }
        pointList.push_back(endPoint);
        
        //Optimize the pointList, skip each point we could already reach by not crossing a boundary. This smooths out the path and makes it skip any unneeded corners.
        Point p0 = startPoint;
        for(unsigned int n=1; n<pointList.size(); n++)
        {
            if (collisionTest(p0, pointList[n]))
            {
                if (collisionTest(p0, pointList[n-1]))
                    return false;
                p0 = pointList[n-1];
                combPoints.push_back(p0);
            }
        }
        if (addEndpoint)
            combPoints.push_back(endPoint);
        return true;
    }
};

#endif//COMB_H
