#ifndef COMB_H
#define COMB_H

bool needsComb(Point startPoint, Point endPoint, Polygons& boundery)
{
    Point diff = endPoint - startPoint;
    if (shorterThen(diff, 1500))
        return false;
    
    PointMatrix matrix(diff);
    Point sp = matrix.apply(startPoint);
    Point ep = matrix.apply(endPoint);
    
    for(unsigned int n=0; n<boundery.size(); n++)
    {
        Point p0 = matrix.apply(boundery[n][boundery[n].size()-1]);
        for(unsigned int i=0; i<boundery[n].size(); i++)
        {
            Point p1 = matrix.apply(boundery[n][i]);
            if ((p0.Y > sp.Y && p1.Y < sp.Y) || (p0.Y > ep.Y && p1.Y < sp.Y))
            {
                int64_t x = p0.X + (p1.X - p0.X) * (sp.Y - p0.Y) / (p1.Y - p0.Y);
                
                if (x > sp.X && x < ep.X)
                    return true;
                if (x > ep.X && x < sp.X)
                    return true;
            }
            p0 = p1;
        }
    }
    return false;
}

#endif//COMB_H
