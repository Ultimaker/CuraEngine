#ifndef POLYGON_OPTIMIZER_H
#define POLYGON_OPTIMIZER_H

void optimizePolygon(ClipperLib::Polygon& poly)
{
    Point p0 = poly[poly.size()-1];
    for(unsigned int i=0;i<poly.size();i++)
    {
        Point p1 = poly[i];
        if (shorterThen(p0 - p1, 10))
        {
            poly.erase(poly.begin() + i);
            i --;
        }else{
            p0 = p1;
        }
    }
}

void optimizePolygons(Polygons& polys)
{
    for(unsigned int n=0;n<polys.size();n++)
    {
        optimizePolygon(polys[n]);
    }
}

#endif//POLYGON_OPTIMIZER_H
