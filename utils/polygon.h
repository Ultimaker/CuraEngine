#ifndef UTILS_POLYGON_H
#define UTILS_POLYGON_H

#include <vector>
using std::vector;
#include "clipper/clipper.hpp"

#include "utils/intpoint.h"

class Polygons
{
private:
    ClipperLib::Polygons polygons;
public:
    unsigned int size()
    {
        return polygons.size();
    }
    
    ClipperLib::Polygon& operator[] (int index) //__attribute__((__deprecated__))
    {
        return polygons[index];
    }
    void remove(int index)
    {
        polygons.erase(polygons.begin() + index);
    }
    void clear()
    {
        polygons.clear();
    }
    void add(const ClipperLib::Polygon& poly)
    {
        polygons.push_back(poly);
    }
    void add(const Polygons& other)
    {
        for(unsigned int n=0; n<other.polygons.size(); n++)
            polygons.push_back(other.polygons[n]);
    }
    
    Polygons() {}
    Polygons(const Polygons& other) { polygons = other.polygons; }
    Polygons& operator=(const Polygons& other) { polygons = other.polygons; return *this; }
    Polygons difference(const Polygons& other) const
    {
        Polygons ret;
        ClipperLib::Clipper clipper;
        clipper.AddPolygons(polygons, ClipperLib::ptSubject);
        clipper.AddPolygons(other.polygons, ClipperLib::ptClip);
        clipper.Execute(ClipperLib::ctDifference, ret.polygons);
        return ret;
    }
    Polygons unionPolygons(const Polygons& other) const
    {
        Polygons ret;
        ClipperLib::Clipper clipper;
        clipper.AddPolygons(polygons, ClipperLib::ptSubject);
        clipper.AddPolygons(other.polygons, ClipperLib::ptSubject);
        clipper.Execute(ClipperLib::ctUnion, ret.polygons, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
        return ret;
    }
    Polygons intersection(const Polygons& other) const
    {
        Polygons ret;
        ClipperLib::Clipper clipper;
        clipper.AddPolygons(polygons, ClipperLib::ptSubject);
        clipper.AddPolygons(other.polygons, ClipperLib::ptClip);
        clipper.Execute(ClipperLib::ctIntersection, ret.polygons);
        return ret;
    }
    Polygons offset(int distance) const
    {
        Polygons ret;
        ClipperLib::OffsetPolygons(polygons, ret.polygons, distance, ClipperLib::jtSquare, 2, false);
        return ret;
    }
    vector<Polygons> splitIntoParts(bool unionAll) const
    {
        vector<Polygons> ret;
        ClipperLib::Clipper clipper;
        ClipperLib::PolyTree resultPolyTree;
        clipper.AddPolygons(polygons, ClipperLib::ptSubject);
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
        
        _processPolyTreeNode(&resultPolyTree, ret);
        return ret;
    }
    void _processPolyTreeNode(ClipperLib::PolyNode* node, vector<Polygons>& ret) const
    {
        for(int n=0; n<node->ChildCount(); n++)
        {
            ClipperLib::PolyNode* child = node->Childs[n];
            Polygons polygons;
            polygons.add(child->Contour);
            for(int i=0; i<child->ChildCount(); i++)
            {
                polygons.add(child->Childs[i]->Contour);
                _processPolyTreeNode(child->Childs[i], ret);
            }
            ret.push_back(polygons);
        }
    }
    
    int64_t polygonLength() const
    {
        int64_t length = 0;
        for(unsigned int i=0; i<polygons.size(); i++)
        {
            Point p0 = polygons[i][polygons[i].size()-1];
            for(unsigned int n=0; n<polygons[i].size(); n++)
            {
                Point p1 = polygons[i][n];
                length += vSize(p0 - p1);
                p0 = p1;
            }
        }
        return length;
    }

    void applyMatrix(const PointMatrix& matrix)
    {
        for(unsigned int i=0; i<polygons.size(); i++)
        {
            for(unsigned int j=0; j<polygons[i].size(); j++)
            {
                polygons[i][j] = matrix.apply(polygons[i][j]);
            }
        }
    }
};

INLINE Point centerOfMass(const ClipperLib::Polygon& poly)
{
    double x = 0, y = 0;
    Point p0 = poly[poly.size()-1];
    for(unsigned int n=0; n<poly.size(); n++)
    {
        Point p1 = poly[n];
        double second_factor = (p0.X * p1.Y) - (p1.X * p0.Y);
        
        x += double(p0.X + p1.X) * second_factor;
        y += double(p0.Y + p1.Y) * second_factor;
        p0 = p1;
    }

    double area = Area(poly);
    x = x / 6 / area;
    y = y / 6 / area;

    if (x < 0)
    {
        x = -x;
        y = -y;
    }
    return Point(x, y);
}

/* Axis aligned boundary box */
class AABB
{
public:
    Point min, max;
    
    AABB()
    : min(LLONG_MIN, LLONG_MIN), max(LLONG_MIN, LLONG_MIN)
    {
    }
    AABB(Polygons polys)
    : min(LLONG_MIN, LLONG_MIN), max(LLONG_MIN, LLONG_MIN)
    {
        calculate(polys);
    }
    
    void calculate(Polygons polys)
    {
        min = Point(LLONG_MAX, LLONG_MAX);
        max = Point(LLONG_MIN, LLONG_MIN);
        for(unsigned int i=0; i<polys.size(); i++)
        {
            for(unsigned int j=0; j<polys[i].size(); j++)
            {
                if (min.X > polys[i][j].X) min.X = polys[i][j].X;
                if (min.Y > polys[i][j].Y) min.Y = polys[i][j].Y;
                if (max.X < polys[i][j].X) max.X = polys[i][j].X;
                if (max.Y < polys[i][j].Y) max.Y = polys[i][j].Y;
            }
        }
    }
    
    bool hit(const AABB& other) const
    {
        if (max.X < other.min.X) return false;
        if (min.X > other.max.X) return false;
        if (max.Y < other.min.Y) return false;
        if (min.Y > other.max.Y) return false;
        return true;
    }
};

#endif//UTILS_POLYGON_H
