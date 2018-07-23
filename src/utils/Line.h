#ifndef CURAENGINE_UTILS_LINE_H
#define CURAENGINE_UTILS_LINE_H

#include "IntPoint.h"


namespace cura
{

struct Line
{
    Point p0;
    Point p1;

    Line()
    {}

    Line(const Point& p0, const Point& p1)
        : p0(p0)
        , p1(p1)
    {}

    bool operator==(const Line& l) const
    {
        return ((p0 == l.p0 && p1 == l.p1) || (p0 == l.p1 && p1 == l.p0));
    }
};


struct Polyline
{
    std::vector<Line> lines;
    bool              is_closed;

    Polyline(bool is_closed = false)
        : is_closed(is_closed)
    {}
};


}

#endif // CURAENGINE_UTILS_LINE_H
