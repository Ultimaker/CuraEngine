/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_LINE_SEGMENT_H
#define UTILS_LINE_SEGMENT_H

#include "intpoint.h"


namespace cura
{


/*!
 * A simple line piece
 */
class LineSegment
{
public:
    Point from;
    Point to;

    //! construct with 
    LineSegment()
    {}

    LineSegment(const Point from, const Point to)
    : from(from)
    , to(to)
    {}

    Point getVector() const
    {
        return to - from;
    }

    void reverse()
    {
        std::swap(from, to);
    }

    LineSegment reversed() const
    {
        return LineSegment(to, from);
    }

    Point middle() const
    {
        return (from + to) / 2;
    }
};

}//namespace cura
#endif//UTILS_LINE_SEGMENT_H

