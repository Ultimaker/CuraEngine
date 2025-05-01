// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef POINT2F_H
#define POINT2F_H

namespace cura
{

class Point2F
{
public:
    float x_, y_;

    Point2F()
    {
    }

    Point2F(float x, float y)
        : x_(x)
        , y_(y)
    {
    }
};

} // namespace cura
#endif // POINT2F_H
