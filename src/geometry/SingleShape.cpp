// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/SingleShape.h"

#include "geometry/Polygon.h"

namespace cura
{

bool SingleShape::inside(const Point2LL& p, bool border_result) const
{
    if (size() < 1)
    {
        return false;
    }

    if (! (*this)[0].inside(p, border_result))
    {
        return false;
    }

    for (unsigned int n = 1; n < size(); n++)
    {
        if ((*this)[n].inside(p, border_result))
        {
            return false;
        }
    }
    return true;
}

Polygon& SingleShape::outerPolygon()
{
    return front();
}

const Polygon& SingleShape::outerPolygon() const
{
    return front();
}

} // namespace cura
