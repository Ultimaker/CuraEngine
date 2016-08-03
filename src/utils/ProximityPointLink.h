/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef PROXIMITY_POINT_LINK_H
#define PROXIMITY_POINT_LINK_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include <utility> // pair

#include <functional> // hash function object

#include "intpoint.h"
#include "polygon.h"
#include "linearAlg2D.h"
#include "optional.h"

#include "ListPolyIt.h"

namespace cura 
{

/*!
 * Type of ProximityPointLink signifying why/how it was created
 */
enum class ProximityPointLinkType
{
    NORMAL, // Point is close to line segment or to another point
    ENDING, // link where two line segments diverge and have the maximum proximity, i.e. where the overlap will be zero
    ENDING_CORNER, // when an overlap area ends in a point
    SHARP_CORNER // The corner in the polygon is so sharp that it will overlap with itself
};

/*!
 * A class recording the amount of overlap implicitly by recording the distance between two points on two different polygons or one and the same polygon.
 * The order of the two points doesn't matter.
 */
struct ProximityPointLink
{
    const ListPolyIt a; //!< the one point (invalidated after list_polygons have been cleared!)
    const ListPolyIt b; //!< the other point (invalidated after list_polygons have been cleared!)
    const int dist; //!< The distance between the two points
    const ProximityPointLinkType type; //!< The type of link; why/how it was created
    ProximityPointLink(const ListPolyIt a, const ListPolyIt b, int dist, const ProximityPointLinkType type);
    bool operator==(const ProximityPointLink& other) const;
};

}//namespace cura

namespace std
{
template <>
struct hash<cura::ProximityPointLink>
{
    size_t operator()(const cura::ProximityPointLink & pp) const
    {
        return std::hash<cura::Point>()(*pp.a.it) + std::hash<cura::Point>()(*pp.b.it);
    }
};
}//namespace std


#endif//PROXIMITY_POINT_LINK_H
