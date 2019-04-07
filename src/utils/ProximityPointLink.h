//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PROXIMITY_POINT_LINK_H
#define PROXIMITY_POINT_LINK_H

#include <functional> // hash function object
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <utility> // pair
#include <vector>

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
    coord_t dist; //!< The distance between the two points
    const ProximityPointLinkType type; //!< The type of link; why/how it was created
    void setDist(coord_t dist) const; //!< Set the distance. This disregards cosntness, which is only relevant for the equality check and hash operation.
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
    { // has to be symmetric wrt a and b!
        return std::hash<cura::Point>()(pp.a.p()) + std::hash<cura::Point>()(pp.b.p());
    }
};
}//namespace std


#endif//PROXIMITY_POINT_LINK_H
