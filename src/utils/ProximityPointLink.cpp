/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "ProximityPointLink.h"

namespace cura 
{

ProximityPointLink::ProximityPointLink(const ListPolyIt a, const ListPolyIt b, int dist, const ProximityPointLinkType type)
: a(a)
, b(b)
, dist(dist)
, type(type)
{
}

bool ProximityPointLink::operator==(const ProximityPointLink& other) const
{
    return (a == other.a && b == other.b) || (a == other.b && b == other.a);
}

void ProximityPointLink::setDist(coord_t distance) const
{
    ProximityPointLink& thiss = *const_cast<ProximityPointLink*>(this);
    thiss.dist = distance;
}

}//namespace cura 
