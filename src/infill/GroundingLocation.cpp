// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/GroundingLocation.h"

#include <cassert>

using namespace curaengine::geometry;

namespace curaengine::infill
{

Point2LL GroundingLocation::p() const
{
    if (tree_node != nullptr)
    {
        return tree_node->getLocation();
    }
    else
    {
        assert(boundary_location);
        return boundary_location->p();
    }
}

} // namespace curaengine::infill
