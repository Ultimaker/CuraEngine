// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/lightning/GroundingLocation.h"

#include <cassert>

namespace curaengine::infill::lightning
{
using namespace types::geometry;

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

} // namespace curaengine::infill::lightning
