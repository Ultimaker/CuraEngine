// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <memory>
#include <optional>

#include "infill/lightning/LightningTreeNode.h"
#include "utils/polygon_utils.h"

namespace curaengine::infill::lightning
{
using LightningTreeNodeSPtr = std::shared_ptr<LightningTreeNode>;

struct GroundingLocation
{
    LightningTreeNodeSPtr tree_node; //!< not null if the gounding location is on a tree
    std::optional<types::geometry::ClosestPointPolygon> boundary_location; //!< in case the gounding location is on the boundary
    types::geometry::Point2LL p() const;
};
} // namespace curaengine::infill::lightning
