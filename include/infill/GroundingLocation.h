// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <memory>
#include <optional>

#include "infill/LightningTreeNode.h"
#include "utils/PolygonUtils.h"

namespace cura
{
using LightningTreeNodeSPtr = std::shared_ptr<LightningTreeNode>;

struct GroundingLocation
{
    LightningTreeNodeSPtr tree_node; //!< not null if the gounding location is on a tree
    std::optional<ClosestPointPolygon> boundary_location; //!< in case the gounding location is on the boundary
    Point2LL p() const;
};
} // namespace cura
