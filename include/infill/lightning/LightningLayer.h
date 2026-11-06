// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <memory>
#include <vector>

#include "infill/lightning/GroundingLocation.h"
#include "infill/lightning/LightningTreeNode.h"
#include "types/geometry/LinesSet.h"
#include "utils/polygon_utils.h"

namespace curaengine::types::geometry
{
template<class Val>
class SparsePointGridInclusive;
}

namespace curaengine::infill::lightning
{

using LightningTreeNodeSPtr = std::shared_ptr<LightningTreeNode>;
using SparseLightningTreeNodeGrid = types::geometry::SparsePointGridInclusive<std::weak_ptr<LightningTreeNode>>;

/*!
 * A layer of the lightning fill.
 *
 * Contains the trees to be printed and propagated to the next layer below.
 */
class LightningLayer
{
public:
    std::vector<LightningTreeNodeSPtr> tree_roots;

    void generateNewTrees(
        const types::geometry::Shape& current_overhang,
        const types::geometry::Shape& current_outlines,
        const types::geometry::LocToLineGrid& outline_locator,
        const types::coord_t supporting_radius,
        const types::coord_t wall_supporting_radius);

    /*! Determine & connect to connection point in tree/outline.
     * \param min_dist_from_boundary_for_tree If the unsupported point is closer to the boundary than this then don't consider connecting it to a tree
     */
    GroundingLocation getBestGroundingLocation(
        const types::geometry::Point2LL& unsupported_location,
        const types::geometry::Shape& current_outlines,
        const types::geometry::LocToLineGrid& outline_locator,
        const types::coord_t supporting_radius,
        const types::coord_t wall_supporting_radius,
        const SparseLightningTreeNodeGrid& tree_node_locator,
        const LightningTreeNodeSPtr& exclude_tree = nullptr);

    /*!
     * \param[out] new_child The new child node introduced
     * \param[out] new_root The new root node if one had been made
     * \return Whether a new root was added
     */
    bool attach(const types::geometry::Point2LL& unsupported_location, const GroundingLocation& ground, LightningTreeNodeSPtr& new_child, LightningTreeNodeSPtr& new_root);

    void reconnectRoots(
        std::vector<LightningTreeNodeSPtr>& to_be_reconnected_tree_roots,
        const types::geometry::Shape& current_outlines,
        const types::geometry::LocToLineGrid& outline_locator,
        const types::coord_t supporting_radius,
        const types::coord_t wall_supporting_radius);

    types::geometry::OpenLinesSet convertToLines(const types::geometry::Shape& limit_to_outline, const types::coord_t line_width) const;

    types::coord_t getWeightedDistance(const types::geometry::Point2LL& boundary_loc, const types::geometry::Point2LL& unsupported_location);

    void fillLocator(SparseLightningTreeNodeGrid& tree_node_locator);
};
} // namespace curaengine::infill::lightning
