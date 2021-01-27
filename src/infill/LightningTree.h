//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef LIGHTNING_TREE_H
#define LIGHTNING_TREE_H

#include "../utils/polygonUtils.h"

#include <functional>
#include <memory>
#include <vector>

namespace cura
{

// NOTE: As written, this struct will only be valid for a single layer, will have to be updated for the next.
// NOTE: Reasons for implementing this with some separate closures:
//       - keep clear deliniation during development
//       - possibility of multiple distance field strategies
class LightningTreeNode : public std::enable_shared_from_this<LightningTreeNode>
{
public:
    // For use with the 'visitBranches' function.
    // Input: Uptree junction point (closer to root), downtree branch point (closer to leaves).
    typedef std::function<void(const Point&, const Point&)> branch_visitor_func_t;

    // For use with the 'visitNodes' function. (Note that the shared_ptr isn't const).
    typedef std::function<void(std::shared_ptr<LightningTreeNode>)> node_visitor_func_t;

    // Workaround for private/protected constructors and 'make_shared': https://stackoverflow.com/a/27832765
    template<typename ...Arg> std::shared_ptr<LightningTreeNode> static create(Arg&&...arg) {
        struct EnableMakeShared : public LightningTreeNode {
            EnableMakeShared(Arg&&...arg) : LightningTreeNode(std::forward<Arg>(arg)...) {}
        };
        return std::make_shared<EnableMakeShared>(std::forward<Arg>(arg)...);
    }

    const Point& getLocation() const;
    void setLocation(const Point& p);

    void addChild(const Point& p);

    void addChild(std::shared_ptr<LightningTreeNode>& new_child);

    // TODO: should be moved outside of this class, because we want to efficiently find pairs of close nodes
    std::shared_ptr<LightningTreeNode> findClosestNode(const Point& x, const coord_t& supporting_radius);

    /*!
     * Propagate this tree to the next layer.
     *
     * Create a copy of this tree,
     * realign it to the new layer boundaries \p next_outlines
     * and reduce (i.e. prune and straighten) it.
     */
    void propagateToNextLayer
    (
        std::vector<std::shared_ptr<LightningTreeNode>>& next_trees,
        const Polygons& next_outlines,
        const coord_t& prune_distance,
        const coord_t& smooth_magnitude
    ) const;

    // NOTE: Depth-first, as currently implemented.
    //       Skips the root (because that has no root itself), but all initial nodes will have the root point anyway.
    void visitBranches(const branch_visitor_func_t& visitor) const;

    // NOTE: Depth-first, as currently implemented.
    //       Also note that, unlike the visitBranches variant, this isn't (...) const!
    void visitNodes(const node_visitor_func_t& visitor);

    coord_t getWeightedDistance(const Point& unsupported_loc, const coord_t& supporting_radius) const;

    bool isRoot() const { return is_root; }

    bool hasOffspring(const std::shared_ptr<LightningTreeNode>& to_be_checked) const;

protected:
    LightningTreeNode() = delete; // Don't allow empty contruction

    // Constructs a node, for insertion into a tree:
    LightningTreeNode(const Point& p);

    // Constructs a root (and initial trunk):
    LightningTreeNode(const Point& a, const Point& b);

    /*!
     * Recursive part of 'findClosestNode'.
     */
    void findClosestNodeHelper(const Point& x, const coord_t supporting_radius, coord_t& closest_distance, std::shared_ptr<LightningTreeNode>& closest_node);

    std::shared_ptr<LightningTreeNode> deepCopy() const; //!< Copy this node and all its children

    /*! Reconnect trees from the layer above to the new outlines of the lower layer.
     * \return Wether or not the root is kept (false is no, true is yes).
     */
    bool realign(const Polygons& outlines, std::vector<std::shared_ptr<LightningTreeNode>>& rerooted_parts, const bool& connected_to_parent = false);

    struct RectilinearJunction
    {
        coord_t total_recti_dist; //!< rectilinear distance along the tree from the last junction above to the junction below
        Point junction_loc; //!< junction location below
    };

    /*! Smoothen the tree to make it a bit more printable, while still supporting the trees above.
     */
    void straighten(const coord_t& magnitude);

    /*! Recursive part of \ref straighten(.)
     * \param junction_above The last seen junction with multiple children above
     * \param accumulated_dist The distance along the tree from the last seen junction to this node
     * \return the total distance along the tree from the last junction above to the first next junction below and the location of the next junction below
     */
    RectilinearJunction straighten(const coord_t& magnitude, Point junction_above, coord_t accumulated_dist);

    /*! Prune the tree from the extremeties (leaf-nodes) until the pruning distance is reached.
     * \return The distance that has been pruned. If less than \p distance, then the whole tree was puned away.
     */
    coord_t prune(const coord_t& distance);

    bool is_root = false;
    Point p;
    std::vector<std::shared_ptr<LightningTreeNode>> children;
};

} // namespace cura

#endif // LIGHTNING_TREE_H
