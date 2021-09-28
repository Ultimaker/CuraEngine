//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef LIGHTNING_TREE_NODE_H
#define LIGHTNING_TREE_NODE_H

#include <functional>
#include <memory>
#include <vector>

#include "../utils/polygonUtils.h"
#include "../utils/polygon.h"
#include "../utils/LazyInitialization.h"

namespace cura
{

class LightningTreeNode;

using LightningTreeNodeSPtr = std::shared_ptr<LightningTreeNode>;

// NOTE: As written, this struct will only be valid for a single layer, will have to be updated for the next.
// NOTE: Reasons for implementing this with some separate closures:
//       - keep clear deliniation during development
//       - possibility of multiple distance field strategies
class LightningTreeNode : public std::enable_shared_from_this<LightningTreeNode>
{
public:
    // Workaround for private/protected constructors and 'make_shared': https://stackoverflow.com/a/27832765
    template<typename ...Arg> LightningTreeNodeSPtr static create(Arg&&...arg) {
        struct EnableMakeShared : public LightningTreeNode {
            EnableMakeShared(Arg&&...arg) : LightningTreeNode(std::forward<Arg>(arg)...) {}
        };
        return std::make_shared<EnableMakeShared>(std::forward<Arg>(arg)...);
    }

    const Point& getLocation() const;
    void setLocation(const Point& p);

    LightningTreeNodeSPtr addChild(const Point& p);
    LightningTreeNodeSPtr addChild(LightningTreeNodeSPtr& new_child);

    /*!
     * Propagate this tree to the next layer.
     *
     * Create a copy of this tree,
     * realign it to the new layer boundaries \p next_outlines
     * and reduce (i.e. prune and straighten) it.
     */
    void propagateToNextLayer
    (
        std::vector<LightningTreeNodeSPtr>& next_trees,
        const Polygons& next_outlines,
        const coord_t& prune_distance,
        const coord_t& smooth_magnitude
    ) const;

    /*! NOTE: Depth-first, as currently implemented.
     *        Skips the root (because that has no root itself), but all initial nodes will have the root point anyway.
     * \param visitor Input: Uptree junction point (closer to root), downtree branch point (closer to leaves).
     */
    void visitBranches(const std::function<void(const Point&, const Point&)>& visitor) const;

    // NOTE: Depth-first, as currently implemented.
    //       Also note that, unlike the visitBranches variant, this isn't (...) const!
    void visitNodes(const std::function<void(LightningTreeNodeSPtr)>& visitor);

    coord_t getWeightedDistance(const Point& unsupported_location, const coord_t& supporting_radius) const;

    bool isRoot() const { return is_root; }

    bool hasOffspring(const LightningTreeNodeSPtr& to_be_checked) const;
protected:
    LightningTreeNode() = delete; // Don't allow empty contruction

    // Constructs a node, either for insertion into a tree, or as root:
    LightningTreeNode(const Point& p);

    LightningTreeNodeSPtr deepCopy() const; //!< Copy this node and all its children

    /*! Reconnect trees from the layer above to the new outlines of the lower layer.
     * \return Wether or not the root is kept (false is no, true is yes).
     */
    bool realign(const Polygons& outlines, std::vector<LightningTreeNodeSPtr>& rerooted_parts, const bool& connected_to_parent = false);

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
    RectilinearJunction straighten(const coord_t& magnitude, const Point& junction_above, const coord_t accumulated_dist);

    /*! Prune the tree from the extremeties (leaf-nodes) until the pruning distance is reached.
     * \return The distance that has been pruned. If less than \p distance, then the whole tree was puned away.
     */
    coord_t prune(const coord_t& distance);
public:
    /*!
     * Convert the tree into polylines
     * 
     * At each junction one line is chosen at random to continue
     * 
     * The lines start at a leaf and end in a junction
     * 
     * \param output all branches in this tree connected into polylines
     */
    void convertToPolylines(Polygons& output, const coord_t line_width) const;

protected:
    /*!
     * Convert the tree into polylines
     * 
     * At each junction one line is chosen at random to continue
     * 
     * The lines start at a leaf and end in a junction
     * 
     * \param long_line a reference to a polyline in \p output which to continue building on in the recursion
     * \param output all branches in this tree connected into polylines
     */
    void convertToPolylines(size_t long_line_idx, Polygons& output) const;

    void removeJunctionOverlap(Polygons& polylines, const coord_t line_width) const;

    bool is_root;
    Point p;
    std::weak_ptr<LightningTreeNode> parent;
    std::vector<LightningTreeNodeSPtr> children;
};

} // namespace cura

#endif // LIGHTNING_TREE_NODE_H
