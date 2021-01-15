//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef RIBBED_SUPPORT_VAULT_GENERATOR_H
#define RIBBED_SUPPORT_VAULT_GENERATOR_H

#include "../utils/polygonUtils.h"

#include <functional>
#include <memory>
#include <vector>
#include <map>

namespace cura 
{
    class SliceMeshStorage;

    // NOTE: As written, this struct will only be valid for a single layer, will have to be updated for the next.
    // NOTE: Reasons for implementing this with some separate closures:
    //       - keep clear deliniation during development
    //       - possibility of multiple distance field strategies
    class RibbedVaultTreeNode : public std::enable_shared_from_this<RibbedVaultTreeNode>
    {
    public:
        friend class std::shared_ptr<RibbedVaultTreeNode>;

        // For use with findClosestNode.
        // Input: Two points. Output: Distance between those points.
        typedef std::function<coord_t(const Point&, const Point&)> point_distance_func_t;

        // For use with the 'visit___' function(s).
        // Input: Uptree junction point (closer to root), downtree branch point (closer to leaves).
        typedef std::function<void(const Point&, const Point&)> visitor_func_t;

        // TODO??: Move/make-settable somehow or merge completely with this class (same as next getter).
        static point_distance_func_t getPointDistanceFunction();

        // Constructs a node, for insertion into a tree:
        RibbedVaultTreeNode(const Point& p);

        // Constructs a root (and initial trunk):
        RibbedVaultTreeNode(const Point& a, const Point& b);

        const Point& getLocation() const;

        void addChild(const Point& p);

        // TODO: should be moved outside of this class, because we want to efficiently find pairs of close nodes
        std::shared_ptr<RibbedVaultTreeNode> findClosestNode(const Point& x, const point_distance_func_t& heuristic);

        /*!
         * Compute the next layer down.
         * 
         * Create a copy of this tree,
         * realign it to the new layer boundaries \p next_outlines
         * and reduce (i.e. prune and smoothen) it.
         */
        void computeNextLayer
        (
            std::vector<std::shared_ptr<RibbedVaultTreeNode>>& next_trees,
            const Polygons& next_outlines,
            const coord_t& prune_distance,
            const float& smooth_magnitude
        ) const;

        // NOTE: Depth-first, as currently implemented.
        //       Skips the root (because that has no root itself), but all initial nodes will have the root point anyway.
        void visitBranches(const visitor_func_t& visitor) const;

    protected:
        RibbedVaultTreeNode() = delete; // Don't allow empty contruction

        /*!
         * What does this function do?!
         */
        void findClosestNodeHelper(const Point& x, const point_distance_func_t& heuristic, coord_t& closest_distance, std::shared_ptr<RibbedVaultTreeNode>& closest_node);

        std::shared_ptr<RibbedVaultTreeNode> deepCopy() const; //!< Copy this node and all its children

        /*! Reconnect trees from the layer above to the new outlines of the lower layer
         */
        void realign(const Polygons& outlines, std::vector<std::shared_ptr<RibbedVaultTreeNode>>& rerooted_parts);

        /*! Smoothen the tree to make it a bit more printable, while still supporting the trees above.
         */
        void smoothen(const float& magnitude);

        //! Prune the tree from the extremeties (leaf-nodes) until the pruning distance is reached.
        bool prune(const coord_t& distance);

        bool is_root = false;
        Point p;
        std::vector<std::shared_ptr<RibbedVaultTreeNode>> children;
    };

    typedef std::vector<std::shared_ptr<RibbedVaultTreeNode>> ribbed_vault_layer_trees_t;

    // NOTE: Currently, the following class is just scaffolding so the entirety can be run during development, while other parts are made in sync.
    //       No particular attention is paid to efficiency & the like. Might be _very_ slow!
    class RibbedVaultDistanceField
    {
    public:
        /*!
         * TODO: explain this function because I don't get it
         * also rename it to something more descriptive of what it does
         */
        void reinit
        (
            const coord_t& radius,
            const Polygons& current_outline,
            const Polygons& current_overhang,
            const std::vector<std::shared_ptr<RibbedVaultTreeNode>>& initial_trees
        );

        bool tryGetNextPoint(Point* p) const;

        /*! update the distance field with a newly added branch
         * TODO: check whether this explanation is correct
         */
        void update(const Point& to_node, const Point& added_leaf);

    protected:
        coord_t supporting_radius; //!< The radius of the area of the layer above supported by a point on a branch of a tree
        Polygons unsupported;
        Polygons supported;
    };

    class RibbedSupportVaultGenerator
    {
    public:
        static bool convertTreesToLines(const ribbed_vault_layer_trees_t& trees, Polygons& result_lines);

        RibbedSupportVaultGenerator(const coord_t& radius, const SliceMeshStorage& mesh);

        void getTreesForLayer(const size_t& layer_id, ribbed_vault_layer_trees_t* p_trees);

    protected:
        //  TODO: Proper, actual, version! ... should probably not be here even (only non static because the radius is used now).
        //        (Make sure random is chosen when difference between epsilon or completely equal).
        Point getClosestOnOutline(const Point& p, const Polygons& pols) const;

        // Necesary, since normally overhangs are only generated for the outside of the model, and only when support is generated.
        void generateInitialInternalOverhangs(const SliceMeshStorage& mesh);

        void generateTrees(const SliceMeshStorage& mesh);

        coord_t radius;
        std::map<coord_t, size_t> layer_id_by_height;
        std::map<size_t, Polygons> overhang_per_layer;
        std::map<size_t, ribbed_vault_layer_trees_t> trees_per_layer;
    };

} // namespace cura

#endif // RIBBED_SUPPORT_VAULT_GENERATOR_H
