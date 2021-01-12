//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef RIBBED_SUPPORT_VAULT_GENERATOR_H
#define RIBBED_SUPPORT_VAULT_GENERATOR_H

#include "../sliceDataStorage.h"

#include <functional>
#include <memory>
#include <vector>

namespace cura 
{
    // NOTE: As written, this struct will only be valid for a single layer, will have to be updated for the next.
    // NOTE: Reasons for implementing this with some separate closures:
    //       - keep clear deliniation during development
    //       - possibility of multiple distance field strategies
    class RibbedVaultTree : public std::enable_shared_from_this<RibbedVaultTree>
    {
    public:
        friend class std::shared_ptr<RibbedVaultTree>;

        // For use with findClosestNode.
        // Input: Two points. Output: Distance between those points.
        typedef std::function<coord_t(const Point&, const Point&)> point_distance_func_t;

        // For use with initNextLayer.
        // Input: Smoothing magnitude, branch-chain. Expected to alter the input branch-chain.
        typedef std::function<void(const float&, std::vector<Point>&)> sequence_smooth_func_t;

        // For use with the 'visit___' function(s).
        // Input: Uptree junction point (closer to root), downtree branch point (closer to leaves).
        typedef std::function<void(const Point&, const Point&)> visitor_func_t;

        // TODO??: Move/make-settable somehow or merge completely with this class (same as next getter).
        static point_distance_func_t getPointDistanceFunction();
        static sequence_smooth_func_t getSequenceSmoothFunction();

        // Constructs a node, for insertion into a tree:
        RibbedVaultTree(const Point& p);

        // Constructs a root (and initial trunk):
        RibbedVaultTree(const Point& a, const Point& b);

        const Point& getNode() const;

        void addNode(const Point& p);

        std::shared_ptr<RibbedVaultTree> findClosestNode(const Point& x, const point_distance_func_t& heuristic);

        void initNextLayer
        (
            std::vector<std::shared_ptr<RibbedVaultTree>>& next_trees,
            const Polygons& next_outlines,
            const coord_t& prune_distance,
            const float& smooth_magnitude,
            const sequence_smooth_func_t& smooth_heuristic
        ) const;

        // NOTE: Depth-first, as currently implemented.
        //       Skips the root (because that has no root itself), but all initial nodes will have the root point anyway.
        void visitBranches(const visitor_func_t& visitor) const;

    protected:
        RibbedVaultTree() = delete;

        void findClosestNodeHelper(const Point& x, const point_distance_func_t& heuristic, coord_t& closest_distance, std::shared_ptr<RibbedVaultTree>& closest_node);

        std::shared_ptr<RibbedVaultTree> deepCopy() const;

        void truncate(const Polygons& outlines, std::vector<std::shared_ptr<RibbedVaultTree>>& rerooted_parts);

        void smooth(const float& magnitude, const sequence_smooth_func_t& heuristic);

        // Prune the tree from the extremeties (leaf-nodes) until the pruning distance is reached.
        bool prune(const coord_t& distance);

        bool is_root = false;
        Point p;
        std::vector<std::shared_ptr<RibbedVaultTree>> nodes;
    };

    // NOTE: Reasons for interface:
    //       - closure would be possible (like point_distance_func_t for example), but required state would be cumbersome to handle
    //       - keep clear deliniation during development
    //       - possibility of multiple distance field strategies
    class IRibbedVaultDistanceField
    {
    public:
        virtual void reinit
        (
            const coord_t& radius,
            const Polygons& current_outline,
            const Polygons& current_overhang,
            const std::vector<std::shared_ptr<RibbedVaultTree>>& initial_trees
        ) = 0;
        virtual bool tryGetNextPoint(Point* p) const = 0;
        virtual void update(const Point& to_node, const Point& added_leaf) = 0;

    protected:
        IRibbedVaultDistanceField() = default;
        virtual ~IRibbedVaultDistanceField() = default;
    };

    // NOTE: The following class is just scaffolding so the entirety can be run during development, while other parts are made in sync.
    //       No particular attention is paid to efficiency & the like. Might be _very_ slow!
    class SillyRibbedVaultDistanceField : public IRibbedVaultDistanceField
    {
    public:
        virtual void reinit
        (
            const coord_t& radius,
            const Polygons& current_outline,
            const Polygons& current_overhang,
            const std::vector<std::shared_ptr<RibbedVaultTree>>& initial_trees
        );

        virtual bool tryGetNextPoint(Point* p) const;

        virtual void update(const Point& to_node, const Point& added_leaf);

    protected:
        coord_t r;
        Polygons unsupported;
        Polygons supported;
    };

    class RibbedSupportVaultGenerator
    {
    public:
        RibbedSupportVaultGenerator(const coord_t& radius, const SliceMeshStorage& mesh);

        // Returns 'added someting'.
        bool addLayerToResult(const coord_t& z, Polygons& result_lines);

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
        std::map<size_t, std::vector<std::shared_ptr<RibbedVaultTree>>> trees_per_layer;
    };

} // namespace cura

#endif // RIBBED_SUPPORT_VAULT_GENERATOR_H
