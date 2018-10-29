//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TREESUPPORT_H
#define TREESUPPORT_H

#include <forward_list>
#include <unordered_set>

#include <memory>
#include <vector>
#include <utility>
#include <unordered_map>

#include "sliceDataStorage.h"


namespace cura
{
/*!
 * \brief Lazily generates tree guidance volumes.
 *
 * NOTE: This class is not currently thread-safe and should not be accessed in OpenMP blocks
 */
class ModelVolumes
{
public:
    ModelVolumes() = default;
    ModelVolumes(const SliceDataStorage& storage, Polygons machine_border, coord_t xy_distance, coord_t max_move,
                 coord_t radius_sample_resolution);

    ModelVolumes(ModelVolumes&&) = default;
    ModelVolumes& operator=(ModelVolumes&&) = default;

    ModelVolumes(const ModelVolumes&) = delete;
    ModelVolumes& operator=(const ModelVolumes&) = delete;

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model.
     *
     * \param radius The radius of the node of interest
     * \param layer The layer of interest
     * \return Polygons object
     */
    const Polygons& collision(coord_t radius, int layer) const;

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches
     * in order to reach the build plate.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model or be unable to reach the build platform.
     *
     * The input collision areas are inset by the maximum move distance and
     * propagated upwards.
     *
     * \param radius The radius of the node of interest
     * \param layer The layer of interest
     * \return Polygons object
     */
    const Polygons& avoidance(coord_t radius, int layer) const;

    /*!
     * \brief Generates the area of a given layer that must be avoided if the
     * branches wish to go towards the model
     *
     * The area represents the areas that do not collide with the model but
     * are unable to reach the build platform
     *
     * \param radius The radius of the node of interest
     * \param layer The layer of interest
     * \return Polygons object
     */
    const Polygons& internal_model(coord_t radius, int layer) const;

private:
    using RadiusLayerPair = std::pair<coord_t, int>;

    coord_t roundRadius(coord_t radius) const;
    const Polygons& calculateCollision(const RadiusLayerPair& key) const;
    const Polygons& calculateAvoidance(const RadiusLayerPair& key) const;
    const Polygons& calculateInternalModel(const RadiusLayerPair& key) const;

    Polygons machine_border_;
    coord_t xy_distance_;
    coord_t max_move_;
    coord_t radius_sample_resolution_;
    std::vector<Polygons> layer_outlines_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> collision_cache_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> avoidance_cache_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> internal_model_cache_;
};

/*!
 * \brief Generates a tree structure to support your models.
 */
class TreeSupport
{
public:
    /*!
     * \brief Creates an instance of the tree support generator.
     *
     * \param storage The data storage to get global settings from.
     */
    TreeSupport(const SliceDataStorage& storage);

    /*!
     * \brief Create the areas that need support.
     *
     * These areas are stored inside the given SliceDataStorage object.
     * \param storage The data storage where the mesh data is gotten from and
     * where the resulting support areas are stored.
     */
    void generateSupportAreas(SliceDataStorage& storage);

    /*!
     * \brief Represents the metadata of a node in the tree.
     */
    struct Node
    {
        static constexpr Node* NO_PARENT = nullptr;

        Node()
         : distance_to_top(0)
         , position(Point(0, 0))
         , skin_direction(false)
         , support_roof_layers_below(0)
         , to_buildplate(true)
         , parent(nullptr)
        {}

        Node(const Point position, const size_t distance_to_top, const bool skin_direction, const int support_roof_layers_below, const bool to_buildplate, Node* const parent)
         : distance_to_top(distance_to_top)
         , position(position)
         , skin_direction(skin_direction)
         , support_roof_layers_below(support_roof_layers_below)
         , to_buildplate(to_buildplate)
         , parent(parent)
        {}

#ifdef DEBUG // Clear the delete node's data so if there's invalid access after, we may get a clue by inspecting that node.
        ~Node()
        {
            parent = nullptr;
            merged_neighbours.clear();
        }
#endif // DEBUG

        /*!
         * \brief The number of layers to go to the top of this branch.
         */
        mutable size_t distance_to_top;

        /*!
         * \brief The position of this node on the layer.
         */
        Point position;

        /*!
         * \brief The direction of the skin lines above the tip of the branch.
         *
         * This determines in which direction we should reduce the width of the
         * branch.
         */
        mutable bool skin_direction;

        /*!
         * \brief The number of support roof layers below this one.
         *
         * When a contact point is created, it is determined whether the mesh
         * needs to be supported with support roof or not, since that is a
         * per-mesh setting. This is stored in this variable in order to track
         * how far we need to extend that support roof downwards.
         */
        mutable int support_roof_layers_below;

        /*!
         * \brief Whether to try to go towards the build plate.
         *
         * If the node is inside the collision areas, it has no choice but to go
         * towards the model. If it is not inside the collision areas, it must
         * go towards the build plate to prevent a scar on the surface.
         */
        mutable bool to_buildplate;

        /*!
         * \brief The originating node for this one, one layer higher.
         *
         * In order to prune branches that can't have any support (because they
         * can't be on the model and the path to the buildplate isn't clear),
         * the entire branch needs to be known.
         */
        Node *parent;

        /*!
        * \brief All neighbours (on the same layer) that where merged into this node.
        *
        * In order to prune branches that can't have any support (because they
        * can't be on the model and the path to the buildplate isn't clear),
        * the entire branch needs to be known.
        */
        mutable std::forward_list<Node*> merged_neighbours;

        bool operator==(const Node& other) const
        {
            return position == other.position;
        }
    };

private:
    /*!
     * \brief Generator for model collision, avoidance and internal guide volumes
     *
     * Lazily computes volumes as needed.
     *  Note: This class is NOT currently thread-safe and should not be accessed in OpenMP blocks
     */
    ModelVolumes volumes_;

    /*!
     * \brief Draws circles around each node of the tree into the final support.
     *
     * This also handles the areas that have to become support roof, support
     * bottom, the Z distances, etc.
     *
     * \param storage[in, out] The settings storage to get settings from and to
     * save the resulting support polygons to.
     * \param contact_nodes The nodes to draw as support.
     */
    void drawCircles(SliceDataStorage& storage, const std::vector<std::unordered_set<Node*>>& contact_nodes);

    /*!
     * \brief Drops down the nodes of the tree support towards the build plate.
     *
     * This is where the cleverness of tree support comes in: The nodes stay on
     * their 2D layers but on the next layer they are slightly shifted. This
     * causes them to move towards each other as they are copied to lower layers
     * which ultimately results in a 3D tree.
     *
     * \param contact_nodes[in, out] The nodes in the space that need to be
     * dropped down. The nodes are dropped to lower layers inside the same
     * vector of layers.
     */
    void dropNodes(std::vector<std::unordered_set<Node*>>& contact_nodes);

    /*!
     * \brief Creates points where support contacts the model.
     *
     * A set of points is created for each layer.
     * \param mesh The mesh to get the overhang areas to support of.
     * \param contact_nodes[out] A vector of mappings from contact points to
     * their tree nodes.
     * \param collision_areas For every layer, the areas where a generated
     * contact point would immediately collide with the model due to the X/Y
     * distance.
     * \return For each layer, a list of points where the tree should connect
     * with the model.
     */
    void generateContactPoints(const SliceMeshStorage& mesh, std::vector<std::unordered_set<Node*>>& contact_nodes);

    /*!
     * \brief Add a node to the next layer.
     *
     * If a node is already at that position in the layer, the nodes are merged.
     */
    void insertDroppedNode(std::unordered_set<Node*>& nodes_layer, Node* node);
};

}

namespace std
{
    template<> struct hash<cura::TreeSupport::Node>
    {
        size_t operator()(const cura::TreeSupport::Node& node) const
        {
            return hash<cura::Point>()(node.position);
        }
    };
}

#endif /* TREESUPPORT_H */
