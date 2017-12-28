//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TREESUPPORT_H
#define TREESUPPORT_H

#include <unordered_set>

#include "sliceDataStorage.h"

namespace cura
{

/*!
 * \brief Generates a tree structure to support your models.
 */
class TreeSupport
{
public:
    /*!
     * \brief Creates an instance of the tree support generator.
     */
    TreeSupport();

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
        Node()
        {
            position = Point(0, 0);
            distance_to_top = 0;
            skin_direction = false;
            support_roof_layers_below = 0;
            to_buildplate = true;
        }

        Node(const Point position, const size_t distance_to_top, const bool skin_direction, const int support_roof_layers_below, const bool to_buildplate)
         : distance_to_top(distance_to_top), position(position), skin_direction(skin_direction), support_roof_layers_below(support_roof_layers_below), to_buildplate(to_buildplate) {}

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

        bool operator==(const Node& other) const
        {
            return position == other.position;
        }
    };

private:
    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches.
     *
     * The result is a vector of 3D volumes that have to be avoided, where each
     * volume consists of a number of layers where the branch would collide with
     * the model.
     * There will be a volume for each sample of branch radius. The radii of the
     * branches are unknown at this point (there will be several radii at any
     * given layer too), so a collision area is generated for every possible
     * radius.
     *
     * \param storage The settings storage to get settings from.
     * \param model_collision[out] A vector to fill with the output collision
     * areas.
     */
    void collisionAreas(const SliceDataStorage& storage, std::vector<std::vector<Polygons>>& model_collision);

    /*!
     * \brief Draws circles around each node of the tree into the final support.
     *
     * This also handles the areas that have to become support roof, support
     * bottom, the Z distances, etc.
     *
     * \param storage[in, out] The settings storage to get settings from and to
     * save the resulting support polygons to.
     * \param contact_nodes The nodes to draw as support.
     * \param model_collision The model infill with the X/Y distance already
     * subtracted.
     */
    void drawCircles(SliceDataStorage& storage, const std::vector<std::unordered_set<Node>>& contact_nodes, const std::vector<std::vector<Polygons>>& model_collision);

    /*!
     * \brief Drops down the nodes of the tree support towards the build plate.
     *
     * This is where the cleverness of tree support comes in: The nodes stay on
     * their 2D layers but on the next layer they are slightly shifted. This
     * causes them to move towards each other as they are copied to lower layers
     * which ultimately results in a 3D tree.
     *
     * \param storage The settings storage to get settings from.
     * \param contact_nodes[in, out] The nodes in the space that need to be
     * dropped down. The nodes are dropped to lower layers inside the same
     * vector of layers.
     * \param model_collision For each sample of radius, a list of layers with
     * the polygons of the collision areas of the model. Any node in there will
     * collide with the model.
     * \param model_avoidance For each sample of radius, a list of layers with
     * the polygons that must be avoided if the branches wish to go towards the
     * build plate.
     * \param model_internal_guide For each sample of radius, a list of layers
     * with the polygons that must be avoided if the branches wish to go towards
     * the model.
     */
    void dropNodes(const SliceDataStorage& storage, std::vector<std::unordered_set<Node>>& contact_nodes, const std::vector<std::vector<Polygons>>& model_collision, const std::vector<std::vector<Polygons>>& model_avoidance, const std::vector<std::vector<Polygons>>& model_internal_guide);

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
    void generateContactPoints(const SliceMeshStorage& mesh, std::vector<std::unordered_set<Node>>& contact_nodes, const std::vector<Polygons>& collision_areas);

    /*!
     * \brief Add a node to the next layer.
     *
     * If a node is already at that position in the layer, the nodes are merged.
     */
    void insertDroppedNode(std::unordered_set<Node>& nodes_layer, Node& node);

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches
     * in order to reach the build plate.
     *
     * The result is a vector of 3D volumes that have to be avoided, where each
     * volume consists of a number of layers where the branch would collide with
     * the model.
     * There will be a volume for each sample of branch radius. The radii of the
     * branches are unknown at this point (there will be several radii at any
     * given layer too), so a collision area is generated for every possible
     * radius.
     *
     * The input collision areas are inset by the maximum move distance and
     * propagated upwards. This generates volumes so that the branches can
     * predict in time when they need to be moving away in order to avoid
     * hitting the model.
     * \param storage The settings storage to get settings from.
     * \param model_collision The collision areas that may not be hit by the
     * model.
     * \param model_avoidance[out] A vector to fill with the output avoidance
     * areas.
     */
    void propagateCollisionAreas(const SliceDataStorage& storage, const std::vector<std::vector<Polygons>>& model_collision, std::vector<std::vector<Polygons>>& model_avoidance);
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

