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

private:
    /*!
     * \brief Represents the metadata of a node in the tree.
     */
    struct Node
    {
        Node()
        {
            distance_to_top = 0;
            skin_direction = false;
            support_roof_layers_below = 0;
            to_buildplate = true;
        }

        /*!
         * \brief The number of layers to go to the top of this branch.
         */
        size_t distance_to_top;

        /*!
         * \brief The direction of the skin lines above the tip of the branch.
         *
         * This determines in which direction we should reduce the width of the
         * branch.
         */
        bool skin_direction;

        /*!
         * \brief The number of support roof layers below this one.
         *
         * When a contact point is created, it is determined whether the mesh
         * needs to be supported with support roof or not, since that is a
         * per-mesh setting. This is stored in this variable in order to track
         * how far we need to extend that support roof downwards.
         */
        int support_roof_layers_below;

        /*!
         * \brief Whether to try to go towards the build plate.
         *
         * If the node is inside the collision areas, it has no choice but to go
         * towards the model. If it is not inside the collision areas, it must
         * go towards the build plate to prevent a scar on the surface.
         */
        bool to_buildplate;
    };

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
     * \brief Creates points where support contacts the model.
     *
     * A set of points is created for each layer.
     * \param mesh The mesh to get the overhang areas to support of.
     * \param contact_points[out] A vector of sets to store the contact points
     * in.
     * \param contact_nodes[out] A vector of mappings from contact points to
     * their tree nodes.
     * \return For each layer, a list of points where the tree should connect
     * with the model.
     */
    void generateContactPoints(const SliceMeshStorage& mesh, std::vector<std::unordered_set<Point>>& contact_points, std::vector<std::unordered_map<Point, Node>>& contact_nodes);
};

}

#endif /* TREESUPPORT_H */

