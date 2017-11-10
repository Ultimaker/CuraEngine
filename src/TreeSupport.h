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
    };

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

