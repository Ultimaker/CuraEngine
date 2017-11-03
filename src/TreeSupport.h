//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TREESUPPORT_H
#define TREESUPPORT_H

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
     * \brief Creates points where support contacts the model.
     *
     * A set of points is created for each layer.
     * \param storage The data storage where the mesh data is gotten from.
     * \param contact_points[out] A vector to store the contact points in.
     * \return For each layer, a list of points where the tree should connect
     * with the model.
     */
    void generateContactPoints(const SliceDataStorage& storage, std::vector<std::vector<Point>>& contact_points);
};

}

#endif /* TREESUPPORT_H */

