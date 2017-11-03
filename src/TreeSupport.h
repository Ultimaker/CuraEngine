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
};

}

#endif /* TREESUPPORT_H */

