// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <vector>

#include "geometry/Point3LL.h"

namespace curaengine
{

using GridPoint3 = Point3LL;

/*!
 * Class for holding the relative positiongs wrt a reference cell on which to perform a dilation.
 */
struct DilationKernel
{
    /*!
     * A cubic kernel checks all voxels in a cube around a reference voxel.
     *  _____
     * |\ ___\
     * | |    |
     *  \|____|
     *
     * A diamond kernel uses a manhattan distance to create a diamond shape around a reference voxel.
     *  /|\
     * /_|_\
     * \ | /
     *  \|/
     *
     * A prism kernel is diamond in XY, but extrudes straight in Z around a reference voxel.
     *   / \
     *  /   \
     * |\   /|
     * | \ / |
     * |  |  |
     *  \ | /
     *   \|/
     */
    enum class Type
    {
        CUBE,
        DIAMOND,
        PRISM
    };
    GridPoint3 kernel_size_; //!< Size of the kernel in number of voxel cells
    Type type_;
    std::vector<GridPoint3> relative_cells_; //!< All offset positions relative to some reference cell which is to be dilated

    DilationKernel(GridPoint3 kernel_size, Type type);
};

} // namespace curaengine
