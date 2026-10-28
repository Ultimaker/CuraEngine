// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <optional>

#include <boost/geometry/algorithms/comparable_distance.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "utils/OccupiedPosition.h"

namespace curaengine
{

class VoxelGrid;

/*!
 * The lookup tree structure stores both the occupied positions list, and the RTree that allows a fast lookup. Since the RTree holds internal reference to the list, this ensures
 * that the list and the RTree will live along with each other.
 */
class SpatialLookup
{
public:
    using LookupTree = boost::geometry::index::rtree<OccupiedPosition, boost::geometry::index::quadratic<8>>;

    SpatialLookup() = default;

    /*! Find the occupation that is closest to the given position */
    std::optional<OccupiedPosition> findClosestOccupation(const Point3D& position) const;

    /*!
     * Make a spatial lookup based on the data stored in the given voxels grid
     * @param voxel_grid The voxels grid filled with occupation, for which we want to process quick spatial queries
     * @return The spatial lookup to be used
     */
    static SpatialLookup makeSpatialLookupFromVoxelGrid(const VoxelGrid& voxel_grid);

private:
    std::vector<OccupiedPosition> occupied_positions_;
    LookupTree lookup_tree_;
};

} // namespace curaengine
