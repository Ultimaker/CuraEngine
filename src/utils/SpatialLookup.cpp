// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/SpatialLookup.h"

#include "utils/VoxelGrid.h"


BOOST_GEOMETRY_REGISTER_POINT_3D(cura::Point3D, double, boost::geometry::cs::cartesian, x_, y_, z_)

namespace cura
{

std::optional<OccupiedPosition> SpatialLookup::findClosestOccupation(const Point3D& position) const
{
    std::vector<OccupiedPosition> nearest_neighbors;
    lookup_tree_.query(boost::geometry::index::nearest(position, 1), std::back_inserter(nearest_neighbors));
    return nearest_neighbors.empty() ? std::nullopt : std::make_optional(nearest_neighbors.front());
}

SpatialLookup SpatialLookup::makeSpatialLookupFromVoxelGrid(const VoxelGrid& voxel_grid)
{
    SpatialLookup spatial_lookup;
    spatial_lookup.occupied_positions_.reserve(voxel_grid.occupiedCount());

    std::mutex mutex;
    voxel_grid.visitOccupiedVoxels(
        [&spatial_lookup, &voxel_grid, &mutex](const auto& voxel)
        {
            const OccupiedPosition occupied_position{ voxel_grid.toGlobalCoordinates(voxel.first), voxel.second };
            const std::lock_guard lock(mutex);
            spatial_lookup.occupied_positions_.push_back(occupied_position);
        });

    spatial_lookup.lookup_tree_ = LookupTree(spatial_lookup.occupied_positions_.begin(), spatial_lookup.occupied_positions_.end());

    return spatial_lookup;
}

} // namespace cura
