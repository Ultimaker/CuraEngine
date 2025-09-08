// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_VOXELGRID_H
#define UTILS_VOXELGRID_H

#include <execution>

#include <boost/unordered/concurrent_flat_map.hpp>

#include "Coord_t.h"
#include "geometry/Triangle3D.h"
#include "utils/Point3D.h"


namespace cura
{

struct AABB3D;

/*!
 * Represent a voxel grid in 3D space. It is strongly optimized for huge grid with low memory consumption, and is completely thread-safe. The empty voxels are not represented,
 * only the "occupied" ones are, with a data value. The data value is currently a uint8_t but it could easily be upgraded to support anything else,
 * at the cost of a higher memory usage.
 */
class VoxelGrid
{
public:
    /*! Local coordinates of voxels are stored on XYZ being unsigned 16-bit integers, so that we can store the whole position on a 64-bit integer. It can represent a build plate
     * of 6.5m side with a 0.1mm resolution, which should be good enough... */
    struct Point3U16
    {
        uint16_t x;
        uint16_t y;
        uint16_t z;
        uint16_t black_hole{ 0 }; // Don't place anything in there, or it would be lost forever (it exists only to properly set the 4th byte of the key)
    };

    /*! This union allows to store a position in the grid by XYZ position stored on UINT16 values, and also to use this position as a key that can
     * be stored in a map, without having to separately hash the X, Y and Z components, which saves some time when heavily dealing with large maps. */
    union LocalCoordinates
    {
        uint64_t key{ 0 };
        Point3U16 position;

        LocalCoordinates(const uint16_t x, const uint16_t y, const uint16_t z)
            : position{ x, y, z }
        {
        }

        bool operator==(const LocalCoordinates& other) const
        {
            return key == other.key;
        }
        bool operator!=(const LocalCoordinates& other) const
        {
            return key != other.key;
        }
        bool operator<(const LocalCoordinates& other) const
        {
            return key < other.key;
        }
    };

public:
    /*!
     * Build an empty voxel grid
     * @param bounding_box The bounding box of the represented grid
     * @param max_resolution The maximum authorized X, Y and Z resolution for this grid. The actual calculated resolution is always smaller than this value, and can be different
     *                       in each direction
     */
    explicit VoxelGrid(const AABB3D& bounding_box, const coord_t max_resolution);

    const Point3D& getResolution() const
    {
        return resolution_;
    }

    Point3LL getSlicesCount() const
    {
        return slices_count_;
    }

    Point3D toGlobalCoordinates(const LocalCoordinates& position, const bool at_center = true) const;

    void setOccupation(const LocalCoordinates& position, const uint8_t extruder_nr);

    void setOrUpdateOccupation(const LocalCoordinates& position, const uint8_t extruder_nr);

    std::optional<uint8_t> getOccupation(const LocalCoordinates& local_position) const;

    size_t occupiedCount() const;

    /*!
     * Visits alls the occupied voxels with the function given as argument. Ths functions should take a single argument which is a pair containing the key (local coordinate)
     * and the value (extruder occupation)
     * @warning The occupied voxels are processed in parallel, so make sure the given function doesn't use external elements that are not thread-safe. Also, since we are iterating
     *          on the occupied voxels list, it is impossible to do any other operation on the voxels grid itself, e.g. looking for the occupation of an other voxel.
     */
    template<class... Args>
    void visitOccupiedVoxels(Args&&... args)
    {
        occupied_voxels_.visit_all(
#ifdef __cpp_lib_execution
            std::execution::par,
#endif
            args...);
    }

    /*!
     * Visits alls the occupied voxels with the function given as argument. Ths functions should take a single argument which is a pair containing the key (local coordinate)
     * and the value (extruder occupation)
     * @warning The occupied voxels are processed in parallel, so make sure the given function doesn't use external elements that are not thread-safe. Also, since we are iterating
     *          on the occupied voxels list, it is impossible to do any other operation on the voxels grid itself, e.g. looking for the occupation of an other voxel.
     */
    template<class... Args>
    void visitOccupiedVoxels(Args&&... args) const
    {
        occupied_voxels_.cvisit_all(
#ifdef __cpp_lib_execution
            std::execution::par,
#endif
            args...);
    }

    std::vector<LocalCoordinates> getVoxelsAround(const LocalCoordinates& point) const;

    LocalCoordinates toLocalCoordinates(const Point3D& position) const;

    uint16_t toLocalX(const double x) const;

    double toGlobalX(const uint16_t x, const bool at_center = true) const;

    uint16_t toLocalY(const double y) const;

    double toGlobalY(const uint16_t y, const bool at_center = true) const;

    uint16_t toLocalZ(const double z) const;

    double toGlobalZ(const uint16_t z, const bool at_center = true) const;

    /*!
     * @brief Gets all the voxels traversed by the given triangle, which is similar to rasterizing the triangle, but in 3D
     * @param triangle The 3D triangle we want to rasterize
     * @return The list of voxels traversed by the triangle
     *
     * To do a fast (enough) rasterization of the triangle, we calculate its bounding box in the voxels grid on the X axis, then we iterate on all the YZ columns
     * in the X direction. For each column we crop the edges of the triangle to fit inside the width of the column, which gives a sub-triangle or a quad shape. Then for this
     * sub-shape, we calculate its bounding box in the Y axis, and iterate on all the Z "square tubes". For each cube we crop the edges again, and iterate over the Z bounding boxes
     * of the sub-sub-shape. Each iterated position is then considered as being traversed by the triangle.
     */
    std::vector<LocalCoordinates> getTraversedVoxels(const Triangle3D& triangle) const;

private:
    Point3D resolution_;
    Point3D origin_;
    Point3LL slices_count_;
    boost::concurrent_flat_map<LocalCoordinates, uint8_t> occupied_voxels_;
};

inline std::size_t hash_value(VoxelGrid::LocalCoordinates const& position)
{
    return boost::hash<uint64_t>()(position.key);
}

} // namespace cura

#endif