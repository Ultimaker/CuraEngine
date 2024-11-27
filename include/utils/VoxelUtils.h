// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_VOXEL_UTILS_H
#define UTILS_VOXEL_UTILS_H

#include <functional>
#include <unordered_set>

#include "geometry/Point3LL.h"
#include "geometry/Polygon.h"

namespace cura
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

/*!
 * Utility class for walking over a 3D voxel grid.
 *
 * Contains the math for intersecting voxels with lines, polgons, areas, etc.
 */
class VoxelUtils
{
public:
    using grid_coord_t = coord_t;

    Point3LL cell_size_;

    VoxelUtils(Point3LL cell_size)
        : cell_size_(cell_size)
    {
    }

    /*!
     * Process voxels which a line segment crosses.
     *
     * \param start Start point of the line
     * \param end End point of the line
     * \param process_cell_func Function to perform on each cell the line crosses
     * \return Whether executing was stopped short as indicated by the \p cell_processing_function
     */
    bool walkLine(Point3LL start, Point3LL end, const std::function<bool(GridPoint3)>& process_cell_func) const;

    /*!
     * Process voxels which the line segments of a polygon crosses.
     *
     * \warning Voxels may be processed multiple times!
     *
     * \param polys The polygons to walk
     * \param z The height at which the polygons occur
     * \param process_cell_func Function to perform on each voxel cell
     * \return Whether executing was stopped short as indicated by the \p cell_processing_function
     */
    bool walkPolygons(const Shape& polys, coord_t z, const std::function<bool(GridPoint3)>& process_cell_func) const;

    /*!
     * Process voxels near the line segments of a polygon.
     * For each voxel the polygon crosses we process each of the offset voxels according to the kernel.
     *
     * \warning Voxels may be processed multiple times!
     *
     * \param polys The polygons to walk
     * \param z The height at which the polygons occur
     * \param process_cell_func Function to perform on each voxel cell
     * \return Whether executing was stopped short as indicated by the \p cell_processing_function
     */
    bool walkDilatedPolygons(const Shape& polys, coord_t z, const DilationKernel& kernel, const std::function<bool(GridPoint3)>& process_cell_func) const;

private:
    /*!
     * \warning the \p polys is assumed to be translated by half the cell_size in xy already
     */
    bool _walkAreas(const Shape& polys, coord_t z, const std::function<bool(GridPoint3)>& process_cell_func) const;

public:
    /*!
     * Process all voxels inside the area of a polygons object.
     *
     * \warning The voxels along the area are not processed. Thin areas might not process any voxels at all.
     *
     * \param polys The area to fill
     * \param z The height at which the polygons occur
     * \param process_cell_func Function to perform on each voxel cell
     * \return Whether executing was stopped short as indicated by the \p cell_processing_function
     */
    bool walkAreas(const Shape& polys, coord_t z, const std::function<bool(GridPoint3)>& process_cell_func) const;

    /*!
     * Process all voxels inside the area of a polygons object.
     * For each voxel inside the polygon we process each of the offset voxels according to the kernel.
     *
     * \warning The voxels along the area are not processed. Thin areas might not process any voxels at all.
     *
     * \param polys The area to fill
     * \param z The height at which the polygons occur
     * \param process_cell_func Function to perform on each voxel cell
     * \return Whether executing was stopped short as indicated by the \p cell_processing_function
     */
    bool walkDilatedAreas(const Shape& polys, coord_t z, const DilationKernel& kernel, const std::function<bool(GridPoint3)>& process_cell_func) const;

    /*!
     * Dilate with a kernel.
     *
     * Extends the \p process_cell_func, so that for each cell we process nearby cells as well.
     *
     * Apply this function to a process_cell_func to create a new process_cell_func which applies the effect to nearby voxels as well.
     *
     * \param kernel The offset positions relative to the input of \p process_cell_func
     * \param process_cell_func Function to perform on each voxel cell
     */
    std::function<bool(GridPoint3)> dilate(const DilationKernel& kernel, const std::function<bool(GridPoint3)>& process_cell_func) const;

    GridPoint3 toGridPoint(const Point3LL& point) const
    {
        return GridPoint3(toGridCoord(point.x_, 0), toGridCoord(point.y_, 1), toGridCoord(point.z_, 2));
    }

    grid_coord_t toGridCoord(const coord_t& coord, const size_t dim) const
    {
        assert(dim < 3);
        return coord / cell_size_[dim] - (coord < 0);
    }

    Point3LL toLowerCorner(const GridPoint3& location) const
    {
        return cura::Point3LL(toLowerCoord(location.x_, 0), toLowerCoord(location.y_, 1), toLowerCoord(location.z_, 2));
    }

    coord_t toLowerCoord(const grid_coord_t& grid_coord, const size_t dim) const
    {
        assert(dim < 3);
        return grid_coord * cell_size_[dim];
    }

    /*!
     * Returns a rectangular polygon equal to the cross section of a voxel cell at coordinate \p p
     */
    Polygon toPolygon(const GridPoint3 p) const
    {
        Polygon ret;
        Point3LL c = toLowerCorner(p);
        ret.emplace_back(c.x_, c.y_);
        ret.emplace_back(c.x_ + cell_size_.x_, c.y_);
        ret.emplace_back(c.x_ + cell_size_.x_, c.y_ + cell_size_.y_);
        ret.emplace_back(c.x_, c.y_ + cell_size_.y_);
        return ret;
    }
};

} // namespace cura

#endif // UTILS_VOXEL_UTILS_H
