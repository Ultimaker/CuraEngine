//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_VOXEL_UTILS_H
#define UTILS_VOXEL_UTILS_H

#include <functional>
#include <unordered_set>

#include "utils/IntPoint.h"
#include "utils/polygon.h"

namespace cura 
{

using GridPoint3 = Point3;

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
    enum class Type { CUBE, DIAMOND, PRISM };
    DilationKernel(GridPoint3 kernel_size, Type type);
    GridPoint3 kernel_size; //!< Size of the kernel in number of voxel cells
    Type type;
    std::vector<GridPoint3> relative_cells; //!< All offset positions relative to some reference cell which is to be dilated
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
    VoxelUtils(Point3 cell_size)
    : cell_size(cell_size)
    {}
    Point3 cell_size;

    /*!
     * Process voxels which a line segment crosses.
     * 
     * \param start Start point of the line
     * \param end End point of the line
     * \param process_cell_func Function to perform on each cell the line crosses
     * \return Whether executing was stopped short as indicated by the \p cell_processing_function
     */
    bool walkLine(Point3 start, Point3 end, const std::function<bool (GridPoint3)>& process_cell_func) const;

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
    bool walkPolygons(const Polygons& polys, coord_t z, const std::function<bool (GridPoint3)>& process_cell_func) const;

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
    bool walkDilatedPolygons(const Polygons& polys, coord_t z, const DilationKernel& kernel, const std::function<bool (GridPoint3)>& process_cell_func) const;

private:
    /*!
     * \warning the \p polys is assumed to be translated by half the cell_size in xy already
     */
    bool _walkAreas(const Polygons& polys, coord_t z, const std::function<bool (GridPoint3)>& process_cell_func) const;

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
    bool walkAreas(const Polygons& polys, coord_t z, const std::function<bool (GridPoint3)>& process_cell_func) const;

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
    bool walkDilatedAreas(const Polygons& polys, coord_t z, const DilationKernel& kernel, const std::function<bool (GridPoint3)>& process_cell_func) const;

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
    std::function<bool (GridPoint3)> dilate(const DilationKernel& kernel, const std::function<bool (GridPoint3)>& process_cell_func) const;
    
    GridPoint3 toGridPoint(const Point3& point)  const
    {
        return GridPoint3(toGridCoord(point.x, 0), toGridCoord(point.y, 1), toGridCoord(point.z, 2));
    }
    
    grid_coord_t toGridCoord(const coord_t& coord, const size_t dim)  const
    {
        assert(dim < 3);
        return coord / cell_size[dim] - (coord < 0);
    }
    
    Point3 toLowerCorner(const GridPoint3& location)  const
    {
        return cura::Point3(toLowerCoord(location.x, 0), toLowerCoord(location.y, 1), toLowerCoord(location.z, 2));
    }
    
    coord_t toLowerCoord(const grid_coord_t& grid_coord, const size_t dim)  const
    {
        assert(dim < 3);
        return grid_coord * cell_size[dim];
    }

    /*!
     * Returns a rectangular polygon equal to the cross section of a voxel cell at coordinate \p p
     */
    Polygon toPolygon(const GridPoint3 p) const
    {
        Polygon ret;
        Point3 c = toLowerCorner(p);
        ret.emplace_back(c.x, c.y);
        ret.emplace_back(c.x + cell_size.x, c.y);
        ret.emplace_back(c.x + cell_size.x, c.y + cell_size.y);
        ret.emplace_back(c.x, c.y + cell_size.y);
        return ret;
    }
    
};

} // namespace cura

#endif // UTILS_VOXEL_UTILS_H
