//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_VOXEL_UTILS_H
#define UTILS_VOXEL_UTILS_H

#include <functional>
#include <unordered_set>

#include "utils/IntPoint.h"
#include "utils/polygon.h"

namespace cura 
{
/*
struct GridPoint3 : public Point3
{
    template<typename... Args>
    GridPoint3(Args... args)
    : Point3(args...)
    {}
    GridPoint3()
    : Point3()
    {}
};
*/

using GridPoint3 = Point3;

class VoxelUtils
{
public:
    using grid_coord_t = coord_t;
    VoxelUtils(Point3 cell_size)
    : cell_size(cell_size)
    {}
    Point3 cell_size;

    /*!
     * \return Whether executing was stopped short as indicated by the \p cell_processing_function
     */
    bool walkLine(Point3 start, Point3 end, const std::function<bool (GridPoint3)>& process_cell_func);

    bool walkPolygons(const Polygons& polys, coord_t z, const std::function<bool (GridPoint3)>& process_cell_func);

    bool walkDilatedPolygons(const Polygons& polys, coord_t z, const std::vector<GridPoint3>& kernel, GridPoint3 kernel_size, const std::function<bool (GridPoint3)>& process_cell_func);

    /*!
     * Dilate with a kernel
     * 
     * The kernel either has a cubic shape or a diamond shape
     * 
     * If the \p kernel_size is even then the kernel is applied off center, such that \p loc is toward the lower end
     */
    static std::vector<GridPoint3> dilationKernel(GridPoint3 kernel_size, bool diamond_kernel);

    bool dilate(GridPoint3 loc, const std::vector<GridPoint3>& kernel, const std::function<bool (GridPoint3)>& process_cell_func);
    
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
    
    
};

} // namespace cura

#endif // UTILS_VOXEL_UTILS_H
