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

struct DilationKernel
{
    enum class Type { CUBE, DIAMOND, PRISM };
    DilationKernel(GridPoint3 kernel_size, Type type);
    GridPoint3 kernel_size;
    Type type;
    std::vector<GridPoint3> relative_cells;
};

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

    bool walkDilatedPolygons(const Polygons& polys, coord_t z, const DilationKernel& kernel, const std::function<bool (GridPoint3)>& process_cell_func);

private:
    /*!
     * \warning the \p polys is assumed to be translated by half the cell_size in xy alreadu
     */
    bool _walkAreas(const Polygons& polys, coord_t z, const std::function<bool (GridPoint3)>& process_cell_func);

public:
    bool walkAreas(const Polygons& polys, coord_t z, const std::function<bool (GridPoint3)>& process_cell_func);
    bool walkDilatedAreas(const Polygons& polys, coord_t z, const DilationKernel& kernel, const std::function<bool (GridPoint3)>& process_cell_func);

    /*!
     * Dilate with a kernel
     * 
     * The kernel either has a cubic shape or a diamond shape
     * 
     * If the \p kernel_size is even then the kernel is applied off center, such that \p loc is toward the lower end
     */
    bool dilate(GridPoint3 loc, const DilationKernel& kernel, const std::function<bool (GridPoint3)>& process_cell_func);

    std::function<bool (GridPoint3)> dilate(const DilationKernel& kernel, const std::function<bool (GridPoint3)>& process_cell_func);
    
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
