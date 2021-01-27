//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "VoxelUtils.h"
#include "utils/polygonUtils.h"

namespace cura 
{

DilationKernel::DilationKernel(GridPoint3 kernel_size, bool diamond_kernel)
: kernel_size(kernel_size)
, diamond_kernel(diamond_kernel)
{
    coord_t mult = kernel_size.x * kernel_size.y * kernel_size.z; // multiplier for division to avoid rounding and to avoid use of floating point numbers
    relative_cells.reserve(mult);
    GridPoint3 half_kernel = kernel_size / 2;

    GridPoint3 start = - half_kernel;
    GridPoint3 end = kernel_size - half_kernel;
    for (coord_t x = start.x; x < end.x; x++)
    {
        for (coord_t y = start.y; y < end.y; y++)
        {
            for (coord_t z = start.z; z < end.z; z++)
            {
                GridPoint3 current(x, y, z);
                if (diamond_kernel)
                {
                    GridPoint3 limit((x < 0)? start.x : end.x - 1,
                                     (y < 0)? start.y : end.y - 1,
                                     (z < 0)? start.z : end.z - 1);
                    if (limit.x == 0) limit.x = 1;
                    if (limit.y == 0) limit.y = 1;
                    if (limit.z == 0) limit.z = 1;
                    const GridPoint3 rel_dists = mult * current / limit;
                    coord_t manhattan_dist = rel_dists.x + rel_dists.y + rel_dists.z;
                    //                     manhattan_dist -=   mult / std::abs(limit.x) +
                    //                                         mult / std::abs(limit.y) +
                    //                                         mult / std::abs(limit.z) +
                    if (manhattan_dist > mult)
                    {
                        continue; // don't consider this cell
                    }
                }
                relative_cells.emplace_back(x, y, z);
            }
        }
    }
}

bool VoxelUtils::walkLine(Point3 start, Point3 end, const std::function<bool (GridPoint3)>& process_cell_func)
{
    Point3 diff = end - start;
    
    const GridPoint3 start_cell = toGridPoint(start);
    const GridPoint3 end_cell = toGridPoint(end);
    if (start_cell == end_cell)
    {
        return process_cell_func(start_cell);
    }
    
    Point3 current_cell = start_cell;
    while (true)
    {
        bool continue_ = process_cell_func(current_cell);
        
        if ( ! continue_) return false;
        if (current_cell == end_cell) break;
        
        int stepping_dim = -1; // dimension in which the line next exits the current cell
        float percentage_along_line = 1.1;
        for (int dim = 0; dim < 3; dim++)
        {
            if (diff[dim] == 0) continue;
            coord_t crossing_boundary = toLowerCoord(current_cell[dim], dim) + (diff[dim] > 0) * cell_size[dim];
            float percentage_along_line_here = (crossing_boundary - start[dim]) / static_cast<float>(diff[dim]);
            if (percentage_along_line_here < percentage_along_line)
            {
                percentage_along_line = percentage_along_line_here;
                stepping_dim = dim;
            }
        }
        assert(stepping_dim != -1);
        current_cell[stepping_dim] += (diff[stepping_dim] > 0) ? 1 : -1;
    }
    return true;
}


bool VoxelUtils::walkPolygons(const Polygons& polys, coord_t z, const std::function<bool (GridPoint3)>& process_cell_func)
{
    for (ConstPolygonRef poly : polys)
    {
        Point last = poly.back();
        for (Point p : poly)
        {
            bool continue_ = walkLine(Point3(last.X, last.Y, z), Point3(p.X, p.Y, z), process_cell_func);
            if ( ! continue_) return false;
            last = p;
        }
    }
    return true;
}

bool VoxelUtils::walkDilatedPolygons(const Polygons& polys, coord_t z, const DilationKernel& kernel, const std::function<bool (GridPoint3)>& process_cell_func)
{
    Polygons translated = polys;
    const Point3 translation = (Point3(1,1,1) - kernel.kernel_size % 2) * -cell_size / 2;
    if (translation.x && translation.y)
    {
        translated.translate(Point(translation.x, translation.y));
    }
    return walkPolygons(translated, z + translation.z, [&kernel, &process_cell_func, this](GridPoint3 cell) { return dilate(cell, kernel, process_cell_func); });
}

bool VoxelUtils::walkAreas(const Polygons& polys, coord_t z, const std::function<bool (GridPoint3)>& process_cell_func)
{
    std::vector<Point> skin_points = PolygonUtils::spreadDotsArea(polys, cell_size.x);
    for (Point p : skin_points)
    {
        bool continue_ = process_cell_func(toGridPoint(Point3(p.X, p.Y, z)));
        if ( ! continue_) return false;
    }
    return true;
}

bool VoxelUtils::walkDilatedAreas(const Polygons& polys, coord_t z, const DilationKernel& kernel, const std::function<bool (GridPoint3)>& process_cell_func)
{
    Polygons translated = polys;
    const Point3 translation = (Point3(1,1,1) - kernel.kernel_size % 2) * -cell_size / 2;
    if (translation.x && translation.y)
    {
        translated.translate(Point(translation.x, translation.y));
    }
    return walkAreas(translated, z + translation.z, [&kernel, &process_cell_func, this](GridPoint3 cell) { return dilate(cell, kernel, process_cell_func); });
}

bool VoxelUtils::dilate(GridPoint3 loc, const DilationKernel& kernel, const std::function<bool (GridPoint3)>& process_cell_func)
{
    for (const GridPoint3& rel : kernel.relative_cells)
    {
        bool continue_ = process_cell_func(loc + rel);
        if ( ! continue_) return false;
    }
    return true;
}

std::function<bool (GridPoint3)> VoxelUtils::dilate(const DilationKernel& kernel, const std::function<bool (GridPoint3)>& process_cell_func)
{
    return [&process_cell_func, &kernel, this](GridPoint3 p) { return dilate(p, kernel, process_cell_func); };
}
} // namespace cura
