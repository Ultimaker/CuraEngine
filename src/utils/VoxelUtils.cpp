// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/VoxelUtils.h"

#include "utils/PolygonUtils.h"

using namespace curaengine::geometry;

namespace curaengine::utils
{

bool VoxelUtils::walkLine(Point3LL start, Point3LL end, const std::function<bool(GridPoint3)>& process_cell_func) const
{
    Point3LL diff = end - start;

    const GridPoint3 start_cell = toGridPoint(start);
    const GridPoint3 end_cell = toGridPoint(end);
    if (start_cell == end_cell)
    {
        return process_cell_func(start_cell);
    }

    Point3LL current_cell = start_cell;
    while (true)
    {
        bool continue_ = process_cell_func(current_cell);

        if (! continue_)
        {
            return false;
        }

        int stepping_dim = -1; // dimension in which the line next exits the current cell
        double percentage_along_line = std::numeric_limits<double>::max();
        for (int dim = 0; dim < 3; dim++)
        {
            if (diff[dim] == 0)
            {
                continue;
            }
            coord_t crossing_boundary = toLowerCoord(current_cell[dim], dim) + (diff[dim] > 0) * cell_size_[dim];
            double percentage_along_line_here = (crossing_boundary - start[dim]) / static_cast<double>(diff[dim]);
            if (percentage_along_line_here < percentage_along_line)
            {
                percentage_along_line = percentage_along_line_here;
                stepping_dim = dim;
            }
        }
        assert(stepping_dim != -1);
        if (percentage_along_line > 1.0)
        {
            // next cell is beyond the end
            return true;
        }
        current_cell[stepping_dim] += (diff[stepping_dim] > 0) ? 1 : -1;
    }
    return true;
}


bool VoxelUtils::walkPolygons(const Shape& polys, coord_t z, const std::function<bool(GridPoint3)>& process_cell_func) const
{
    for (const Polygon& poly : polys)
    {
        Point2LL last = poly.back();
        for (Point2LL p : poly)
        {
            bool continue_ = walkLine(Point3LL(last.X, last.Y, z), Point3LL(p.X, p.Y, z), process_cell_func);
            if (! continue_)
            {
                return false;
            }
            last = p;
        }
    }
    return true;
}

bool VoxelUtils::walkDilatedPolygons(const Shape& polys, coord_t z, const DilationKernel& kernel, const std::function<bool(GridPoint3)>& process_cell_func) const
{
    Shape translated = polys;
    const Point3LL translation = (Point3LL(1, 1, 1) - kernel.kernel_size_ % 2) * cell_size_ / 2;
    if (translation.x_ && translation.y_)
    {
        translated.translate(Point2LL(translation.x_, translation.y_));
    }
    return walkPolygons(translated, z + translation.z_, dilate(kernel, process_cell_func));
}

bool VoxelUtils::walkAreas(const Shape& polys, coord_t z, const std::function<bool(GridPoint3)>& process_cell_func) const
{
    Shape translated = polys;
    const Point3LL translation = -cell_size_ / 2; // offset half a cell so that the dots of spreadDotsArea are centered on the middle of the cell isntead of the lower corners.
    if (translation.x_ && translation.y_)
    {
        translated.translate(Point2LL(translation.x_, translation.y_));
    }
    return _walkAreas(translated, z, process_cell_func);
}

bool VoxelUtils::_walkAreas(const Shape& polys, coord_t z, const std::function<bool(GridPoint3)>& process_cell_func) const
{
    std::vector<Point2LL> skin_points = PolygonUtils::spreadDotsArea(polys, Point2LL(cell_size_.x_, cell_size_.y_));
    for (Point2LL p : skin_points)
    {
        bool continue_ = process_cell_func(toGridPoint(Point3LL(p.X + cell_size_.x_ / 2, p.Y + cell_size_.y_ / 2, z)));
        if (! continue_)
        {
            return false;
        }
    }
    return true;
}

bool VoxelUtils::walkDilatedAreas(const Shape& polys, coord_t z, const DilationKernel& kernel, const std::function<bool(GridPoint3)>& process_cell_func) const
{
    Shape translated = polys;
    const Point3LL translation = (Point3LL(1, 1, 1) - kernel.kernel_size_ % 2) * cell_size_ / 2 // offset half a cell when using a n even kernel
                               - cell_size_ / 2; // offset half a cell so that the dots of spreadDotsArea are centered on the middle of the cell isntead of the lower corners.
    if (translation.x_ && translation.y_)
    {
        translated.translate(Point2LL(translation.x_, translation.y_));
    }
    return _walkAreas(translated, z + translation.z_, dilate(kernel, process_cell_func));
}

std::function<bool(GridPoint3)> VoxelUtils::dilate(const DilationKernel& kernel, const std::function<bool(GridPoint3)>& process_cell_func) const
{
    return [&process_cell_func, &kernel](GridPoint3 loc)
    {
        for (const GridPoint3& rel : kernel.relative_cells_)
        {
            bool continue_ = process_cell_func(loc + rel);
            if (! continue_)
                return false;
        }
        return true;
    };
}
} // namespace curaengine::utils
