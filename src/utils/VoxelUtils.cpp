// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/VoxelUtils.h"

#include "utils/polygonUtils.h"

namespace cura
{

DilationKernel::DilationKernel(GridPoint3 kernel_size, DilationKernel::Type type)
    : kernel_size_(kernel_size)
    , type_(type)
{
    coord_t mult = kernel_size.x_ * kernel_size.y_ * kernel_size.z_; // multiplier for division to avoid rounding and to avoid use of floating point numbers
    relative_cells_.reserve(mult);
    GridPoint3 half_kernel = kernel_size / 2;

    GridPoint3 start = -half_kernel;
    GridPoint3 end = kernel_size - half_kernel;
    for (coord_t x = start.x_; x < end.x_; x++)
    {
        for (coord_t y = start.y_; y < end.y_; y++)
        {
            for (coord_t z = start.z_; z < end.z_; z++)
            {
                GridPoint3 current(x, y, z);
                if (type != Type::CUBE)
                {
                    GridPoint3 limit((x < 0) ? start.x_ : end.x_ - 1, (y < 0) ? start.y_ : end.y_ - 1, (z < 0) ? start.z_ : end.z_ - 1);
                    if (limit.x_ == 0)
                        limit.x_ = 1;
                    if (limit.y_ == 0)
                        limit.y_ = 1;
                    if (limit.z_ == 0)
                        limit.z_ = 1;
                    const GridPoint3 rel_dists = mult * current / limit;
                    if ((type == Type::DIAMOND && rel_dists.x_ + rel_dists.y_ + rel_dists.z_ > mult) || (type == Type::PRISM && rel_dists.x_ + rel_dists.y_ > mult))
                    {
                        continue; // don't consider this cell
                    }
                }
                relative_cells_.emplace_back(x, y, z);
            }
        }
    }
}

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
} // namespace cura
