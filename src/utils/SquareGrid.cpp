//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SquareGrid.h"

using namespace cura;


SquareGrid::SquareGrid(coord_t cell_size)
{
    assert(cell_size > 0U);
    m_cell_size = cell_size;
}


SquareGrid::GridPoint SquareGrid::toGridPoint(const Point &point)  const
{
    return Point(toGridCoord(point.X), toGridCoord(point.Y));
}


SquareGrid::grid_coord_t SquareGrid::toGridCoord(const coord_t& coord)  const
{
    // This mapping via truncation results in the cells with
    // GridPoint.x==0 being twice as large and similarly for
    // GridPoint.y==0.  This doesn't cause any incorrect behavior,
    // just changes the running time slightly.  The change in running
    // time from this is probably not worth doing a proper floor
    // operation.
    return coord / m_cell_size;
}


cura::Point SquareGrid::toLowerCorner(const GridPoint& location)  const
{
    return cura::Point(toLowerCoord(location.X), toLowerCoord(location.Y));
}


cura::coord_t SquareGrid::toLowerCoord(const grid_coord_t& grid_coord)  const
{
    // This mapping via truncation results in the cells with
    // GridPoint.x==0 being twice as large and similarly for
    // GridPoint.y==0.  This doesn't cause any incorrect behavior,
    // just changes the running time slightly.  The change in running
    // time from this is probably not worth doing a proper floor
    // operation.
    return grid_coord * m_cell_size;
}


bool SquareGrid::processLineCells(
    const std::pair<Point, Point> line,
    const std::function<bool (GridPoint)>& process_cell_func)
{
    return static_cast<const SquareGrid*>(this)->processLineCells(line, process_cell_func);
}


bool SquareGrid::processLineCells(
    const std::pair<Point, Point> line,
    const std::function<bool (GridPoint)>& process_cell_func) const
{

    Point start = line.first;
    Point end = line.second;
    if (end.X < start.X)
    { // make sure X increases between start and end
        std::swap(start, end);
    }

    const GridPoint start_cell = toGridPoint(start);
    const GridPoint end_cell = toGridPoint(end);
    const coord_t y_diff = end.Y - start.Y;
    const grid_coord_t y_dir = nonzero_sign(y_diff);

    grid_coord_t x_cell_start = start_cell.X;
    for (grid_coord_t cell_y = start_cell.Y; cell_y * y_dir <= end_cell.Y * y_dir; cell_y += y_dir)
    { // for all Y from start to end
        // nearest y coordinate of the cells in the next row
        coord_t nearest_next_y = toLowerCoord(cell_y + ((nonzero_sign(cell_y) == y_dir || cell_y == 0) ? y_dir : coord_t(0)));
        grid_coord_t x_cell_end; // the X coord of the last cell to include from this row
        if (y_diff == 0)
        {
            x_cell_end = end_cell.X;
        }
        else
        {
            coord_t area = (end.X - start.X) * (nearest_next_y - start.Y);
            // corresponding_x: the x coordinate corresponding to nearest_next_y
            coord_t corresponding_x = start.X + area / y_diff;
            x_cell_end = toGridCoord(corresponding_x + ((corresponding_x < 0) && ((area % y_diff) != 0)));
            if (x_cell_end < start_cell.X)
            { // process at least one cell!
                x_cell_end = x_cell_start;
            }
        }

        for (grid_coord_t cell_x = x_cell_start; cell_x <= x_cell_end; ++cell_x)
        {
            GridPoint grid_loc(cell_x, cell_y);
            bool continue_ = process_cell_func(grid_loc);
            if (!continue_)
            {
                return false;
            }
            if (grid_loc == end_cell)
            {
                return true;
            }
        }
        // TODO: this causes at least a one cell overlap for each row, which
        // includes extra cells when crossing precisely on the corners
        // where positive slope where x > 0 and negative slope where x < 0
        x_cell_start = x_cell_end;
    }
    assert(false && "We should have returned already before here!");
    return false;
}


bool SquareGrid::processNearby(const Point &query_pt, coord_t radius,
                             const std::function<bool (const GridPoint&)>& process_func) const
{
    Point min_loc(query_pt.X - radius, query_pt.Y - radius);
    Point max_loc(query_pt.X + radius, query_pt.Y + radius);

    GridPoint min_grid = toGridPoint(min_loc);
    GridPoint max_grid = toGridPoint(max_loc);

    for (coord_t grid_y = min_grid.Y; grid_y <= max_grid.Y; ++grid_y)
    {
        for (coord_t grid_x = min_grid.X; grid_x <= max_grid.X; ++grid_x)
        {
            GridPoint grid_pt(grid_x,grid_y);
            bool continue_ = process_func(grid_pt);
            if (!continue_)
            {
                return false;
            }
        }
    }
    return true;
}



SquareGrid::grid_coord_t SquareGrid::nonzero_sign(const grid_coord_t z) const
{
    return (z >= 0) - (z < 0);
}

coord_t SquareGrid::getCellSize() const
{
    return m_cell_size;
}
