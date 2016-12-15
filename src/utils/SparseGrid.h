/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef UTILS_SPARSE_GRID_H
#define UTILS_SPARSE_GRID_H

#include "intpoint.h"

#include <cassert>
#include <unordered_map>
#include <vector>
#include <functional>

namespace cura {

/*! \brief Sparse grid which can locate spatially nearby elements efficiently.
 * 
 * \tparam ElemT The element type to store.
 */
template<class ElemT>
class SparseGrid
{
public:
    using Elem = ElemT;

    /*! \brief Constructs a sparse grid with the specified cell size.
     *
     * \param[in] cell_size The size to use for a cell (square) in the grid.
     *    Typical values would be around 0.5-2x of expected query radius.
     * \param[in] elem_reserve Number of elements to research space for.
     * \param[in] max_load_factor Maximum average load factor before rehashing.
     */
    SparseGrid(coord_t cell_size, size_t elem_reserve=0U, float max_load_factor=1.0f);

    /*! \brief Returns all data within radius of query_pt.
     *
     * Finds all elements with location within radius of \p query_pt.  May
     * return additional elements that are beyond radius.
     *
     * Average running time is a*(1 + 2 * radius / cell_size)**2 +
     * b*cnt where a and b are proportionality constance and cnt is
     * the number of returned items.  The search will return items in
     * an area of (2*radius + cell_size)**2 on average.  The max range
     * of an item from the query_point is radius + cell_size.
     *
     * \param[in] query_pt The point to search around.
     * \param[in] radius The search radius.
     * \return Vector of elements found
     */
    std::vector<Elem> getNearby(const Point &query_pt, coord_t radius) const;

    static const std::function<bool(const Elem&)> no_precondition;

    /*!
     * Find the nearest element to a given \p query_pt within \p radius.
     *
     * \param[in] query_pt The point for which to find the nearest object.
     * \param[in] radius The search radius.
     * \param[out] elem_nearest the nearest element. Only valid if function returns true.
     * \param[in] precondition A precondition which must return true for an element
     *    to be considered for output
     * \return True if and only if an object has been found within the radius.
     */
    bool getNearest(const Point &query_pt, coord_t radius, Elem &elem_nearest,
                    const std::function<bool(const Elem& elem)> precondition = no_precondition) const;

    /*! \brief Process elements from cells that might contain sought after points.
     *
     * Processes elements from cell that might have elements within \p
     * radius of \p query_pt.  Processes all elements that are within
     * radius of query_pt.  May process elements that are up to radius +
     * cell_size from query_pt.
     *
     * \param[in] query_pt The point to search around.
     * \param[in] radius The search radius.
     * \param[in] process_func Processes each element.  process_func(elem) is
     *    called for each element in the cell. Processing stops if function returns false.
     */
    void processNearby(const Point &query_pt, coord_t radius,
                       const std::function<bool (const ElemT&)>& process_func) const;

    /*! \brief Process elements from cells that might contain sought after points along a line.
     *
     * Processes elements from cells that cross the line \p query_line.
     * May process elements that are up to sqrt(2) * cell_size from \p query_line.
     *
     * \param[in] query_line The line along which to check each cell
     * \param[in] process_func Processes each element.  process_func(elem) is
     *    called for each element in the cells. Processing stops if function returns false.
     */
    void processLine(const std::pair<Point, Point> query_line,
                       const std::function<bool (const Elem&)>& process_elem_func) const;

    coord_t getCellSize() const;

    /*! \brief Inserts elem into the sparse grid.
     *
     * \param[in] location The location where to insert the element
     * \param[in] elem The element to be inserted.
     */
    void insert(Point location, const Elem &elem);
protected:
    using GridPoint = Point;
    using grid_coord_t = coord_t;
    using GridMap = std::unordered_multimap<GridPoint, Elem>;

    /*! \brief Process elements from the cell indicated by \p grid_pt.
     *
     * \param[in] grid_pt The grid coordinates of the cell.
     * \param[in] process_func Processes each element.  process_func(elem) is
     *    called for each element in the cell. Processing stops if function returns false.
     * \return Whether we need to continue processing a next cell.
     */
    bool processFromCell(const GridPoint &grid_pt,
                         const std::function<bool (const Elem&)>& process_func) const;

    /*! \brief Process cells along a line indicated by \p line.
     *
     * \param[in] line The line along which to process cells
     * \param[in] process_func Processes each cell.  process_func(elem) is
     *    called for each cell. Processing stops if function returns false.
     */
    void processLineCells(const std::pair<Point, Point> line,
                         const std::function<bool (GridPoint)>& process_cell_func);

    /*! \brief Process cells along a line indicated by \p line.
     *
     * \param[in] line The line along which to process cells
     * \param[in] process_func Processes each cell.  process_func(elem) is
     *    called for each cell. Processing stops if function returns false.
     */
    void processLineCells(const std::pair<Point, Point> line,
                         const std::function<bool (GridPoint)>& process_cell_func) const;

    /*! \brief Compute the grid coordinates of a point.
     *
     * \param[in] point The actual location.
     * \return The grid coordinates that correspond to \p point.
     */
    GridPoint toGridPoint(const Point& point) const;

    /*! \brief Compute the grid coordinate of a print space coordinate.
     *
     * \param[in] coord The actual location.
     * \return The grid coordinate that corresponds to \p coord.
     */
    grid_coord_t toGridCoord(const coord_t& coord) const;

    /*! \brief Compute the lowest point in a grid cell.
     * The lowest point is the point in the grid cell closest to the origin.
     *
     * \param[in] location The grid location.
     * \return The print space coordinates that correspond to \p location.
     */
    Point toLowerCorner(const GridPoint& location) const; 

    /*! \brief Compute the lowest coord in a grid cell.
     * The lowest point is the point in the grid cell closest to the origin.
     *
     * \param[in] grid_coord The grid coordinate.
     * \return The print space coordinate that corresponds to \p grid_coord.
     */
    coord_t toLowerCoord(const grid_coord_t& grid_coord) const; 

    /*! \brief Map from grid locations (GridPoint) to elements (Elem). */
    GridMap m_grid;
    /*! \brief The cell (square) size. */
    coord_t m_cell_size;

    grid_coord_t nonzero_sign(const grid_coord_t z) const;
};



#define SGI_TEMPLATE template<class ElemT>
#define SGI_THIS SparseGrid<ElemT>

SGI_TEMPLATE
SGI_THIS::SparseGrid(coord_t cell_size, size_t elem_reserve, float max_load_factor)
{
    assert(cell_size > 0U);

    m_cell_size = cell_size;

    // Must be before the reserve call.
    m_grid.max_load_factor(max_load_factor);
    if (elem_reserve != 0U) {
        m_grid.reserve(elem_reserve);
    }
}

SGI_TEMPLATE
typename SGI_THIS::GridPoint SGI_THIS::toGridPoint(const Point &point)  const
{
    return Point(toGridCoord(point.X), toGridCoord(point.Y));
}

SGI_TEMPLATE
typename SGI_THIS::grid_coord_t SGI_THIS::toGridCoord(const coord_t& coord)  const
{
    // This mapping via truncation results in the cells with
    // GridPoint.x==0 being twice as large and similarly for
    // GridPoint.y==0.  This doesn't cause any incorrect behavior,
    // just changes the running time slightly.  The change in running
    // time from this is probably not worth doing a proper floor
    // operation.
    return coord / m_cell_size;
}

SGI_TEMPLATE
typename cura::Point SGI_THIS::toLowerCorner(const GridPoint& location)  const
{
    return cura::Point(toLowerCoord(location.X), toLowerCoord(location.Y));
}

SGI_TEMPLATE
typename cura::coord_t SGI_THIS::toLowerCoord(const grid_coord_t& grid_coord)  const
{
    // This mapping via truncation results in the cells with
    // GridPoint.x==0 being twice as large and similarly for
    // GridPoint.y==0.  This doesn't cause any incorrect behavior,
    // just changes the running time slightly.  The change in running
    // time from this is probably not worth doing a proper floor
    // operation.
    return grid_coord * m_cell_size;
}

SGI_TEMPLATE
void SGI_THIS::insert(Point loc, const Elem &elem)
{
    GridPoint grid_loc = toGridPoint(loc);

    m_grid.emplace(grid_loc, elem);
}

SGI_TEMPLATE
bool SGI_THIS::processFromCell(
    const GridPoint &grid_pt,
    const std::function<bool (const Elem&)>& process_func) const
{
    auto grid_range = m_grid.equal_range(grid_pt);
    for (auto iter = grid_range.first; iter != grid_range.second; ++iter)
    {
        if (!process_func(iter->second))
        {
            return false;
        }
    }
    return true;
}

SGI_TEMPLATE
void SGI_THIS::processLineCells(
    const std::pair<Point, Point> line,
    const std::function<bool (GridPoint)>& process_cell_func)
{
    static_cast<const SGI_THIS*>(this)->processLineCells(line, process_cell_func);
}

SGI_TEMPLATE
void SGI_THIS::processLineCells(
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
                return;
            }
            if (grid_loc == end_cell)
            {
                return;
            }
        }
        // TODO: this causes at least a one cell overlap for each row, which
        // includes extra cells when crossing precisely on the corners
        // where positive slope where x > 0 and negative slope where x < 0
        x_cell_start = x_cell_end;
    }
    assert(false && "We should have returned already before here!");
}

SGI_TEMPLATE
typename SGI_THIS::grid_coord_t SGI_THIS::nonzero_sign(const grid_coord_t z) const
{
    return (z >= 0) - (z < 0);
}

SGI_TEMPLATE
void SGI_THIS::processNearby(const Point &query_pt, coord_t radius,
                             const std::function<bool (const Elem&)>& process_func) const
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
            bool continue_ = processFromCell(grid_pt, process_func);
            if (!continue_)
            {
                return;
            }
        }
    }
}

SGI_TEMPLATE
void SGI_THIS::processLine(const std::pair<Point, Point> query_line,
                            const std::function<bool (const Elem&)>& process_elem_func) const
{
    const std::function<bool (const GridPoint&)> process_cell_func = [&process_elem_func, this](GridPoint grid_loc)
        {
            return processFromCell(grid_loc, process_elem_func);
        };
    processLineCells(query_line, process_cell_func);
}

SGI_TEMPLATE
std::vector<typename SGI_THIS::Elem>
SGI_THIS::getNearby(const Point &query_pt, coord_t radius) const
{
    std::vector<Elem> ret;
    const std::function<bool (const Elem&)> process_func = [&ret](const Elem &elem)
        {
            ret.push_back(elem);
            return true;
        };
    processNearby(query_pt, radius, process_func);
    return ret;
}

SGI_TEMPLATE
const std::function<bool(const typename SGI_THIS::Elem &)>
    SGI_THIS::no_precondition =
    [](const typename SGI_THIS::Elem &)
    {
        return true;
    };

SGI_TEMPLATE
bool SGI_THIS::getNearest(
    const Point &query_pt, coord_t radius, Elem &elem_nearest,
    const std::function<bool(const Elem& elem)> precondition) const
{
    bool found = false;
    int64_t best_dist2 = static_cast<int64_t>(radius) * radius;
    const std::function<bool (const Elem&)> process_func =
        [&query_pt, &elem_nearest, &found, &best_dist2, &precondition](const Elem &elem)
        {
            if (!precondition(elem))
            {
                return true;
            }
            int64_t dist2 = vSize2(elem.point - query_pt);
            if (dist2 < best_dist2)
            {
                found = true;
                elem_nearest = elem;
                best_dist2 = dist2;
            }
            return true;
        };
    processNearby(query_pt, radius, process_func);
    return found;
}

SGI_TEMPLATE
coord_t SGI_THIS::getCellSize() const
{
    return m_cell_size;
}

#undef SGI_TEMPLATE
#undef SGI_THIS

} // namespace cura

#endif // UTILS_SPARSE_GRID_H
