//Copyright (c) 2016 Scott Lenser
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_SPARSE_GRID_H
#define UTILS_SPARSE_GRID_H

#include <cassert>
#include <unordered_map>
#include <vector>
#include <functional>

#include "IntPoint.h"
#include "SquareGrid.h"

namespace cura {

/*! \brief Sparse grid which can locate spatially nearby elements efficiently.
 * 
 * \note This is an abstract template class which doesn't have any functions to insert elements.
 * \see SparsePointGrid
 *
 * \tparam ElemT The element type to store.
 */
template<class ElemT>
class SparseGrid : public SquareGrid
{
public:
    using Elem = ElemT;

    using GridPoint = SquareGrid::GridPoint;
    using grid_coord_t = SquareGrid::grid_coord_t;
    using GridMap = std::unordered_multimap<GridPoint, Elem>;
    
    using iterator = typename GridMap::iterator;
    using const_iterator = typename GridMap::const_iterator;

    /*! \brief Constructs a sparse grid with the specified cell size.
     *
     * \param[in] cell_size The size to use for a cell (square) in the grid.
     *    Typical values would be around 0.5-2x of expected query radius.
     * \param[in] elem_reserve Number of elements to research space for.
     * \param[in] max_load_factor Maximum average load factor before rehashing.
     */
    SparseGrid(coord_t cell_size, size_t elem_reserve=0U, float max_load_factor=1.0f);
    
    iterator begin()
    {
        return m_grid.begin();
    }
    
    iterator end()
    {
        return m_grid.end();
    }

    const_iterator begin() const
    {
        return m_grid.begin();
    }
    
    const_iterator end() const
    {
        return m_grid.end();
    }
    
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
     * \return Whether we need to continue processing after this function
     */
    bool processNearby(const Point &query_pt, coord_t radius,
                       const std::function<bool (const ElemT&)>& process_func) const;

    /*! \brief Process elements from cells that might contain sought after points along a line.
     *
     * Processes elements from cells that cross the line \p query_line.
     * May process elements that are up to sqrt(2) * cell_size from \p query_line.
     *
     * \param[in] query_line The line along which to check each cell
     * \param[in] process_func Processes each element.  process_func(elem) is
     *    called for each element in the cells. Processing stops if function returns false.
     * \return Whether we need to continue processing after this function
     */
    bool processLine(const std::pair<Point, Point> query_line,
                       const std::function<bool (const Elem&)>& process_elem_func) const;

protected:
    /*! \brief Process elements from the cell indicated by \p grid_pt.
     *
     * \param[in] grid_pt The grid coordinates of the cell.
     * \param[in] process_func Processes each element.  process_func(elem) is
     *    called for each element in the cell. Processing stops if function returns false.
     * \return Whether we need to continue processing a next cell.
     */
    bool processFromCell(const GridPoint &grid_pt,
                         const std::function<bool (const Elem&)>& process_func) const;

    /*! \brief Map from grid locations (GridPoint) to elements (Elem). */
    GridMap m_grid;
};



#define SGI_TEMPLATE template<class ElemT>
#define SGI_THIS SparseGrid<ElemT>

SGI_TEMPLATE
SGI_THIS::SparseGrid(coord_t cell_size, size_t elem_reserve, float max_load_factor)
: SquareGrid(cell_size)
{
    // Must be before the reserve call.
    m_grid.max_load_factor(max_load_factor);
    if (elem_reserve != 0U) {
        m_grid.reserve(elem_reserve);
    }
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
bool SGI_THIS::processNearby(const Point &query_pt, coord_t radius,
                             const std::function<bool (const Elem&)>& process_func) const
{
    return SquareGrid::processNearby(query_pt, radius,
                                     [&process_func, this](const GridPoint& grid_pt)
                                     {
                                         return processFromCell(grid_pt, process_func);
                                     });
}

SGI_TEMPLATE
bool SGI_THIS::processLine(const std::pair<Point, Point> query_line,
                            const std::function<bool (const Elem&)>& process_elem_func) const
{
    const std::function<bool (const GridPoint&)> process_cell_func = [&process_elem_func, this](GridPoint grid_loc)
        {
            return processFromCell(grid_loc, process_elem_func);
        };
    return processLineCells(query_line, process_cell_func);
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

#undef SGI_TEMPLATE
#undef SGI_THIS

} // namespace cura

#endif // UTILS_SPARSE_GRID_H
