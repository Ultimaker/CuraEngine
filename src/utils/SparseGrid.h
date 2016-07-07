/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef SPARSE_GRID_H
#define SPARSE_GRID_H

#include "intpoint.h"

#include <unordered_map>
#include <vector>

namespace cura {

/*! \brief Sparse grid which can locate spatially nearby values efficiently.
 *
 * \tparam Val The value type to store.
 * \tparam PointAccess The functor to get the location from Val.  PointAccess
 *    must have: Point operator()(const Val &val) const
 *    which returns the location associated with val.
 */
template<class Val, class PointAccess>
class SparseGrid
{
public:
    /*! \brief Constructs a sparse grid with the specified cell size.
     *
     * \param[in] cell_size The size to use for a cell (square) in the grid.
     *    Typical values would be around 0.5-2x of expected query radius.
     */
    SparseGrid(coord_t cell_size);

    /*! \brief Inserts val into the sparse grid.
     *
     * \param[in] val The value to be inserted.
     */
    void insert(const Val &val);

    /*! \brief Returns all data within radius of query_pt.
     *
     * Finds all values with location within radius of \p query_pt.  May
     * return additional values that are beyond radius.
     *
     * Average running time is a*(1 + 2 * radius / cell_size)**2 +
     * b*cnt where a and b are proportionality constance and cnt is
     * the number of returned items.  The search will return items in
     * an area of (2*radius + cell_size)**2 on average.  The max range
     * of an item from the query_point is radius + cell_size.
     *
     * \param[in] query_pt The point to search around.
     * \param[in] radius The search radius.
     * \return Vector of values found
     */
    std::vector<Val> getNearby(const Point &query_pt, coord_t radius) const;
    
private:
    using GridPoint = Point;
    using GridMap = std::unordered_multimap<GridPoint, Val>;

    /*! \brief Add values from the cell indicated by \p grid_pt.
     *
     * \param[in] grid_pt The grid coordinates of the cell.
     * \param[out] ret The values in the cell are appended to ret.
     */
    void addFromCell(const GridPoint &grid_pt,
                     std::vector<Val> &ret) const;
    
    /*! \brief Add values from cells that might contain sought after points.
     *
     * Appends values from cell that might have values within \p
     * radius of \p query_pt.  Appends all values that are within
     * radius of query_pt.  May append values that are up to radius +
     * cell_size from query_pt.
     *
     * \param[in] query_pt The point to search around.
     * \param[in] radisu The search radius.
     * \param[out] ret The values in the cell are appended to ret.
     */
    void addNearby(const Point &query_pt, coord_t radius,
                   std::vector<Val> &ret) const;

    /*! \brief Compute the grid coordinates of a point.
     *
     * \param[in] point The actual location.
     * \return The grid coordinates that correspond to point.
     */
    GridPoint toGridPoint(const Point &point) const;

    /*! \brief Map from grid locations (GridPoint) to values (Val). */
    GridMap m_grid;
    /*! \brief Accessor for getting locations from values. */
    PointAccess m_point_access;
    /*! \brief The cell (square) size. */
    coord_t m_cell_size;
};

#define SG_TEMPLATE template<class Val, class PointAccess>
#define SG_THIS SparseGrid<Val, PointAccess>

SG_TEMPLATE
SG_THIS::SparseGrid(coord_t cell_size)
{
    m_cell_size = cell_size;
}

SG_TEMPLATE
typename SG_THIS::GridPoint SG_THIS::toGridPoint(const Point &point)  const
{
    // This mapping via truncation results in the cells with
    // GridPoint.x==0 being twice as large and similarly for
    // GridPoint.y==0.  This doesn't cause any incorrect behavior,
    // just changes the running time slightly.  The change in running
    // time from this is probably not worth doing a proper floor
    // operation.
    return point / m_cell_size;
}

SG_TEMPLATE
void SG_THIS::insert(const Val &val)
{
    Point loc = m_point_access(val);
    GridPoint grid_loc = toGridPoint(loc);

    m_grid.emplace(grid_loc,val);
}

SG_TEMPLATE
void SG_THIS::addFromCell(const GridPoint &grid_pt,
                          std::vector<Val> &ret) const
{
    auto grid_range = m_grid.equal_range(grid_pt);
    for (auto iter=grid_range.first; iter!=grid_range.second; ++iter)
    {
        ret.push_back(iter->second);
    }
    
}

SG_TEMPLATE
void SG_THIS::addNearby(const Point &query_pt, coord_t radius,
                        std::vector<Val> &ret) const
{
    Point min_loc(query_pt.X-radius, query_pt.Y-radius);
    Point max_loc(query_pt.X+radius, query_pt.Y+radius);

    GridPoint min_grid = toGridPoint(min_loc);
    GridPoint max_grid = toGridPoint(max_loc);

    for (coord_t grid_y=min_grid.Y; grid_y<=max_grid.Y; ++grid_y)
    {
        for (coord_t grid_x=min_grid.X; grid_x<=max_grid.X; ++grid_x)
        {
            GridPoint grid_pt(grid_x,grid_y);
            addFromCell(grid_pt, ret);
        }
    }
}

SG_TEMPLATE
std::vector<Val> SG_THIS::getNearby(const Point &query_pt, coord_t radius) const
{
    std::vector<Val> ret;
    addNearby(query_pt, radius, ret);
    return ret;
}

#undef SG_TEMPLATE
#undef SG_THIS

}

#endif // SPARSE_GRID_H
