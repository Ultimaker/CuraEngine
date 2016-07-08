/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef SPARSE_GRID_H
#define SPARSE_GRID_H

#include "intpoint.h"

#include <unordered_map>
#include <vector>

namespace cura {

/*! \brief Sparse grid which can locate spatially nearby elements efficiently.
 *
 * \tparam ElemT The element type to store.
 * \tparam PointAccess The functor to get the location from ElemT.  PointAccess
 *    must have: Point operator()(const ElemT &elem) const
 *    which returns the location associated with val.
 */
template<class ElemT, class PointAccess>
class SparseGridInvasive
{
public:
    using Elem = ElemT;

    /*! \brief Constructs a sparse grid with the specified cell size.
     *
     * \param[in] cell_size The size to use for a cell (square) in the grid.
     *    Typical values would be around 0.5-2x of expected query radius.
     */
    SparseGridInvasive(coord_t cell_size);

    /*! \brief Inserts elem into the sparse grid.
     *
     * \param[in] elem The element to be inserted.
     */
    void insert(const Elem &elem);

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

private:
    using GridPoint = Point;
    using GridMap = std::unordered_multimap<GridPoint, Elem>;

    /*! \brief Add elements from the cell indicated by \p grid_pt.
     *
     * \param[in] grid_pt The grid coordinates of the cell.
     * \param[out] ret The elements in the cell are appended to ret.
     */
    void addFromCell(const GridPoint &grid_pt,
                     std::vector<Elem> &ret) const;

    /*! \brief Add elements from cells that might contain sought after points.
     *
     * Appends elements from cell that might have elements within \p
     * radius of \p query_pt.  Appends all elements that are within
     * radius of query_pt.  May append elements that are up to radius +
     * cell_size from query_pt.
     *
     * \param[in] query_pt The point to search around.
     * \param[in] radisu The search radius.
     * \param[out] ret The elements in the cell are appended to ret.
     */
    void addNearby(const Point &query_pt, coord_t radius,
                   std::vector<Elem> &ret) const;

    /*! \brief Compute the grid coordinates of a point.
     *
     * \param[in] point The actual location.
     * \return The grid coordinates that correspond to point.
     */
    GridPoint toGridPoint(const Point &point) const;

    /*! \brief Map from grid locations (GridPoint) to elements (Elem). */
    GridMap m_grid;
    /*! \brief Accessor for getting locations from elements. */
    PointAccess m_point_access;
    /*! \brief The cell (square) size. */
    coord_t m_cell_size;
};

namespace SparseGridImpl {

template<class Val>
struct SparseGridElem
{
    SparseGridElem()
    {
    }

    SparseGridElem(const Point &point_, const Val &val_) :
        point(point_),
        val(val_)
    {
    }

    Point point;
    Val val;
};

template<class T>
struct PointAccessor
{
    Point operator()(const SparseGridElem<T> &elem)
    {
        return elem.point;
    }
};

} // namespace SparseGridImpl

template<class Val>
class SparseGrid : public SparseGridInvasive<SparseGridImpl::SparseGridElem<Val>,
                                             SparseGridImpl::PointAccessor<Val> >
{
public:
    using Base = SparseGridInvasive<SparseGridImpl::SparseGridElem<Val>,
                                    SparseGridImpl::PointAccessor<Val> >;
    
    /*! \brief Constructs a sparse grid with the specified cell size.
     *
     * \param[in] cell_size The size to use for a cell (square) in the grid.
     *    Typical values would be around 0.5-2x of expected query radius.
     */
    SparseGrid(coord_t cell_size);

    /*! \brief Inserts an element with specified point and value into the sparse grid.
     *
     * This is a convenience wrapper over \ref SparseGridInvasive::insert()
     *
     * \param[in] point The location for the element.
     * \param[in] val The value for the element.
     */
    void insert(const Point &point, const Val &val);
};

#define SGI_TEMPLATE template<class ElemT, class PointAccess>
#define SGI_THIS SparseGridInvasive<ElemT, PointAccess>

SGI_TEMPLATE
SGI_THIS::SparseGridInvasive(coord_t cell_size)
{
    m_cell_size = cell_size;
}

SGI_TEMPLATE
typename SGI_THIS::GridPoint SGI_THIS::toGridPoint(const Point &point)  const
{
    // This mapping via truncation results in the cells with
    // GridPoint.x==0 being twice as large and similarly for
    // GridPoint.y==0.  This doesn't cause any incorrect behavior,
    // just changes the running time slightly.  The change in running
    // time from this is probably not worth doing a proper floor
    // operation.
    return point / m_cell_size;
}

SGI_TEMPLATE
void SGI_THIS::insert(const Elem &elem)
{
    Point loc = m_point_access(elem);
    GridPoint grid_loc = toGridPoint(loc);

    m_grid.emplace(grid_loc,elem);
}

SGI_TEMPLATE
void SGI_THIS::addFromCell(const GridPoint &grid_pt,
                          std::vector<Elem> &ret) const
{
    auto grid_range = m_grid.equal_range(grid_pt);
    for (auto iter=grid_range.first; iter!=grid_range.second; ++iter)
    {
        ret.push_back(iter->second);
    }

}

SGI_TEMPLATE
void SGI_THIS::addNearby(const Point &query_pt, coord_t radius,
                        std::vector<Elem> &ret) const
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

SGI_TEMPLATE
std::vector<typename SGI_THIS::Elem>
SGI_THIS::getNearby(const Point &query_pt, coord_t radius) const
{
    std::vector<Elem> ret;
    addNearby(query_pt, radius, ret);
    return ret;
}

#undef SGI_TEMPLATE
#undef SGI_THIS

#define SG_TEMPLATE template<class Val>
#define SG_THIS SparseGrid<Val>

SG_TEMPLATE
SG_THIS::SparseGrid(coord_t cell_size) : Base(cell_size)
{
}

SG_TEMPLATE
void SG_THIS::insert(const Point &point, const Val &val)
{
    insert(SparseGridElem(point,val));
}


#undef SG_TEMPLATE
#undef SG_THIS

}

#endif // SPARSE_GRID_H
