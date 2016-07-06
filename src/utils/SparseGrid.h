/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef SPARSE_GRID_H
#define SPARSE_GRID_H

#include "intpoint.h"

#include <unordered_map>
#include <vector>

namespace cura {

template<class Val, class PointAccess>
class SparseGrid
{
public:
    SparseGrid(coord_t cell_size);

    void insert(const Val &val);

    /** \brief Returns all data within radius of query_pt.
     *
     * May return additional values that are beyond radius.
     *
     * Average running time is a*(1 + 2 * radius / cell_size)**2 +
     * b*cnt where a and b are proportionality constance and cnt is
     * the number of returned items.  The search will return items in
     * an area of (2*radius + cell_size)**2 on average.
     */
    std::vector<Val> getNearby(const Point &query_pt, coord_t radius) const;
    
private:
    using GridPoint = Point;
    using GridMap = std::unordered_multimap<GridPoint, Val>;

    void addFromCell(const GridPoint &grid_pt,
                     std::vector<Val> &ret) const;
    void addNearby(const Point &query_pt, coord_t radius,
                   std::vector<Val> &ret) const;

    GridPoint toGridPoint(const Point &point) const;
    
    GridMap m_grid;
    PointAccess m_point_access;
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
    for (auto iter=grid_range.first; iter!=grid_range.second; ++iter) {
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

    for (coord_t grid_y=min_grid.Y; grid_y<=max_grid.Y; ++grid_y) {
        for (coord_t grid_x=min_grid.X; grid_x<=max_grid.X; ++grid_x) {
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

#endif
    
