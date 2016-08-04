/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */

#ifndef UTILS_SPARSE_LINE_GRID_H
#define UTILS_SPARSE_LINE_GRID_H

#include <cassert>
#include <unordered_map>
#include <vector>

#include "intpoint.h"
#include "SparseGrid.h"
#include "SVG.h" // debug

namespace cura {

/*! \brief Sparse grid which can locate spatially nearby elements efficiently.
 *
 * \tparam ElemT The element type to store.
 * \tparam Locator The functor to get the start and end locations from ElemT.
 *    must have: std::pair<Point, Point> operator()(const ElemT &elem) const
 *    which returns the location associated with val.
 */
template<class ElemT, class Locator>
class SparseLineGrid : public SparseGrid<ElemT>
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
    SparseLineGrid(coord_t cell_size, size_t elem_reserve=0U, float max_load_factor=1.0f);

    /*! \brief Inserts elem into the sparse grid.
     *
     * \param[in] elem The element to be inserted.
     */
    void insert(const Elem &elem);
    
    void debugHTML(std::string filename);

protected:
    using GridPoint = typename SparseGrid<ElemT>::GridPoint;
    using grid_coord_t = typename SparseGrid<ElemT>::grid_coord_t;

    /*! \brief Accessor for getting locations from elements. */
    Locator m_locator;
};



#define SGI_TEMPLATE template<class ElemT, class Locator>
#define SGI_THIS SparseLineGrid<ElemT, Locator>

SGI_TEMPLATE
SGI_THIS::SparseLineGrid(coord_t cell_size, size_t elem_reserve, float max_load_factor)
 : SparseGrid<ElemT>(cell_size, elem_reserve, max_load_factor)
{
}

SGI_TEMPLATE
void SGI_THIS::insert(const Elem &elem)
{
    const std::pair<Point, Point> line = m_locator(elem);
    Point start = line.first;
    Point end = line.second;
    if (end.X < start.X)
    { // make sure X increases between start and end
        std::swap(start, end);
    }

    const GridPoint start_cell = SparseGrid<ElemT>::toGridPoint(start);
    const GridPoint end_cell = SparseGrid<ElemT>::toGridPoint(end);
    coord_t y_diff = (end.Y - start.Y);

    grid_coord_t y_dir = (end_cell.Y > start_cell.Y)? 1 : -1;
    grid_coord_t x_cell_start = start_cell.X;
    for (grid_coord_t cell_y = start_cell.Y; cell_y * y_dir <= end_cell.Y * y_dir; cell_y += y_dir)
    { // for all Y from start to end
        coord_t nearest_next_y = SparseGrid<ElemT>::toLowerCoord(cell_y + std::max(coord_t(0), y_dir)); // nearest y coordinate of the cells in the next row
        grid_coord_t x_cell_end; // the X coord of the last cell to include from this row
        if (y_diff == 0)
        {
            x_cell_end = end_cell.X;
        }
        else
        {
            coord_t corresponding_x = start.X + (end.X - start.X) * (nearest_next_y - start.Y) / y_diff;
            x_cell_end = SparseGrid<ElemT>::toGridCoord(corresponding_x);
        }

        for (grid_coord_t cell_x = x_cell_start; cell_x <= x_cell_end; ++cell_x)
        {
            GridPoint grid_loc(cell_x, cell_y);
            SparseGrid<ElemT>::m_grid.emplace(grid_loc,elem);
            if (grid_loc == end_cell)
            {
                return;
            }
        }
        x_cell_start = x_cell_end; // TODO: doesn't account for lines crossing cell boundaries exactly diagonally over the 4-way intersection point
    }
}

SGI_TEMPLATE
void SGI_THIS::debugHTML(std::string filename)
{
    AABB aabb;
    for (std::pair<GridPoint, ElemT> cell:  SparseGrid<ElemT>::m_grid)
    {
        aabb.include(SparseGrid<ElemT>::toLowerCorner(cell.first));
        aabb.include(SparseGrid<ElemT>::toLowerCorner(cell.first + GridPoint(1, 1)));
    }
    SVG svg(filename.c_str(), aabb);
    for (std::pair<GridPoint, ElemT> cell:  SparseGrid<ElemT>::m_grid)
    {
        Point lb = SparseGrid<ElemT>::toLowerCorner(cell.first);
        Point lt = SparseGrid<ElemT>::toLowerCorner(cell.first + GridPoint(0, 1));
        Point rt = SparseGrid<ElemT>::toLowerCorner(cell.first + GridPoint(1, 1));
        Point rb = SparseGrid<ElemT>::toLowerCorner(cell.first + GridPoint(1, 0));
        svg.writePoint(lb, true);
        svg.writeLine(lb, lt, SVG::Color::GRAY);
        svg.writeLine(lt, rt, SVG::Color::GRAY);
        svg.writeLine(rt, rb, SVG::Color::GRAY);
        svg.writeLine(rb, lb, SVG::Color::GRAY);

        std::pair<Point, Point> line = m_locator(cell.second);
        svg.writePoint(line.first, true);
        svg.writePoint(line.second, true);
        svg.writeLine(line.first, line.second, SVG::Color::BLACK);
    }
}


#undef SGI_TEMPLATE
#undef SGI_THIS

} // namespace cura

#endif // UTILS_SPARSE_LINE_GRID_H
