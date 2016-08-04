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

    static void debugTest();
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
    coord_t y_diff = end.Y - start.Y;

    grid_coord_t y_dir = (y_diff >= 0)? 1 : -1;
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
            if (x_cell_end < start_cell.X)
            { // process at least one cell!
                x_cell_end = x_cell_start;
            }
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
    assert(false && "We should have returned already before here!");
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
//         svg.writePoint(lb, true, 1);
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

SGI_TEMPLATE
void SGI_THIS::debugTest()
{
    struct PairLocator
    {
        std::pair<Point, Point> operator()(const std::pair<Point, Point>& val) const
        {
            return val;
        }
    };
    SparseLineGrid<std::pair<Point, Point>, PairLocator> line_grid(10);
    // straight lines
    line_grid.insert(std::make_pair<Point, Point>(Point(50, 0), Point(50, 70)));
    line_grid.insert(std::make_pair<Point, Point>(Point(0, 90), Point(50, 90)));
    line_grid.insert(std::make_pair<Point, Point>(Point(253, 103), Point(253, 173)));
    line_grid.insert(std::make_pair<Point, Point>(Point(203, 193), Point(253, 193)));

    // diagonal lines
    line_grid.insert(std::make_pair<Point, Point>(Point(113, 133), Point(166, 125)));
    line_grid.insert(std::make_pair<Point, Point>(Point(13, 73), Point(26, 25)));
    line_grid.insert(std::make_pair<Point, Point>(Point(166, 33), Point(113, 25)));
    line_grid.insert(std::make_pair<Point, Point>(Point(26, 173), Point(13, 125)));

    // diagonal lines exactly crossing cell corners
    line_grid.insert(std::make_pair<Point, Point>(Point(160, 190), Point(220, 170)));
    line_grid.insert(std::make_pair<Point, Point>(Point(60, 130), Point(80, 70)));
    line_grid.insert(std::make_pair<Point, Point>(Point(220, 90), Point(160, 70)));
    line_grid.insert(std::make_pair<Point, Point>(Point(80, 220), Point(60, 160)));

    // single cell
    line_grid.insert(std::make_pair<Point, Point>(Point(203, 213), Point(203, 213)));
    line_grid.insert(std::make_pair<Point, Point>(Point(223, 213), Point(223, 215)));
    line_grid.insert(std::make_pair<Point, Point>(Point(243, 213), Point(245, 213)));
    line_grid.insert(std::make_pair<Point, Point>(Point(263, 213), Point(265, 215)));
    line_grid.insert(std::make_pair<Point, Point>(Point(283, 215), Point(285, 213)));

    line_grid.debugHTML("line_grid.html");
}


#undef SGI_TEMPLATE
#undef SGI_THIS

} // namespace cura

#endif // UTILS_SPARSE_LINE_GRID_H
