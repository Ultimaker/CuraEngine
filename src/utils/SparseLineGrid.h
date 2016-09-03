/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#ifndef UTILS_SPARSE_LINE_GRID_H
#define UTILS_SPARSE_LINE_GRID_H

#include <cassert>
#include <unordered_map>
#include <vector>

#include "intpoint.h"
#include "SparseGrid.h"
#include "SVG.h" // debug
#include "logoutput.h"

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
    SparseLineGrid(coord_t cell_size, size_t elem_reserve = 0U, float max_load_factor = 1.0f);

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
    const Point start = line.first;
    const Point end = line.second;

    const GridPoint start_cell = SparseGrid<ElemT>::toGridPoint(start);
    const GridPoint end_cell = SparseGrid<ElemT>::toGridPoint(end);

    const grid_coord_t x_cell_step = start_cell.X < end_cell.X ? 1 : -1;
    const grid_coord_t y_cell_step = start_cell.Y < end_cell.Y ? 1 : -1;
    const grid_coord_t x_cell_diff = (end_cell.X - start_cell.X) * x_cell_step;
    const grid_coord_t y_cell_diff = (end_cell.Y - start_cell.Y) * y_cell_step;

    GridPoint grid_loc(start_cell.X, start_cell.Y);
    // the algorithm works by successively substracting the small diff from the
    // large diff until all that is left is a remainder, then it steps in the
    // other axis and adds a large diff again. The starting large diff should
    // be scaled by the inverse (size of cell - error) of the amount of
    // starting error in the small diff.
    grid_coord_t err;
    int max_steps = 0;
    if (x_cell_diff > y_cell_diff)
    {
        // cell edge in the direction we're heading (y_cell_step)
        grid_coord_t y_cell_edge = SparseGrid<ElemT>::toLowerCoord(start_cell.Y + ((y_cell_step * start.Y < 0) ? 0 : y_cell_step));
        // since toLowerCoord always truncates toward the origin, we're heading
        // toward the origin if the signs of the step and the coord are opposite
        grid_coord_t y_remaining_err = std::abs(start.Y - y_cell_edge);
        // scale of y_remaining_err to cell size determines how soon we'll make
        // the first step in y (afterward at rate of x_cell_diff / y_cell_diff)
        err = x_cell_diff * y_remaining_err / SparseGrid<ElemT>::toLowerCoord(1);
        max_steps = x_cell_diff + 1;
    }
    else
    {
        // see comment in y computation, x is symmetric
        grid_coord_t x_cell_edge = SparseGrid<ElemT>::toLowerCoord(start_cell.X + ((x_cell_step * start.X < 0) ? 0 : x_cell_step));
        grid_coord_t x_remaining_err = std::abs(start.X - x_cell_edge);
        err = -y_cell_diff * x_remaining_err / SparseGrid<ElemT>::toLowerCoord(1);
        max_steps = y_cell_diff + 1;
    }
    // +err means we're stepping in x watching for steps in y
    // -err means we're stepping in y watching for steps in x

    while (max_steps--) {
        SparseGrid<ElemT>::m_grid.emplace(grid_loc, elem);
        if (x_cell_step * grid_loc.X >= x_cell_step * end_cell.X || y_cell_step * grid_loc.Y >= y_cell_step * end_cell.Y)
        {
            return;
        }

        grid_coord_t cur_err = err;
        if (cur_err > -x_cell_diff)
        {
            err -= y_cell_diff;
            grid_loc.X += x_cell_step;
        }
        if (cur_err > -x_cell_diff && cur_err < y_cell_diff)
        {
            // this is the fat line version of Bresenham's in that on each step
            // we fill in both of the "aliasing" pixels
            GridPoint grid_other(grid_loc.X - x_cell_step, grid_loc.Y + y_cell_step);
            SparseGrid<ElemT>::m_grid.emplace(grid_loc, elem);
            SparseGrid<ElemT>::m_grid.emplace(grid_other, elem);
        }
        if (cur_err < y_cell_diff)
        {
            err += x_cell_diff;
            grid_loc.Y += y_cell_step;
        }
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
