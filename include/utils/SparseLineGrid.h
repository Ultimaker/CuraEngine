// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.


#ifndef UTILS_SPARSE_LINE_GRID_H
#define UTILS_SPARSE_LINE_GRID_H

#include <cassert>
#include <functional>
#include <unordered_map>
#include <vector>

#include "SVG.h" // debug
#include "SparseGrid.h"
#include "geometry/Point2LL.h"

namespace cura
{

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
    using typename SparseGrid<ElemT>::GridMap;

    /*! \brief Constructs a sparse grid with the specified cell size.
     *
     * \param[in] cell_size The size to use for a cell (square) in the grid.
     *    Typical values would be around 0.5-2x of expected query radius.
     * \param[in] elem_reserve Number of elements to research space for.
     * \param[in] max_load_factor Maximum average load factor before rehashing.
     */
    SparseLineGrid(coord_t cell_size, size_t elem_reserve = 0U, double max_load_factor = 1.0);

    /*! \brief Inserts elem into the sparse grid.
     *
     * \param[in] elem The element to be inserted.
     */
    void insert(const Elem& elem);

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
SGI_THIS::SparseLineGrid(coord_t cell_size, size_t elem_reserve, double max_load_factor)
    : SparseGrid<ElemT>(cell_size, elem_reserve, max_load_factor)
{
}

SGI_TEMPLATE
void SGI_THIS::insert(const Elem& elem)
{
    const std::pair<Point2LL, Point2LL> line = m_locator(elem);
    // below is a workaround for the fact that lambda functions cannot access private or protected members
    // first we define a lambda which works on any GridMap and then we bind it to the actual protected GridMap of the parent class
    std::function<bool(GridMap*, const GridPoint)> process_cell_func_ = [&elem, this](GridMap* grid, const GridPoint grid_loc)
    {
        grid->emplace(grid_loc, elem);
        return true;
    };
    using namespace std::placeholders; // for _1, _2, _3...
    GridMap* grid = &(this->grid_);
    std::function<bool(const GridPoint)> process_cell_func(std::bind(process_cell_func_, grid, _1));

    SparseGrid<ElemT>::processLineCells(line, process_cell_func);
}

SGI_TEMPLATE
void SGI_THIS::debugHTML(std::string filename)
{
    AABB aabb;
    for (std::pair<GridPoint, ElemT> cell : SparseGrid<ElemT>::grid_)
    {
        aabb.include(SparseGrid<ElemT>::toLowerCorner(cell.first));
        aabb.include(SparseGrid<ElemT>::toLowerCorner(cell.first + GridPoint(SparseGrid<ElemT>::nonzero_sign(cell.first.X), SparseGrid<ElemT>::nonzero_sign(cell.first.Y))));
    }
    SVG svg(filename.c_str(), aabb);
    for (std::pair<GridPoint, ElemT> cell : SparseGrid<ElemT>::grid_)
    {
        // doesn't draw cells at x = 0 or y = 0 correctly (should be double size)
        Point2LL lb = SparseGrid<ElemT>::toLowerCorner(cell.first);
        Point2LL lt = SparseGrid<ElemT>::toLowerCorner(cell.first + GridPoint(0, SparseGrid<ElemT>::nonzero_sign(cell.first.Y)));
        Point2LL rt = SparseGrid<ElemT>::toLowerCorner(cell.first + GridPoint(SparseGrid<ElemT>::nonzero_sign(cell.first.X), SparseGrid<ElemT>::nonzero_sign(cell.first.Y)));
        Point2LL rb = SparseGrid<ElemT>::toLowerCorner(cell.first + GridPoint(SparseGrid<ElemT>::nonzero_sign(cell.first.X), 0));
        if (lb.X == 0)
        {
            lb.X = -SparseGrid<ElemT>::cell_size_;
            lt.X = -SparseGrid<ElemT>::cell_size_;
        }
        if (lb.Y == 0)
        {
            lb.Y = -SparseGrid<ElemT>::cell_size_;
            rb.Y = -SparseGrid<ElemT>::cell_size_;
        }
        //         svg.writePoint(lb, true, 1);
        svg.writeLine(lb, lt, SVG::Color::GRAY);
        svg.writeLine(lt, rt, SVG::Color::GRAY);
        svg.writeLine(rt, rb, SVG::Color::GRAY);
        svg.writeLine(rb, lb, SVG::Color::GRAY);

        std::pair<Point2LL, Point2LL> line = m_locator(cell.second);
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
        std::pair<Point2LL, Point2LL> operator()(const std::pair<Point2LL, Point2LL>& val) const
        {
            return val;
        }
    };
    SparseLineGrid<std::pair<Point2LL, Point2LL>, PairLocator> line_grid(10);

    // straight lines
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(50, 0), Point2LL(50, 70)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(0, 90), Point2LL(50, 90)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(253, 103), Point2LL(253, 173)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(203, 193), Point2LL(253, 193)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(-50, 0), Point2LL(-50, -70)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(0, -90), Point2LL(-50, -90)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(-253, -103), Point2LL(-253, -173)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(-203, -193), Point2LL(-253, -193)));

    // diagonal lines
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(113, 133), Point2LL(166, 125)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(13, 73), Point2LL(26, 25)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(166, 33), Point2LL(113, 25)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(26, 173), Point2LL(13, 125)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(-24, -18), Point2LL(-19, -64)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(-113, -133), Point2LL(-166, -125)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(-166, -33), Point2LL(-113, -25)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(-26, -173), Point2LL(-13, -125)));

    // diagonal lines exactly crossing cell corners
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(160, 190), Point2LL(220, 170)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(60, 130), Point2LL(80, 70)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(220, 90), Point2LL(160, 70)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(80, 220), Point2LL(60, 160)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(-160, -190), Point2LL(-220, -170)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(-60, -130), Point2LL(-80, -70)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(-220, -90), Point2LL(-160, -70)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(-80, -220), Point2LL(-60, -160)));

    // single cell
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(203, 213), Point2LL(203, 213)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(223, 213), Point2LL(223, 215)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(243, 213), Point2LL(245, 213)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(263, 213), Point2LL(265, 215)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(283, 215), Point2LL(285, 213)));
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(-203, -213), Point2LL(-203, -213)));

    // around origin
    line_grid.insert(std::make_pair<Point2LL, Point2LL>(Point2LL(20, -20), Point2LL(-20, 20)));

    line_grid.debugHTML("line_grid.html");
}


#undef SGI_TEMPLATE
#undef SGI_THIS

} // namespace cura

#endif // UTILS_SPARSE_LINE_GRID_H
