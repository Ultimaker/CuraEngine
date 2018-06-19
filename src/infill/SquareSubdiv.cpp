/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#include "SquareSubdiv.h"

#include <cassert>
#include <sstream> // debug output

#include "../utils/math.h"
#include "../utils/linearAlg2D.h"
#include "../utils/gettime.h"
#include "../utils/string.h" // PrecisionedDouble


namespace cura {

SquareSubdiv::SquareSubdiv(const DensityProvider& density_provider, const AABB3D aabb, const int max_depth, coord_t line_width, bool space_filling_curve)
: InfillFractal2D<AABB>(density_provider, aabb, max_depth, line_width, /* root_is_bogus = */ false)
, space_filling_curve(space_filling_curve)
{
}


float SquareSubdiv::getDensity(const Cell& cell) const
{
    AABB cell_aabb = cell.elem;
    AABB3D cell_aabb3d(Point3(cell_aabb.min.X, cell_aabb.min.Y, Base::aabb.min.z), Point3(cell_aabb.max.X, cell_aabb.max.Y, Base::aabb.max.z));
    return density_provider(cell_aabb3d);
}



void SquareSubdiv::createTree()
{
    assert(cell_data.empty());
    size_t to_be_reserved = std::pow(4, max_depth); // note: total tree size equals leaf_count*2-1
    logDebug("Cross3D reserved %i nodes\n", to_be_reserved);
    cell_data.reserve(to_be_reserved);


    AABB root_geometry(aabb.flatten());
    cell_data.emplace_back(root_geometry, /*index =*/ 0, /* depth =*/ 0);

    createTree(cell_data[0]);

    setVolume(cell_data[0]);

    setSpecificationAllowance(cell_data[0]);
}

void SquareSubdiv::createTree(Cell& sub_tree_root)
{
    int parent_depth = sub_tree_root.depth;
    if (parent_depth >= Base::max_depth)
    {
        for (int child_idx = 0; child_idx < 4; child_idx++)
        {
            sub_tree_root.children[child_idx] = -1;
        }
        return;
    }

    idx_t parent_idx = sub_tree_root.index; // use index below because we are changing the data vector, so sub_tree_root might get invalidated!

    const AABB& parent_aabb = sub_tree_root.elem;
    Point middle = parent_aabb.getMiddle();
    // Add children ordered on polarity and dimension: first X from less to more, then Y
    idx_t child_index_lb = cell_data.size();
    cell_data[parent_idx].children[0] = child_index_lb;
    cell_data.emplace_back(AABB(parent_aabb.min, middle), child_index_lb, parent_depth + 1);
    idx_t child_index_rb = cell_data.size();
    cell_data[parent_idx].children[1] = child_index_rb;
    cell_data.emplace_back(AABB(Point(middle.X, parent_aabb.min.Y), Point(parent_aabb.max.X, middle.Y)), child_index_rb, parent_depth + 1);
    idx_t child_index_lt = cell_data.size();
    cell_data[parent_idx].children[2] = child_index_lt;
    cell_data.emplace_back(AABB(Point(parent_aabb.min.X, middle.Y), Point(middle.X, parent_aabb.max.Y)), child_index_lt, parent_depth + 1);
    idx_t child_index_rt = cell_data.size();
    cell_data[parent_idx].children[3] = child_index_rt;
    cell_data.emplace_back(AABB(middle, parent_aabb.max), child_index_rt, parent_depth + 1);
    
    for (idx_t child_idx = 0; child_idx < max_subdivision_count; child_idx++)
    {
        idx_t child_data_idx = cell_data[parent_idx].children[child_idx]; // access child data index from within the loop, because this loop is changing the data in cell_data!
        if (child_data_idx < 0)
        {
            break;
        }
        createTree(cell_data[child_data_idx]);
    }
}

void SquareSubdiv::setVolume(Cell& sub_tree_root)
{
    Point aabb_size = sub_tree_root.elem.size();
    sub_tree_root.volume = INT2MM(aabb_size.X) * INT2MM(aabb_size.Y);

    bool has_children = sub_tree_root.children[0] >= 0;
    if (has_children)
    {
        for (idx_t child_idx : sub_tree_root.children)
        {
            if (child_idx == -1)
            {
                break;
            }
            assert(child_idx > 0);
            assert(child_idx < static_cast<idx_t>(cell_data.size()));
            setVolume(cell_data[child_idx]);
        }
    }
}

/*
 * Tree creation /\                         .
 * 
 * =======================================================
 * 
 * Lower bound sequence \/                  .
 */

float SquareSubdiv::getActualizedVolume(const Cell& node) const
{
    const Point node_size = node.elem.size();
    const coord_t line_length = (node_size.X + node_size.Y) / (space_filling_curve? 2 : 1);
    return INT2MM(line_width) * INT2MM(line_length);
}

bool SquareSubdiv::isNextTo(const Cell& a, const Cell& b, Direction side) const
{
    AABB a_square_expanded = a.elem;
    a_square_expanded.expand(10); // TODO: move allowed error to central place?
    if (!a_square_expanded.hit(b.elem))
    {
        return false; // cells are apart:  []     []
    }
    bool dimension = side == Direction::UP || side == Direction::DOWN;
    Range<coord_t> a_range(getCoord(a.elem.min, !dimension), getCoord(a.elem.max, !dimension));
    Range<coord_t> b_range(getCoord(b.elem.min, !dimension), getCoord(b.elem.max, !dimension));
    // contract to prevent
    // []
    //   []   from counting as being next to each other
    return a_range.expanded(-10).overlap(b_range); // TODO: move allowed error to central place?
}


/*
 * Tree creation /\                         .
 * 
 * =======================================================
 * 
 * Error redistribution \/                  .
 */


/*
 * Error redistribution /\                         .
 * 
 * =======================================================
 * 
 * Output \/                  .
 */

Polygon SquareSubdiv::createMooreLine() const
{
    std::vector<const Cell*> pattern = createMoorePattern();
    Polygon ret;
    for (const Cell* cell : pattern)
    {
        ret.add(cell->elem.getMiddle());
    }
    return ret;
}

std::vector<const SquareSubdiv::Cell*> SquareSubdiv::createMoorePattern() const
{
    std::vector<const Cell*> ret;
    
    ret.reserve(std::pow(4, max_depth));
    
    createMoorePattern(cell_data[0], ret, 0, 1);
    
    return ret;
}

void SquareSubdiv::createMoorePattern(const Cell& sub_tree_root, std::vector<const Cell*>& pattern, uint_fast8_t starting_child, int_fast8_t winding_dir) const
{
    if (sub_tree_root.is_subdivided)
    {
        /* Basic order of Moore curve for first iteration:
            * 1 2
            * 0 3
            */
        constexpr ChildSide child_nr_to_child_side[] { ChildSide::LEFT_BOTTOM, ChildSide::LEFT_TOP, ChildSide::RIGHT_TOP, ChildSide::RIGHT_BOTTOM };
        /*  child_winding_dirs  and child_starting_child
            *     1---2···1---2
            *     |CW     * CW|
            *     0---3*  0---3
            *         :   :
            *     1---2  *1---2
            *     | CW      CW|
            *     0---3*  0---3
            */
        constexpr int_fast8_t child_winding_dirs[] { 1, 1, 1, 1 };
        constexpr int_fast8_t child_starting_child[] { 3, 3, 1, 1 };
        for (uint_fast8_t child_idx_offset = 0; child_idx_offset < 4; child_idx_offset++)
        {
            uint8_t child_idx = (starting_child + winding_dir * child_idx_offset + 4 ) % 4;
            idx_t global_child_idx = sub_tree_root.children[static_cast<int_fast8_t>(child_nr_to_child_side[child_idx])];
            assert(global_child_idx >= 0);
            createHilbertPattern(cell_data[global_child_idx], pattern, (starting_child + winding_dir * child_starting_child[child_idx_offset] + 4 ) % 4, winding_dir * child_winding_dirs[child_idx_offset]);
        }
    }
    else
    {
        pattern.push_back(&sub_tree_root);
    }
}


void SquareSubdiv::createHilbertPattern(const Cell& sub_tree_root, std::vector<const Cell*>& pattern, uint_fast8_t starting_child, int_fast8_t winding_dir) const
{
    if (sub_tree_root.is_subdivided)
    {
        /* Basic order of Hilbert curve for first iteration:
            * 1 2
            * 0 3
            */
        constexpr ChildSide child_nr_to_child_side[] { ChildSide::LEFT_BOTTOM, ChildSide::LEFT_TOP, ChildSide::RIGHT_TOP, ChildSide::RIGHT_BOTTOM };
        /*  child_winding_dirs         child_starting_child
            *     +---+   +---+              1---2   1---2
            *     |CW |   | CW|              |   |   |   |
            *     +   +···+   +             *0   3···0*  3
            *     :           :              :           :
            *     +---+   +---+              1---2   1---2*
            *      CCW|   |CCW                   |   |
            *     +---+   +---+             *0---3   0---3
            */
        constexpr int_fast8_t child_winding_dirs[] { -1, 1, 1, -1 };
        constexpr int_fast8_t child_starting_child[] { 0, 0, 0, 2 };
        for (uint_fast8_t child_idx_offset = 0; child_idx_offset < 4; child_idx_offset++)
        {
            uint8_t child_idx = (starting_child + winding_dir * child_idx_offset + 4 ) % 4;
            idx_t global_child_idx = sub_tree_root.children[static_cast<int_fast8_t>(child_nr_to_child_side[child_idx])];
            assert(global_child_idx >= 0 );
            createHilbertPattern(cell_data[global_child_idx], pattern, (starting_child + winding_dir * child_starting_child[child_idx_offset] + 4 ) % 4, winding_dir * child_winding_dirs[child_idx_offset]);
        }
    }
    else
    {
        pattern.push_back(&sub_tree_root);
    }
}

/*
 * Output /\                         .
 * 
 * =======================================================
 * 
 * Debug \/                  .
 */

void SquareSubdiv::debugCheckChildrenOverlap(const Cell& cell) const
{
    for (idx_t child_idx : cell.children)
    {
        if (child_idx < 0) continue;
        const Cell& child = cell_data[child_idx];
        assert(cell.elem.intersect(child.elem).isPositive());
        // TODO:
//         for (size_t side = 0; side < 4; side++)
//         { // before and after
//             for (const Link& link : child.adjacent_cells[side])
//             {
//                 const Cell& neighbor = cell_data[link.to_index];
//                 assert(child.elem.z_range.overlap(neighbor.elem.z_range));
//             }
//         }
    }
}


void SquareSubdiv::debugOutput(SVG& svg, float drawing_line_width, bool draw_arrows) const
{
    assert(!cell_data.empty());
    const Cell& root = cell_data[0];
    svg.writePolygon(root.elem.toPolygon(), SVG::Color::BLACK, drawing_line_width);
    debugOutput(svg, root, drawing_line_width, draw_arrows);
}

void SquareSubdiv::debugOutput(SVG& svg, const Cell& sub_tree_root, float drawing_line_width, bool draw_arrows) const
{
    if (sub_tree_root.is_subdivided)
    {
        for (idx_t child_idx : sub_tree_root.children)
        {
            assert(child_idx >= 0);
            debugOutput(svg, cell_data[child_idx], drawing_line_width, draw_arrows);
        }
    }
    else
    {
        AABB square = sub_tree_root.elem;
        svg.writeLine(square.min, Point(square.max.X, square.min.Y), SVG::Color::BLACK, drawing_line_width);
        svg.writeLine(square.max, Point(square.max.X, square.min.Y), SVG::Color::BLACK, drawing_line_width);
        
        const Point from_middle = sub_tree_root.elem.getMiddle();
        
        // draw links
        if (draw_arrows)
        {
            for (const std::list<Link>& side : sub_tree_root.adjacent_cells)
            {
                for (const Link& link : side)
                {
                    const Point to_middle = cell_data[link.to_index].elem.getMiddle();
                    const Point link_vector = to_middle - from_middle;
                    Point arrow_from = from_middle + link_vector / 5 * 1 - turn90CCW(link_vector / 20);
                    Point arrow_to = from_middle + link_vector / 5 * 4 - turn90CCW(link_vector / 20);
                    
                    Point arrow_head_back_l = arrow_to - link_vector / 15 + turn90CCW(link_vector / 30);
                    Point arrow_head_back_r = arrow_to - link_vector / 15 - turn90CCW(link_vector / 30);
                    svg.writeLine(arrow_from, arrow_to, SVG::Color::BLUE);
                    svg.writeLine(arrow_to, arrow_head_back_l, SVG::Color::BLUE);
                    svg.writeLine(arrow_to, arrow_head_back_r, SVG::Color::BLUE);
                    svg.writeLine(arrow_head_back_l, arrow_head_back_r, SVG::Color::BLUE);
                    
                    std::ostringstream os;
                    os << PrecisionedDouble{ 2, link.loan};
                    svg.writeText((arrow_from + arrow_to) / 2 + turn90CCW(link_vector / 10) + link_vector / 10, os.str(), SVG::Color::BLACK, 4);
                }
            }
        }
    }
}

void SquareSubdiv::debugOutputCell(const Cell& cell, SVG& svg, float drawing_line_width, bool horizontal_connections_only) const
{
    debugOutputSquare(cell.elem, svg, drawing_line_width);
    for (int_fast8_t dir = 0; dir < getNumberOfSides(); dir++)
    {
        if (horizontal_connections_only && dir >= static_cast<int_fast8_t>(Direction::DOWN)) break;
        for (const Link& link : cell.adjacent_cells[dir])
        {
            debugOutputLink(link, svg);
        }
    }
}

void SquareSubdiv::debugOutputSquare(const AABB& square, SVG& svg, float drawing_line_width) const
{
    Polygon tri = square.toPolygon();
//     svg.writePoint(triangle.a, true, 1);
//     svg.writePoint(triangle.b, true, 1);
//     svg.writePoint(triangle.straight_corner, true, 1);
//     svg.writePolygon(tri, SVG::Color::GRAY);
    Polygons polys;
    polys.add(tri);
    polys = polys.offset(-80);
//     svg.writeAreas(polys, SVG::Color::GRAY, SVG::Color::BLACK);
    svg.writePolygons(polys, SVG::Color::GRAY);

//     svg.writeLine(triangle.getFromEdge().middle(), triangle.getToEdge().middle(), SVG::Color::RED, drawing_line_width);
}

void SquareSubdiv::debugOutputLink(const Link& link, SVG& svg) const
{
    Point a = cell_data[link.getReverse().to_index].elem.getMiddle();
    Point b = cell_data[link.to_index].elem.getMiddle();
    Point ab = b - a;
    Point shift = normal(turn90CCW(-ab), vSize(ab) / 20);
    coord_t shortening = vSize(ab) / 10;
    // draw arrow body
    Point c = a + shift + normal(ab, shortening);
    Point d = a + shift + normal(ab, vSize(ab) - shortening);
    svg.writeLine(c, d, SVG::Color::BLUE);
    svg.writePoint(c, false, 3, SVG::Color::BLUE);
}

void SquareSubdiv::debugOutputTree(SVG& svg, float drawing_line_width) const
{
    for (const Cell& cell : cell_data)
    {
        debugOutputSquare(cell.elem, svg, drawing_line_width);
    }
}

void SquareSubdiv::debugOutputSequence(SVG& svg, float drawing_line_width) const
{
    debugOutputSequence(cell_data[0], svg, drawing_line_width);
}

void SquareSubdiv::debugOutputSequence(const Cell& cell, SVG& svg, float drawing_line_width) const
{
    if (cell.is_subdivided)
    {
        for (idx_t child_idx : cell.children)
        {
            if (child_idx > 0)
            {
                debugOutputSequence(cell_data[child_idx], svg, drawing_line_width);
            }
        }
    }
    else
    {
        debugOutputCell(cell, svg, drawing_line_width, false);
    }
}

}; // namespace cura
