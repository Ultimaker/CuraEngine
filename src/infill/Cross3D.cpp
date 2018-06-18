/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#include "Cross3D.h"

#include <cassert>
#include <sstream>  // debug TODO

#include "../utils/math.h"
#include "../utils/linearAlg2D.h"
#include "../utils/gettime.h"


namespace cura {

Cross3D::Cross3D(const DensityProvider& density_provider, const AABB3D aabb, const int max_depth, coord_t line_width)
: InfillFractal2D<Cross3DPrism>(density_provider, aabb, max_depth, line_width)
, min_dist_to_cell_bound(line_width / 2)
, min_dist_to_cell_bound_diag(line_width * 0.5 * sqrt2)
{
}


float Cross3D::getDensity(const Cell& cell) const
{
    AABB aabb;
    aabb.include(cell.elem.triangle.straight_corner);
    aabb.include(cell.elem.triangle.a);
    aabb.include(cell.elem.triangle.b);
    AABB3D aabb3d(Point3(aabb.min.X, aabb.min.Y, cell.elem.z_range.min), Point3(aabb.max.X, aabb.max.Y, cell.elem.z_range.max));
    return density_provider(aabb3d);
}



void Cross3D::createTree()
{
    assert(cell_data.empty());
    size_t to_be_reserved = 1.05 * sqrt2 * (2 << (max_depth * 3 / 2)); // magic formula predicting nr of cells in the tree. Overestimates to prevent reallocation.
    logError("Cross3D reserved %i nodes\n", to_be_reserved);
    cell_data.reserve(to_be_reserved);
    Prism root_prism; // initialized with invalid data
    cell_data.emplace_back(root_prism, /*index =*/ 0, /* depth =*/ 0);

    // old: start with 2 half cubes and make a non-closed polyline
//     Triangle first_triangle(Point(aabb2d.min.X, aabb2d.max.Y), aabb2d.min, aabb2d.max, Triangle::Direction::AC_TO_AB, /* straight_corner_is_left =*/ false);
//     Triangle second_triangle(Point(aabb2d.max.X, aabb2d.min.Y), aabb2d.max, aabb2d.min, Triangle::Direction::AB_TO_BC, /* straight_corner_is_left =*/ false);
//     cell_data[0].children[2] = -1;
//     cell_data[0].children[3] = -1;

    // new: start with 4 quarter cubes so as to form a closed sierpinski curve
    AABB aabb2d = aabb.flatten();
    Point middle = aabb2d.getMiddle();
    Triangle triangle_0(middle, aabb2d.min, Point(aabb2d.min.X, aabb2d.max.Y), Triangle::Direction::AC_TO_BC, /* straight_corner_is_left =*/ false);
    createTree(triangle_0, 2);

    Triangle triangle_1(middle, Point(aabb2d.min.X, aabb2d.max.Y), aabb2d.max, Triangle::Direction::AC_TO_BC, /* straight_corner_is_left =*/ false);
    createTree(triangle_1, 3);

    Triangle triangle_2(middle, aabb2d.max, Point(aabb2d.max.X, aabb2d.min.Y), Triangle::Direction::AC_TO_BC, /* straight_corner_is_left =*/ false);
    createTree(triangle_2, 0);

    Triangle triangle_3(middle, Point(aabb2d.max.X, aabb2d.min.Y), aabb2d.min, Triangle::Direction::AC_TO_BC, /* straight_corner_is_left =*/ false);
    createTree(triangle_3, 1);

    Cell& root = cell_data[0];
    Point3 aabb_size = aabb.max - aabb.min;
    root.volume = INT2MM(aabb_size.x) * INT2MM(aabb_size.y) * INT2MM(aabb_size.z);
    root.is_subdivided = true;

    initialConnection(cell_data[root.children[0]], cell_data[root.children[1]], Direction::RIGHT);
    initialConnection(cell_data[root.children[1]], cell_data[root.children[2]], Direction::RIGHT);
    initialConnection(cell_data[root.children[2]], cell_data[root.children[3]], Direction::RIGHT);
    initialConnection(cell_data[root.children[3]], cell_data[root.children[0]], Direction::RIGHT);
    
    setSpecificationAllowance(cell_data[0]);
}

void Cross3D::createTree(const Triangle& triangle, size_t root_child_number)
{
    Prism prism(triangle, aabb.min.z, aabb.max.z, /* is_expanding =*/ true);
    idx_t child_index = cell_data.size();
    cell_data[0].children[root_child_number] = child_index;
    cell_data.emplace_back(prism, child_index, /* depth =*/ 1);
    createTree(cell_data.back(), max_depth);
    setVolume(cell_data[child_index]);
}
void Cross3D::createTree(Cell& sub_tree_root, int max_depth)
{
    int parent_depth = sub_tree_root.depth;
    if (parent_depth >= max_depth)
    {
        for (uint_fast8_t child_idx = 0; child_idx < 4; child_idx++)
        {
            sub_tree_root.children[child_idx] = -1;
        }
        return;
    }
    
    // At each subdivision we divide the triangle in two.
    const Prism parent_prism = sub_tree_root.elem;
    std::array<Triangle, 2> subdivided_triangles = parent_prism.triangle.subdivide();

    idx_t parent_idx = sub_tree_root.index; // use index below because we are changing the data vector, so sub_tree_root might get invalidated!

    uint_fast8_t child_count = parent_prism.isHalfCube() ? 2 : 4;
    assert(child_count == 2 || child_count == 4);
    coord_t child_z_min = parent_prism.z_range.min;
    coord_t child_z_max = (child_count == 2)? parent_prism.z_range.max : (parent_prism.z_range.max + parent_prism.z_range.min) / 2;
    for (uint_fast8_t child_z_idx = 0; child_z_idx < 2; child_z_idx++)
    { // only do a second iteration if there are 4 children
        for (uint_fast8_t child_xy_idx = 0; child_xy_idx < 2; child_xy_idx++)
        {
            idx_t child_idx = child_z_idx * 2 + child_xy_idx;
            if (child_z_idx == child_count / 2)
            {
                cell_data[parent_idx].children[child_idx] = -1; // change parent 
                continue;
            }
            const bool is_expanding = (child_count == 4)? static_cast<bool>(child_z_idx) : parent_prism.is_expanding;
            idx_t child_data_index = cell_data.size();
            cell_data[parent_idx].children[child_idx] = child_data_index; // change parent 
            cell_data.emplace_back(Prism(subdivided_triangles[child_xy_idx], child_z_min, child_z_max, is_expanding), child_data_index, parent_depth + 1);
            createTree(cell_data.back(), max_depth);
        }
        // update z range for the upper children
        child_z_min = child_z_max; // middle of parent z range
        child_z_max = parent_prism.z_range.max;
    }
}

void Cross3D::setVolume(Cell& sub_tree_root)
{
    Triangle& parent_triangle = sub_tree_root.elem.triangle;
    Point ac = parent_triangle.straight_corner - parent_triangle.a;
    float area = 0.5 * INT2MM2(vSize2(ac));
    sub_tree_root.volume = area * INT2MM(sub_tree_root.elem.z_range.max - sub_tree_root.elem.z_range.min);

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

void Cross3D::sanitize()
{
    sanitize(cell_data[0]);
}

void Cross3D::sanitize(Cell& sub_tree_root)
{
    if (sub_tree_root.is_subdivided)
    { // recurse
        for (idx_t child_idx : sub_tree_root.children)
        {
            if (child_idx < 0)
            {
                break;
            }
            sanitize(cell_data[child_idx]);
        }
    }
    else
    {
        if (sub_tree_root.elem.triangle.dir == Triangle::Direction::AC_TO_BC
            && sub_tree_root.elem.isQuarterCube())
        {
            uint_fast8_t child_count = 0;
            uint_fast8_t deeper_child_count = 0;
            for (const std::list<Link>& side : sub_tree_root.adjacent_cells)
            {
                for (const Link& link : side)
                {
                    child_count++;
                    const char child_depth = cell_data[link.to_index].depth;
                    if (child_depth > sub_tree_root.depth)
                    {
                        deeper_child_count++;
                    }
                    else if (child_depth < sub_tree_root.depth)
                    {
                        return; // The cell has neighbors with lower recursion depth, so the subdivision of this node is constrained.
                    }
                    break; // other child on this side has to have the same depth!
                }
            }
            if (deeper_child_count >= child_count / 2)
            {
                constexpr bool redistribute_errors = true;
                subdivide(sub_tree_root, redistribute_errors);
            }
        }
    }
}

float Cross3D::getActualizedVolume(const Cell& node) const
{
    const Triangle& triangle = node.elem.triangle;
    Point from_middle, to_middle;
    Point ac_middle = (triangle.a + triangle.straight_corner) / 2;
    Point bc_middle = (triangle.b + triangle.straight_corner) / 2;
    Point ab_middle = (triangle.a + triangle.b) / 2;
    switch(triangle.dir)
    {
        case Triangle::Direction::AC_TO_AB:
            from_middle = ac_middle;
            to_middle = ab_middle;
            break;
        case Triangle::Direction::AC_TO_BC:
            from_middle = ac_middle;
            to_middle = bc_middle;
            break;
        case Triangle::Direction::AB_TO_BC:
            from_middle = ab_middle;
            to_middle = bc_middle;
            break;
    }
    return INT2MM(line_width) * INT2MM(vSize(from_middle - to_middle)) * INT2MM(node.elem.z_range.max - node.elem.z_range.min);
}

bool Cross3D::isNextTo(const Cell& a, const Cell& b, Direction side) const
{
    LineSegment a_edge, b_edge;
    switch (side)
    {
        case Direction::UP:
        case Direction::DOWN:
        {
            // check if z ranges touch (or overlap)
            if (!a.elem.z_range.overlap(b.elem.z_range.expanded(10)))
            {
                return false;
            }
            // check if triangle areas overlap
            Polygon a_polygon = a.elem.triangle.toPolygon();
            double a_area = a_polygon.area();
            Polygon b_polygon = b.elem.triangle.toPolygon();
            double b_area = b_polygon.area();
            Polygons intersection = a_polygon.intersection(b_polygon);
            double intersection_area = intersection.area();
            bool triangles_overlap = std::abs(intersection_area - std::min(a_area, b_area)) < 100.0; // TODO magic number
            return triangles_overlap;
        }
        case Direction::LEFT:
            a_edge = a.elem.triangle.getFromEdge();
            b_edge = b.elem.triangle.getToEdge();
            break;
        case Direction::RIGHT:
            a_edge = a.elem.triangle.getToEdge();
            b_edge = b.elem.triangle.getFromEdge();
            break;
        default:
            logError("Unknown direction passed to Cross3D::isNextTo!\n");
    }
    if (!a.elem.z_range.inside(b.elem.z_range.middle()) && !b.elem.z_range.inside(a.elem.z_range.middle()))
    { // they are not next to each other in z ranges
        return false;
    }
    if (!LinearAlg2D::areCollinear(a_edge, b_edge))
    {
        return false;
    }
    Point a_vec = a_edge.getVector();
    coord_t a_size = vSize(a_vec);
    assert(a_size > 0);
    Range<coord_t> a_edge_projected(0, a_size);
    Range<coord_t> b_edge_projected;
    b_edge_projected.include(dot(b_edge.from - a_edge.from, a_vec) / a_size);
    b_edge_projected.include(dot(b_edge.to - a_edge.from, a_vec) / a_size);
    return a_edge_projected.intersection(b_edge_projected).size() > 10;
}


/*
 * Tree creation /\                         .
 * 
 * =======================================================
 * 
 * Error redistribution \/                  .
 */

float Cross3D::getValueError(const Cell& cell) const
{
    float balance = cell.filled_volume_allowance - getActualizedVolume(cell) + getTotalLoanError(cell);
    return balance;
}

float Cross3D::getSubdivisionError(const Cell& cell) const
{
    float balance = cell.filled_volume_allowance - getChildrenActualizedVolume(cell) + getTotalLoanError(cell);
    return balance;
}

float Cross3D::getTotalLoanError(const Cell& cell) const
{
    float loan = 0.0;

    for (const std::list<Link>& neighbors_in_a_given_direction : cell.adjacent_cells)
    {
        for (const Link& link : neighbors_in_a_given_direction)
        {
            loan -= link.loan;
            loan += link.getReverse().loan;
        }
    }
    return loan;
}

float Cross3D::getTotalLoanObtained(const Cell& cell) const
{
    float loan = 0.0;

    for (const std::list<Link>& neighbors_in_a_given_direction : cell.adjacent_cells)
    {
        for (const Link& link : neighbors_in_a_given_direction)
        {
            const float loan_here = link.getReverse().loan;
            loan += loan_here;
            assert(std::abs(loan_here) < allowed_volume_error || std::abs(link.loan) < allowed_volume_error);
        }
    }
    return loan;
}

void Cross3D::transferLoans(Link& old, const std::list<Link*>& new_links)
{
    if (old.loan == 0.0)
    {
        return;
    }
    // TODO
    // this implements naive equal transfer
    int new_link_count = new_links.size();
    for (Link* link : new_links)
    {
        link->loan = old.loan / new_link_count;
    }
}



void Cross3D::distributeLeftOvers(Cell& from, float left_overs)
{
//     from.loan_balance -= left_overs;
    std::vector<Link*> loaners;
    float total_loan = 0.0;
    for (std::list<Link>& neighbors_in_a_given_direction : from.adjacent_cells)
    {
        for (Link& link : neighbors_in_a_given_direction)
        {
            if (link.getReverse().loan > allowed_volume_error)
            {
                total_loan += link.getReverse().loan;
                loaners.push_back(&link.getReverse());
            }
        }
    }
    assert(left_overs < total_loan * 1.00001 && "There cannot be more left over than what was initially loaned");
    for (Link* loaner : loaners)
    {
        float pay_back = loaner->loan * left_overs / total_loan;
        loaner->loan -= pay_back;
//         loaner->from()->loan_balance += pay_back;
    }
}

/*
 * Error redistribution /\                         .
 * 
 * =======================================================
 * 
 * Output \/                  .
 */

Cross3D::SliceWalker Cross3D::getSequence(coord_t z) const
{
    SliceWalker ret;
    // get first cell
    const Cell* last_cell = &cell_data[0];
    while (last_cell->is_subdivided)
    {
        const Cell& left_bottom_child = cell_data[last_cell->children[0]];
        if (last_cell->getChildCount() == 2 || left_bottom_child.elem.z_range.inside(z))
        {
            last_cell = &left_bottom_child;
        }
        else
        {
            const Cell& left_top_child = cell_data[last_cell->children[2]];
            assert(left_top_child.elem.z_range.inside(z));
            last_cell = &left_top_child;
        }
    }
    const Cell& start_cell = *last_cell;

    ret.layer_sequence.push_back(last_cell);
    while (!last_cell->adjacent_cells[static_cast<size_t>(Direction::RIGHT)].empty())
    {
        const Cell& bottom_neighbor = cell_data[last_cell->adjacent_cells[static_cast<size_t>(Direction::RIGHT)].front().to_index];
        const Cell& top_neighbor = cell_data[last_cell->adjacent_cells[static_cast<size_t>(Direction::RIGHT)].back().to_index];
        if (bottom_neighbor.elem.z_range.inside(z))
        {
            last_cell = &bottom_neighbor;
        }
        else
        {
            assert(top_neighbor.elem.z_range.inside(z) && "we assume a cell has max two neighbors in any given direction and either of them must overlap with the required z!");
            last_cell = &top_neighbor;
        }
        if (last_cell == &start_cell)
        {
            break;
        }
        ret.layer_sequence.push_back(last_cell);
    }
    debugCheckHeights(ret, z);
    {
        SVG svg("output/bottom_sequence.svg", aabb.flatten());
        debugOutput(ret, svg, 1);
    }
    return ret;
}

void Cross3D::advanceSequence(SliceWalker& walker, coord_t new_z) const
{
    std::list<const Cell*>& sequence = walker.layer_sequence;
    bool new_z_is_beyond_current = true;
    while (new_z_is_beyond_current)
    {
        // replace all cells which have become too low with their upstairs neighbor
        // untill the new z is met
        using iter_t = std::list<const Cell*>::iterator;
        for (iter_t iter = sequence.begin(); iter != sequence.end(); ++iter)
        {
            const Cell& cell = **iter;
            if (cell.elem.z_range.max < new_z)
            { // we have to replace this cell with the upstairs neighbors
                idx_t cell_before_idx = -1;
                if (iter != sequence.begin())
                {
                    cell_before_idx = (*std::prev(iter))->index;
                }
                idx_t cell_after_idx = -1;
                if (std::next(iter) != sequence.end())
                {
                    cell_after_idx = (*std::next(iter))->index;
                }

                const std::list<Link>& neighbors_above = cell.adjacent_cells[static_cast<size_t>(Direction::UP)];
                assert(!neighbors_above.empty());
                for (const Link& neighbor_above : neighbors_above)
                { // add cells that weren't added yet
                    // cells might have already been added because of the advancement of the previous cell its upstairs neigbors
                    // two consecutive (left-right) cells might share the same upstairs neighbor
                    if (neighbor_above.to_index != cell_before_idx && neighbor_above.to_index != cell_after_idx)
                    {
                        sequence.insert(iter, &cell_data[neighbor_above.to_index]);
                    }
                }
                iter_t iter_after = std::next(iter);
                sequence.erase(iter);
                iter = std::prev(iter_after);
                assert(iter_after == sequence.end() || (*iter_after)->index != (*iter)->index);
            }
        }

        new_z_is_beyond_current = false;
        for (iter_t iter = sequence.begin(); iter != sequence.end(); ++iter)
        {
            const Cell& cell = **iter;
            if (cell.elem.z_range.max < new_z)
            { // apparently we haven't moved up in the sequence by enough distance.
                new_z_is_beyond_current = true;
                logWarning("Layers seem to be higher than prisms in the Cross3D pattern! The fidelity of the Cross3D pattern is too high or something else is wrong.\n");
                break;
            }
        }
    }

    debugCheckHeights(walker, new_z);
}

Polygon Cross3D::generateSierpinski(const SliceWalker& walker) const
{
    Polygon poly;
    for (const Cell* cell : walker.layer_sequence)
    {
        poly.add(cell->elem.triangle.getMiddle());
    }
    return poly;
}

Polygon Cross3D::generateCross(const SliceWalker& walker, coord_t z) const
{
    Polygon poly;

    assert(walker.layer_sequence.size() >= 4); // the dummy root cell should always be subdivided into its 4 children
    const Cell* before = *std::prev(std::prev(walker.layer_sequence.end()));
    const Cell* here = walker.layer_sequence.back();
    
    Point from = getCellEdgeLocation(*before, *here, z);
    for (const Cell* after: walker.layer_sequence)
    {
        assert(here->index != after->index);
        sliceCell(*here, *after, z, from, poly);
        here = after;
    }

    return poly;
}

void Cross3D::sliceCell(const Cell& cell, const Cell& after, const coord_t z, Point& from_output, PolygonRef output) const
{
    Point from = from_output;
    Point to = getCellEdgeLocation(cell, after, z);
    from_output = to;

    LineSegment from_edge = cell.elem.triangle.getFromEdge();
    LineSegment to_edge = cell.elem.triangle.getToEdge();

    // from and to should lie ON the segments of the triangle
    assert(LinearAlg2D::getDist2FromLine(from, from_edge.from, from_edge.to) < 100);
    assert(LinearAlg2D::getDist2FromLine(to, to_edge.from, to_edge.to) < 100);

    output.add(from);
    if (vSize2(to - from) > line_width * line_width * 3 / 4) // 1.5 * (.5line_width*sqrt(2))^2
    { // distance is too small to make non-overlapping
        if (isOverlapping(from_edge, LineSegment(from, to)))
        {
            add45degBend(from, to, from_edge.reversed(), output);
        }
        if (isOverlapping(to_edge, LineSegment(to, output.back())))
        {
            add45degBend(to, output.back(), to_edge, output);
        }
    }
    // don't add [to]; it will already be added in the next call to sliceCell(.)
}

Point Cross3D::getCellEdgeLocation(const Cell& before, const Cell& after, const coord_t z) const
{
    /*
     * the edge of a triangle is connected to a side of the prism
     * the oscillation of the space filling curve vertices is determined by the connected prisms
     * there are two cells connected to such an edge in the XY plane
     * and the one with the highest recursion depth determines the basic oscillation frequency.
     * However, the prism sides are also connected to prism sides above and below.
     * The osciallation pattern must connect to cells above and below.
     * This is done by altering the oscillation pattern of the lowest density cell to adjust it such that
     * the lower recursion depth cell attaches to the higher recursion depth cell.
     * 
     * Simplified 2D example:            .
     *  ____              ____           .
     * [-.  ]            [-.  ]          .
     * [__'-]___         [__'-]___       .
     * [      .-]   ==>  [    '.  ]      .
     * [   .-'  ]        [   .-'  ]      .
     * [.-'_____]        [.-'_____]      .
     *                                   .
     * alternative ascii art:            .
     *                                   .
     * ] /[ [          ] /[ [            .
     * ]/ [ [          ]/ [ [            .
     * ]\ [ [          ]\ [ [            .
     * ] \[ [          ] \[ [            .
     * ]   /[ >change> ]  \ [            .
     * ]  / [          ]  / [            .
     * ] /  [          ] /  [            .
     * ]/   [          ]/   [            .
     * ]\   [    ==>   ]\   [            .
     * ] \  [          ] \  [            .
     * ]  \ [          ]  \ [            .
     * ]   \[          ]   \[            .
     * ]   /[          ]   /[            .
     * ]  / [          ]  / [            .
     * ] /  [          ] /  [            .
     * ]/   [          ]/   [            .
     */
    const Cell& densest_cell = (after.depth > before.depth)? after : before;
    const LineSegment edge = (after.depth > before.depth)? after.elem.triangle.getFromEdge() : before.elem.triangle.getToEdge();

    const coord_t edge_size = vSize(edge.getVector());
    coord_t pos = getCellEdgePosition(densest_cell, edge_size, z); // position along the edge where to put the vertex

    // check for constraining cells above or below
    if (z > densest_cell.elem.z_range.middle())
    { // check cell above
        applyZOscillationConstraint(before, after, z, densest_cell, edge, edge_size, Direction::UP, pos);
    }
    else
    { // check cell below
        applyZOscillationConstraint(before, after, z, densest_cell, edge, edge_size, Direction::DOWN, pos);
    }

    { // Keep lines away from cell boundary to prevent line overlap
        coord_t from_min_dist = (edge.from == densest_cell.elem.triangle.straight_corner)? min_dist_to_cell_bound : min_dist_to_cell_bound_diag;
        coord_t to_min_dist = (edge.to == densest_cell.elem.triangle.straight_corner)? min_dist_to_cell_bound : min_dist_to_cell_bound_diag;
        pos  = std::min(edge_size - to_min_dist, std::max(from_min_dist, pos));
        if (pos < from_min_dist)
        { // edge size is smaller than a line width
            pos = edge_size / 2;
        }
        assert(pos >= 0);
        assert(pos <= edge_size);
    }

    Point ret = edge.from + normal(edge.getVector(), pos);
    assert(aabb.flatten().contains(ret));

    LineSegment edge1 = before.elem.triangle.getToEdge();
    LineSegment edge2 = after.elem.triangle.getFromEdge();
    assert(LinearAlg2D::getDist2FromLine(ret, edge1.from, edge1.to) < 100);
    assert(LinearAlg2D::getDist2FromLine(ret, edge2.from, edge2.to) < 100);
    return ret;
}

void Cross3D::applyZOscillationConstraint(const Cell& before, const Cell& after, coord_t z, const Cell& densest_cell, const LineSegment edge, const coord_t edge_size, const Direction checking_direction, coord_t& pos) const
{
    const bool checking_up = checking_direction == Direction::UP;
    const coord_t flip_for_down = checking_up? 1 : -1;

    // documentation and naming in this function assumes it has been called for checking the edge for constraining from above
    const Cell* densest_cell_above = nullptr; // there might be no cell above
    LineSegment edge_above;
    { // find densest cell above and the corresponding edge
        const std::list<Link>& before_neighbors_above = before.adjacent_cells[static_cast<size_t>(checking_direction)];
        if (!before_neighbors_above.empty())
        {
            densest_cell_above = &cell_data[before_neighbors_above.back().to_index];
            edge_above = densest_cell_above->elem.triangle.getToEdge();
        }
        const std::list<Link>& after_neighbors_above = after.adjacent_cells[static_cast<size_t>(checking_direction)];
        if (!after_neighbors_above.empty())
        {
            const Cell& after_above = cell_data[after_neighbors_above.front().to_index];
            if (!densest_cell_above || after_above.depth > densest_cell_above->depth)
            {
                densest_cell_above = &after_above;
                edge_above = after_above.elem.triangle.getFromEdge();
            }
        }
    }

    /* TODO:
     * Don't assume the oscillation position in the cell above is either left middle or center!
     * This is causing problems for a specific case where a recursion depth change happens across Z ad across XY
     * the proper oscillation position is somewhere at 2/3rd
     * This is causing overhang problems.
     *       /|\               /|\                                                                          .
     *      / | \             / | \                                                                         .
     *     /  |  \           /  |  \                                                                        .
     *    /   |   \  ==>    /   |   \                                                                       .
     *   /___↗|→→→↘\       /    |    \                                                                      .
     *   \   ↑|    ↘\      \   ↗|↘    \                                                                     .
     *    \  ↑|   / ↘\      \  ↑|  ↘   \                                                                    .
     *     \ ↑|  /   ↘\      \ ↑|    ↘  \                                                                   .
     *      \↑| /     ↘\      \↑|      ↘ \                                                                  .
     *       \|/_______↘\      \|________↘\                                                                 .
     */
    if (densest_cell_above && densest_cell_above->depth > densest_cell.depth)
    { // this cells oscillation pattern is altered to fit the oscillation pattern above
        const Point oscillation_end_point = // where the oscillation should end on the edge at this height
            (densest_cell_above->elem.is_expanding == checking_up) // flip for downward direction
            ? edge_above.from
            : edge_above.to;
        const coord_t oscillation_end_pos = dot(oscillation_end_point - edge.from, edge.to - edge.from) / vSize(edge.getVector()); // end position along the edge at this height
        assert(std::abs(oscillation_end_pos - vSize(oscillation_end_point - edge.from)) < 10 && "oscillation_end_point should lie on the segment!");
        assert(oscillation_end_pos >= -10 && oscillation_end_pos <= edge_size + 10);
        if (oscillation_end_pos > edge_size / 4 && oscillation_end_pos < edge_size * 3 / 4)
        { // oscillation end pos is in the middle
            if (z * flip_for_down > flip_for_down * (densest_cell.elem.z_range.middle() + flip_for_down * densest_cell.elem.z_range.size() / 4))
            // z is more than 3/4 for up or z less than 1/4 for down
            { // we're in the top quarter z of this prism
                if (densest_cell.elem.is_expanding == checking_up) // flip when cheking down
                {
                    pos = edge_size * 3 / 2 - pos;
                }
                else
                {
                    pos = edge_size / 2 - pos;
                }
            }
        }
        else
        { // oscillation end pos is at one of the ends
            if ((oscillation_end_pos > edge_size / 2) == (densest_cell.elem.is_expanding == checking_up)) // flip on is_expanding and on checking_up
            { // constraining cell above is constraining this edge to be the same is it would normally be
                // don't alter pos
            }
            else
            { // constraining cell causes upper half of oscillation pattern to be inverted
                pos = edge_size - pos;
            }
        }
    }
}


coord_t Cross3D::getCellEdgePosition(const Cell& cell, const coord_t edge_size, coord_t z) const
{
    coord_t pos = (z - cell.elem.z_range.min) * edge_size / cell.elem.z_range.size();
    if (!cell.elem.is_expanding)
    {
        pos = edge_size - pos;
    }
    assert(pos >= 0);
    assert(pos <= edge_size);
    return pos;
}


bool Cross3D::isOverlapping(const LineSegment edge, const LineSegment segment) const
{
    assert(LinearAlg2D::getDist2FromLine(segment.from, edge.from, edge.to) < 100);
    Point a = edge.getVector();
    Point b = segment.getVector();
    coord_t prod = std::abs(dot(a, b));
    return prod > vSize(a) * vSize(b) * 0.5 * sqrt2;
}

void Cross3D::add45degBend(const Point end_point, const Point other_end_point, const LineSegment edge, PolygonRef output) const
{
    Point insetted = end_point + normal(turn90CCW(edge.getVector()), line_width / 2);
    Point along_edge = edge.from - end_point;
    coord_t dir = (dot(other_end_point - end_point, along_edge) > 0)? 1 : -1;
    Point offsetted = insetted + normal(along_edge, dir * line_width / 2); // make inward piece angled toward the other_end_point with 45*
    output.add(offsetted);
}

/*
 * Output /\                         .
 * 
 * =======================================================
 * 
 * Debug \/                  .
 */


void Cross3D::debugCheckHeights(const SliceWalker& sequence, coord_t z) const
{
    const Cell* prev = sequence.layer_sequence.back();
    for (const Cell* cell : sequence.layer_sequence)
    {
        assert(cell->elem.z_range.inside(z));
        assert(cell->index != prev->index);
        prev = cell;
    }
}

void Cross3D::debugCheckChildrenOverlap(const Cell& cell) const
{
    if (cell.depth > 0)
    { // ignore properties of dummy cell of root
        for (idx_t child_idx : cell.children)
        {
            if (child_idx < 0) continue;
            const Cell& child = cell_data[child_idx];
            assert(cell.elem.z_range.inside(child.elem.z_range.middle()));
            for (size_t side = 0; side < 2; side++)
            { // before and after
                for (const Link& link : child.adjacent_cells[side])
                {
                    const Cell& neighbor = cell_data[link.to_index];
                    assert(child.elem.z_range.overlap(neighbor.elem.z_range));
                }
            }
        }
    }
}

void Cross3D::debugOutputCell(const Cell& cell, SVG& svg, float drawing_line_width, bool horizontal_connections_only) const
{
    debugOutputTriangle(cell.elem.triangle, svg, drawing_line_width);
    for (int_fast8_t dir = 0; dir < getNumberOfSides(); dir++)
    {
        if (horizontal_connections_only && dir >= static_cast<int_fast8_t>(Direction::DOWN)) break;
        for (const Link& link : cell.adjacent_cells[dir])
        {
            debugOutputLink(link, svg);
        }
    }
}

void Cross3D::debugOutputTriangle(const Triangle& triangle, SVG& svg, float drawing_line_width) const
{
    Polygon tri;
    tri.add(triangle.a);
    tri.add(triangle.b);
    tri.add(triangle.straight_corner);
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

void Cross3D::debugOutputLink(const Link& link, SVG& svg) const
{
    Point a = cell_data[link.getReverse().to_index].elem.triangle.getMiddle();
    Point b = cell_data[link.to_index].elem.triangle.getMiddle();
    Point ab = b - a;
    Point shift = normal(turn90CCW(-ab), vSize(ab) / 20);
    coord_t shortening = vSize(ab) / 10;
    // draw arrow body
    Point c = a + shift + normal(ab, shortening);
    Point d = a + shift + normal(ab, vSize(ab) - shortening);
    svg.writeLine(c, d, SVG::Color::BLUE);
    svg.writePoint(c, false, 3, SVG::Color::BLUE);
}

void Cross3D::debugOutput(const SliceWalker& walker, SVG& svg, float drawing_line_width) const
{
    for (const Cell* cell : walker.layer_sequence)
    {
//         debugOutputTriangle(cell->prism.triangle, svg, drawing_line_width);
        debugOutputCell(*cell, svg, drawing_line_width, true);
    }
}
void Cross3D::debugOutputTree(SVG& svg, float drawing_line_width) const
{
    for (const Cell& cell : cell_data)
    {
        debugOutputTriangle(cell.elem.triangle, svg, drawing_line_width);
    }
}

void Cross3D::debugOutputSequence(SVG& svg, float drawing_line_width) const
{
    debugOutputSequence(cell_data[0], svg, drawing_line_width);
}

void Cross3D::debugOutputSequence(const Cell& cell, SVG& svg, float drawing_line_width) const
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
