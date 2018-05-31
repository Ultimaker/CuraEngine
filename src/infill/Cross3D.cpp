/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#include "Cross3D.h"

#include <cassert>

#include "../utils/math.h"
#include "../utils/linearAlg2D.h"
#include "../utils/gettime.h"


namespace cura {

Cross3D::Direction Cross3D::opposite(Cross3D::Direction in)
{
    switch(in)
    {
        case Direction::LEFT: return Direction::RIGHT;
        case Direction::RIGHT: return Direction::LEFT;
        case Direction::UP: return Direction::DOWN;
        case Direction::DOWN: return Direction::UP;
        default: return Direction::COUNT;
    }
}

uint_fast8_t Cross3D::opposite(uint_fast8_t in)
{
    return static_cast<uint_fast8_t>(opposite(static_cast<Direction>(in)));
}


LineSegment Cross3D::Triangle::getFromEdge() const
{
    LineSegment ret;
    switch(dir)
    {
        case Direction::AB_TO_BC:
            ret = LineSegment(a, b);
            break;
        case Direction::AC_TO_AB:
            ret = LineSegment(straight_corner, a);
            break;
        case Direction::AC_TO_BC:
            ret = LineSegment(straight_corner, a);
            break;
    }
    if (straight_corner_is_left)
    {
        ret.reverse();
    }
    return ret;
}

LineSegment Cross3D::Triangle::getToEdge() const
{
    LineSegment ret;
    switch(dir)
    {
        case Direction::AB_TO_BC:
            ret = LineSegment(straight_corner, b);
            break;
        case Direction::AC_TO_AB:
            ret = LineSegment(b, a);
            break;
        case Direction::AC_TO_BC:
            ret = LineSegment(straight_corner, b);
            break;
    }
    if (straight_corner_is_left)
    {
        ret.reverse();
    }
    return ret;
}

Point Cross3D::Triangle::getMiddle() const
{
    return (straight_corner + a + b) / 3;
}

Polygon Cross3D::Triangle::toPolygon() const
{
    Polygon ret;
    ret.add(straight_corner);
    Point second = a;
    Point third = b;
    if (!straight_corner_is_left)
    {
        std::swap(second, third);
    }
    ret.add(second);
    ret.add(third);
    assert(ret.area() > 0);
    return ret;
}

std::array<Cross3D::Triangle, 2> Cross3D::Triangle::subdivide() const
{
/*
 * Triangles are subdivided into two children like so:
 * |\       |\        .
 * |A \     |A \      .
 * |    \   |    \    . where C is always the 90* straight corner
 * |     C\ |C____B\  .       The direction between A and B is maintained
 * |      / |C    A/
 * |    /   |    /      Note that the polygon direction flips between clockwise and CCW each subdivision
 * |B /     |B /        as does Triangle::straight_corner_is_left
 * |/       |/
 * 
 * The direction of the space filling curve along each triangle is recorded:
 * 
 * |\                           |\                                        .
 * |B \  AC_TO_BC               |B \   AC_TO_AB                           .
 * |  ↑ \                       |  ↑ \                                    .
 * |  ↑  C\  subdivides into    |C_↑__A\                                  .
 * |  ↑   /                     |C ↑  B/                                  .
 * |  ↑ /                       |  ↑ /                                    .
 * |A /                         |A /   AB_TO_BC                           .
 * |/                           |/                                        .
 *                                                                        .
 * |\                           |\                                        .
 * |B \  AC_TO_AB               |B \   AC_TO_BC                           .
 * |    \                       |↖   \                                    .
 * |↖    C\  subdivides into    |C_↖__A\                                  .
 * |  ↖   /                     |C ↑  B/                                  .
 * |    /                       |  ↑ /                                    .
 * |A /                         |A /   AB_TO_BC                           .
 * |/                           |/                                        .
 *                                                                        .
 * |\                           |\                                        .
 * |B \  AB_TO_BC               |B \   AC_TO_AB                           .
 * |  ↗ \                       |  ↑ \                                    .
 * |↗    C\  subdivides into    |C_↑__A\                                  .
 * |      /                     |C ↗  B/                                  .
 * |    /                       |↗   /                                    .
 * |A /                         |A /   AC_TO_BC                           .
 * |/                           |/                                        .
 */
    std::array<Cross3D::Triangle, 2> ret;
    Point middle = (a + b) / 2;
    ret[0].straight_corner = middle;
    ret[0].a = a;
    ret[0].b = straight_corner;
    ret[0].straight_corner_is_left = !straight_corner_is_left;
    ret[1].straight_corner = middle;
    ret[1].a = straight_corner;
    ret[1].b = b;
    ret[1].straight_corner_is_left = !straight_corner_is_left;
    switch(dir)
    {
        case Triangle::Direction::AB_TO_BC:
            ret[0].dir = Triangle::Direction::AC_TO_BC;
            ret[1].dir = Triangle::Direction::AC_TO_AB;
            break;
        case Triangle::Direction::AC_TO_AB:
            ret[0].dir = Triangle::Direction::AB_TO_BC;
            ret[1].dir = Triangle::Direction::AC_TO_BC;
            break;
        case Triangle::Direction::AC_TO_BC:
            ret[0].dir = Triangle::Direction::AB_TO_BC;
            ret[1].dir = Triangle::Direction::AC_TO_AB;
            break;
    }
    return ret;
}

bool Cross3D::Prism::isHalfCube() const
{
    return std::abs(vSize(triangle.straight_corner - triangle.b) - (z_range.max - z_range.min)) < 10;
}
bool Cross3D::Prism::isQuarterCube() const
{
    return std::abs(vSize(triangle.a - triangle.b) - (z_range.max - z_range.min)) < 10;
}

uint_fast8_t Cross3D::Cell::getChildCount() const
{
    return (children[2] < 0)? 2 : 4;
}

Cross3D::Cross3D(const DensityProvider& density_provider, const AABB3D aabb, const int max_depth, coord_t line_width)
: aabb(aabb)
, max_depth(max_depth)
, line_width(line_width)
, density_provider(density_provider)
{
}


float Cross3D::getDensity(const Cell& cell) const
{
    AABB aabb;
    aabb.include(cell.prism.triangle.straight_corner);
    aabb.include(cell.prism.triangle.a);
    aabb.include(cell.prism.triangle.b);
    AABB3D aabb3d(Point3(aabb.min.X, aabb.min.Y, cell.prism.z_range.min), Point3(aabb.max.X, aabb.max.Y, cell.prism.z_range.max));
    return density_provider(aabb3d);
}


void Cross3D::initialize()
{
    TimeKeeper tk;
    createTree();
    debugCheckDepths();
    debugCheckVolumeStats();
    logDebug("Created Cross3D tree with %i nodes and max depth %i in %5.2fs.\n", cell_data.size(), max_depth, tk.restart());
}


void Cross3D::createTree()
{
    assert(cell_data.empty());
    cell_data.reserve(2 << (max_depth / 2));
    Prism root_prism; // initialized with invalid data
    cell_data.emplace_back(root_prism, /*index =*/ 0, /* depth =*/ 0);
    Point3 aabb_size = aabb.max - aabb.min;
    cell_data[0].volume = INT2MM(aabb_size.x) * INT2MM(aabb_size.y) * INT2MM(aabb_size.z);
    cell_data[0].children[2] = -1;
    cell_data[0].children[3] = -1;

    // TODO: start with 4 quarter cubes so as to form a closed sierpinski curve
    AABB aabb2d = aabb.flatten();
    Triangle first_triangle(Point(aabb2d.min.X, aabb2d.max.Y), aabb2d.min, aabb2d.max, Triangle::Direction::AC_TO_AB, /* straight_corner_is_left =*/ true);
    Prism first_prism(first_triangle, aabb.min.z, aabb.max.z, /* is_expanding =*/ true);
    idx_t first_child_index = cell_data.size();
    cell_data[0].children[0] = first_child_index;
    cell_data.emplace_back(first_prism, first_child_index, /* depth =*/ 1);
    createTree(cell_data.back(), max_depth);
    setVolume(cell_data[first_child_index]);

    Triangle second_triangle(Point(aabb2d.max.X, aabb2d.min.Y), aabb2d.max, aabb2d.min, Triangle::Direction::AB_TO_BC, /* straight_corner_is_left =*/ true);
    Prism second_prism(second_triangle, aabb.min.z, aabb.max.z, /* is_expanding =*/ true);
    idx_t second_child_index = cell_data.size();
    cell_data[0].children[1] = second_child_index;
    cell_data.emplace_back(second_prism, second_child_index, /* depth =*/ 1);
    createTree(cell_data.back(), max_depth);
    setVolume(cell_data[second_child_index]);

    setSpecificationAllowance(cell_data[0]);
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
    const Prism parent_prism = sub_tree_root.prism;
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
    Triangle& parent_triangle = sub_tree_root.prism.triangle;
    Point ac = parent_triangle.straight_corner - parent_triangle.a;
    float area = 0.5 * INT2MM2(vSize2(ac));
    sub_tree_root.volume = area * INT2MM(sub_tree_root.prism.z_range.max - sub_tree_root.prism.z_range.min);

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

void Cross3D::setSpecificationAllowance(Cell& sub_tree_root)
{
    bool has_children = sub_tree_root.children[0] >= 0;
    if (has_children)
    {
        for (idx_t child_idx : sub_tree_root.children)
        {
            if (child_idx < 0)
            {
                break;
            }
            Cell& child = cell_data[child_idx];
            setSpecificationAllowance(child);
            sub_tree_root.filled_volume_allowance += child.filled_volume_allowance;
            sub_tree_root.minimally_required_density = std::max(sub_tree_root.minimally_required_density, child.minimally_required_density);
        }
    }
    else
    {
        float requested_density = getDensity(sub_tree_root);
        sub_tree_root.minimally_required_density = requested_density;
        sub_tree_root.filled_volume_allowance = sub_tree_root.volume * requested_density;
    }
}

/*
 * Tree creation /\                         .
 * 
 * =======================================================
 * 
 * Lower bound sequence \/                  .
 */


void Cross3D::createMinimalDensityPattern()
{
    TimeKeeper tk;
    std::list<idx_t> all_to_be_subdivided;
    
    std::function<bool(const Cell&)> shouldBeSubdivided =
        [this](const Cell& cell)
        {
            return getActualizedVolume(cell) / cell.volume < cell.minimally_required_density;
        };
    
    assert(cell_data.size() > 0);
    all_to_be_subdivided.push_back(0); // always subdivide the root, which is a bogus node!
    
    while (!all_to_be_subdivided.empty())
    {
        idx_t to_be_subdivided_idx = all_to_be_subdivided.front();
        Cell& to_be_subdivided = cell_data[to_be_subdivided_idx];

        if (to_be_subdivided.children[0] < 0 || to_be_subdivided.depth >= max_depth)
        { // this is a leaf cell
            all_to_be_subdivided.pop_front();
            continue;
        }

        if (!isConstrained(to_be_subdivided))
        {
            all_to_be_subdivided.pop_front();
            subdivide(to_be_subdivided);
            for (idx_t child_idx : to_be_subdivided.children)
            {
                if (child_idx >= 0 && shouldBeSubdivided(cell_data[child_idx]))
                {
                    all_to_be_subdivided.push_back(child_idx);
                }
            }
        }
        else
        {
//             all_to_be_subdivided.push_front(to_be_subdivided_idx); // retry after subdividing all neighbors
            for (std::list<Link>& side : to_be_subdivided.adjacent_cells)
            {
                for (Link& neighbor : side)
                {
                    if (isConstrainedBy(to_be_subdivided, cell_data[neighbor.to_index]))
                    {
                        all_to_be_subdivided.push_front(neighbor.to_index);
                    }
                }
            }
        }
    }
    logDebug("Cross3D::createMinimalDensityPattern finished in %5.2fs.\n", tk.restart());
}




float Cross3D::getActualizedVolume(const Cell& node) const
{
    const Triangle& triangle = node.prism.triangle;
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
    return INT2MM(line_width) * INT2MM(vSize(from_middle - to_middle)) * INT2MM(node.prism.z_range.max - node.prism.z_range.min);
}


bool Cross3D::canSubdivide(const Cell& cell) const
{
    if (cell.depth >= max_depth || isConstrained(cell))
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool Cross3D::isConstrained(const Cell& cell) const
{
    for (const std::list<Link>& side : cell.adjacent_cells)
    {
        for (const Link& neighbor : side)
        {
            if (isConstrainedBy(cell, cell_data[neighbor.to_index]))
            {
                return true;
            }
        }
    }
    return false;
}

bool Cross3D::isConstrainedBy(const Cell& constrainee, const Cell& constrainer) const
{
    return constrainer.depth < constrainee.depth;
}



void Cross3D::subdivide(Cell& cell)
{
//     const float total_loan_balance_before = getTotalLoanBalance(cell);
    
    assert(cell.children[0] >= 0 && cell.children[1] >= 0 && "Children must be initialized for subdivision!");
    Cell& child_lb = cell_data[cell.children[0]];
    Cell& child_rb = cell_data[cell.children[1]];
    initialConnection(child_lb, child_rb, Direction::RIGHT);

    if (cell.getChildCount() == 4)
    {
        Cell& child_lt = cell_data[cell.children[2]];
        Cell& child_rt = cell_data[cell.children[3]];
        initialConnection(child_lt, child_rt, Direction::RIGHT);
        initialConnection(child_lb, child_lt, Direction::UP);
        initialConnection(child_rb, child_rt, Direction::UP);
    }
    
    
    
    for (uint_fast8_t side = 0; side < getNumberOfSides(); side++)
    {
        /* two possible cases:
         * 1                                                                             ______          __  __
         * neighbor is refined more                                                   [][      ]      [][  ][  ]
         *      __                                                     deeper example [][      ]  =>  [][__][__]
         * [][][  ]  => [][][][]                                                      [][      ]      [][  ][  ]
         * [][][__]     [][][][]    We have the same amount of links                  [][______]      [][__][__]
         *       ^parent cell
         * 2
         * neighbor is refined less or equal                                           ______  __       ______
         *  __  __        __                                                          [      ][  ]     [      ][][]
         * [  ][  ]  =>  [  ][][]                                                     [      ][__]  => [      ][][]
         * [__][__]      [__][][]                                      deeper example [      ][  ]     [      ][][]
         *       ^parent cell                                                         [______][__]     [______][][]
         * Each link from a neighbor cell is split
         * into two links to two child cells
         * 
         * Both cases are cought by replacing each link with as many as needed,
         * which is either 1 or 2, because
         * in the new situation there are either 1 or 2 child cells neigghboring a beighbor cell of the parent.
         */
        for (LinkIterator neighbor_it = cell.adjacent_cells[side].begin(); neighbor_it != cell.adjacent_cells[side].end(); ++neighbor_it)
        {
            Link& neighbor = *neighbor_it;
            assert(neighbor.reverse);
            Cell& neighboring_cell = cell_data[neighbor.to_index];
            std::list<Link>& neighboring_edge_links = neighboring_cell.adjacent_cells[opposite(side)];
            
            std::list<Link*> new_incoming_links;
            std::list<Link*> new_outgoing_links;
            for (idx_t child_idx : cell.children)
            {
                if (child_idx < 0)
                {
                    break;
                }
                Cell& child = cell_data[child_idx];
                Cell& neighbor_cell = cell_data[neighbor.to_index];
                assert(neighbor.to_index > 0);
                if (isNextTo(child, neighbor_cell, static_cast<Direction>(side)))
                {
                    child.adjacent_cells[side].emplace_front(neighbor.to_index);
                    LinkIterator outlink = child.adjacent_cells[side].begin();
                    
                    neighboring_edge_links.emplace(*neighbor.reverse, child_idx);
                    LinkIterator inlink = *neighbor.reverse;
                    inlink--;
                    
                    outlink->reverse = inlink;
                    inlink->reverse = outlink;
                    
                    new_incoming_links.push_back(&*inlink);
                    new_outgoing_links.push_back(&*outlink);
                }
            }
//             transferLoans(neighbor.getReverse(), new_incoming_links);
//             transferLoans(neighbor, new_outgoing_links);
            neighboring_edge_links.erase(*neighbor.reverse);
        }
        
        cell.adjacent_cells[side].clear();
        
    }
    
    cell.is_subdivided = true;
    
    
//     float total_loan_balance_after = 0.0;
//     for (const Cell* child : cell.children)
//     {
//         assert(child && " we just subdivided!");
//         total_loan_balance_after += getTotalLoanBalance(*child);
//     }
//     assert(std::abs(total_loan_balance_after - total_loan_balance_before) < 0.0001);

    if (cell.depth > 0)
    { // ignore properties of dummy cell of root
        for (idx_t child_idx : cell.children)
        {
            if (child_idx < 0) continue;
            const Cell& child = cell_data[child_idx];
            assert(cell.prism.z_range.inside(child.prism.z_range.middle()));
            for (size_t side = 0; side < 2; side++)
            { // before and after
                for (const Link& link : child.adjacent_cells[side])
                {
                    const Cell& neighbor = cell_data[link.to_index];
                    assert(child.prism.z_range.overlap(neighbor.prism.z_range));
                }
            }
        }
    }
}


void Cross3D::initialConnection(Cell& before, Cell& after, Direction dir)
{
    before.adjacent_cells[static_cast<size_t>(dir)].emplace_front(after.index);
    after.adjacent_cells[static_cast<size_t>(opposite(dir))].emplace_front(before.index);
    LinkIterator before_to_after = before.adjacent_cells[static_cast<size_t>(dir)].begin();
    LinkIterator after_to_before = after.adjacent_cells[static_cast<size_t>(opposite(dir))].begin();
    before_to_after->reverse = after_to_before;
    after_to_before->reverse = before_to_after;
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
            if (!a.prism.z_range.overlap(b.prism.z_range.expanded(10)))
            {
                return false;
            }
            // check if triangle areas overlap
            Polygon a_polygon = a.prism.triangle.toPolygon();
            double a_area = a_polygon.area();
            Polygon b_polygon = b.prism.triangle.toPolygon();
            double b_area = b_polygon.area();
            Polygons intersection = a_polygon.intersection(b_polygon);
            double intersection_area = intersection.area();
            bool triangles_overlap = std::abs(intersection_area - std::min(a_area, b_area)) < 100.0; // TODO magic number
            return triangles_overlap;
        }
        case Direction::LEFT:
            a_edge = a.prism.triangle.getFromEdge();
            b_edge = b.prism.triangle.getToEdge();
            break;
        case Direction::RIGHT:
            a_edge = a.prism.triangle.getToEdge();
            b_edge = b.prism.triangle.getFromEdge();
            break;
        default:
            logError("Unknown direction passed to Cross3D::isNextTo!\n");
    }
    if (!a.prism.z_range.inside(b.prism.z_range.middle()) && !b.prism.z_range.inside(a.prism.z_range.middle()))
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
        if (last_cell->getChildCount() == 2 || left_bottom_child.prism.z_range.inside(z))
        {
            last_cell = &left_bottom_child;
        }
        else
        {
            const Cell& left_top_child = cell_data[last_cell->children[2]];
            assert(left_top_child.prism.z_range.inside(z));
            last_cell = &left_top_child;
        }
    }

    ret.layer_sequence.push_back(last_cell);
    while (!last_cell->adjacent_cells[static_cast<size_t>(Direction::RIGHT)].empty())
    {
        const Cell& bottom_neighbor = cell_data[last_cell->adjacent_cells[static_cast<size_t>(Direction::RIGHT)].front().to_index];
        const Cell& top_neighbor = cell_data[last_cell->adjacent_cells[static_cast<size_t>(Direction::RIGHT)].back().to_index];
        if (bottom_neighbor.prism.z_range.inside(z))
        {
            last_cell = &bottom_neighbor;
        }
        else
        {
            assert(top_neighbor.prism.z_range.inside(z) && "we assume a cell has max two neighbors in any given direction and either of them must overlap with the required z!");
            last_cell = &top_neighbor;
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
            if (cell.prism.z_range.max < new_z)
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
            }
        }

        new_z_is_beyond_current = false;
        for (iter_t iter = sequence.begin(); iter != sequence.end(); ++iter)
        {
            const Cell& cell = **iter;
            if (cell.prism.z_range.max < new_z)
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
        poly.add(cell->prism.triangle.getMiddle());
    }
    return poly;
}

Polygon Cross3D::generateCross(const SliceWalker& walker, coord_t z) const
{
    Polygon poly;
// {
// SVG svg("output/layer_sequence.svg", aabb.getAABB());

    const Cell* before = *std::prev(std::prev(walker.layer_sequence.end()));
    const Cell* here = walker.layer_sequence.back();
    for (const Cell* after: walker.layer_sequence)
    {
        sliceCell(*before, *here, *after, z, poly);
// debugOutputCell(*here, svg, 1, true);
// svg.writePoint(poly[poly.size() - 2]);
// svg.writePoint(poly[poly.size() - 1]);
// svg.writeDashedLine(poly[poly.size() - 2], poly[poly.size() - 1]);
        before = here;
        here = after;
    }
// }
    return poly;
}

void Cross3D::sliceCell(const Cell& before, const Cell& cell, const Cell& after, const coord_t z, PolygonRef output) const
{
    // TODO: complete this function!
    
    Point from = getCellEdgeLocation(before, cell, z);
    Point to = getCellEdgeLocation(cell, after, z);
//     Point from = getCellEdgeLocation(cell, cell.prism.triangle.getFromEdge(), z); // simplified TODO: revert to code above
//     Point to = getCellEdgeLocation(cell, cell.prism.triangle.getToEdge(), z); // simplified TODO: revert to code above

//     assert(output.empty() || vSize(to - output.back()) > 10);
    output.add(from);
//     output.add(from + normal(turn90CCW(from_edge.getVector()), line_width / 2));

//     output.add(to + normal(turn90CCW(to_edge.getVector()), -line_width / 2));
    output.add(to);
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
    
    const Cell* _densest_cell = &before;
    LineSegment _edge = before.prism.triangle.getToEdge();
    if (after.depth > before.depth)
    {
        _densest_cell = &after;
        _edge = after.prism.triangle.getFromEdge();
    }
    const Cell& densest_cell = *_densest_cell;
    const LineSegment edge = _edge;
    
    const coord_t edge_size = vSize(edge.getVector());
    coord_t pos = getCellEdgePosition(densest_cell, edge_size, z); // position along the edge where to put the vertex

    // check for constraining cells above or below
    if (z > densest_cell.prism.z_range.middle())
    { // check cell above
        applyZOscillationConstraint(before, after, z, densest_cell, edge, edge_size, Direction::UP, pos);
    }
    else
    { // check cell below
        applyZOscillationConstraint(before, after, z, densest_cell, edge, edge_size, Direction::DOWN, pos);
    }
    Point ret = getEdgeLocation(edge, edge_size, pos);
    assert(aabb.flatten().contains(ret));
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
            edge_above = densest_cell_above->prism.triangle.getToEdge();
        }
        const std::list<Link>& after_neighbors_above = after.adjacent_cells[static_cast<size_t>(checking_direction)];
        if (!after_neighbors_above.empty())
        {
            const Cell& after_above = cell_data[after_neighbors_above.front().to_index];
            if (!densest_cell_above || after_above.depth > densest_cell_above->depth)
            {
                densest_cell_above = &after_above;
                edge_above = after_above.prism.triangle.getFromEdge();
            }
        }
    }

    if (densest_cell_above && densest_cell_above->depth > densest_cell.depth)
    { // this cells oscillation pattern is altered to fit the oscillation pattern above
        const Point oscillation_end_point = // where the oscillation should end on the edge at this height
            (densest_cell_above->prism.is_expanding == checking_up) // flip for downward direction
            ? edge_above.from
            : edge_above.to;
        const coord_t oscillation_end_pos = dot(oscillation_end_point - edge.from, edge.to - edge.from) / vSize(edge.getVector()); // end position along the edge at this height
        assert(std::abs(oscillation_end_pos - vSize(oscillation_end_point - edge.from)) < 10 && "oscillation_end_point should lie on the segment!");
        assert(oscillation_end_pos >= -10 && oscillation_end_pos <= edge_size + 10);
        if (oscillation_end_pos > edge_size / 4 && oscillation_end_pos < edge_size * 3 / 4)
        { // oscillation end pos is in the middle
            if (z * flip_for_down > flip_for_down * (densest_cell.prism.z_range.middle() + flip_for_down * densest_cell.prism.z_range.size() / 4))
            // z is more than 3/4 for up or z less than 1/4 for down
            { // we're in the top quarter z of this prism
                if (densest_cell.prism.is_expanding == checking_up) // flip when cheking down
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
            if ((oscillation_end_pos > edge_size / 2) == (densest_cell.prism.is_expanding == checking_up)) // flip on is_expanding and on checking_up
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

Point Cross3D::getCellEdgeLocation(const Cell& cell, const LineSegment edge, const coord_t z) const
{
    const coord_t edge_size = vSize(edge.getVector());
    coord_t pos = getCellEdgePosition(cell, edge_size, z);
    return getEdgeLocation(edge, edge_size, pos);
}
Point Cross3D::getEdgeLocation(const LineSegment edge, const coord_t edge_size, coord_t pos) const
{
    assert(std::abs(vSize(edge.getVector()) - edge_size) < 10);
    pos  = std::min(edge_size - line_width / 2, std::max(line_width / 2, pos));
    if (pos < line_width / 2)
    { // edge size is smaller than a line width
        pos = edge_size / 2;
    }
    assert(pos >= 0);
    assert(pos <= edge_size);
    Point loc = edge.from + normal(edge.getVector(), pos);
    
    assert(aabb.flatten().contains(loc));
    return loc;
}

coord_t Cross3D::getCellEdgePosition(const Cell& cell, const coord_t edge_size, coord_t z) const
{
    coord_t pos = (z - cell.prism.z_range.min) * edge_size / cell.prism.z_range.size();
    if (!cell.prism.is_expanding)
    {
        pos = edge_size - pos;
    }
    assert(pos >= 0);
    assert(pos <= edge_size);
    return pos;
}

/*
 * Output /\                         .
 * 
 * =======================================================
 * 
 * Debug \/                  .
 */

void Cross3D::debugCheckDepths() const
{
    int problems = 0;
    for (const Cell& cell : cell_data)
    {
        for (idx_t child_idx : cell.children)
        {
            if (child_idx < 0) break;
            if (cell_data[child_idx].depth != cell.depth + 1)
            {
                problems++;
                logError("Cell with depth %i has a child with depth %i!\n", cell.depth, cell_data[child_idx].depth);
            }
        }
    }
    assert(problems == 0 && "no depth difference problems");
}

void Cross3D::debugCheckVolumeStats() const
{
    int problems = 0;
    for (const Cell& cell : cell_data)
    {
        if (cell.volume <= 0)
        {
            problems++;
            logError("Cell with depth %i has incorrect volume %f!\n", cell.depth, cell.volume);
        }
        if (cell.filled_volume_allowance < 0)
        {
            problems++;
            logError("Cell with depth %i has incorrect filled_volume_allowance  %f!\n", cell.depth, cell.filled_volume_allowance );
        }
        if (cell.minimally_required_density < 0)
        {
            problems++;
            logError("Cell with depth %i has incorrect minimally_required_density %f!\n", cell.depth, cell.minimally_required_density);
        }
        float child_filled_volume_allowance = 0;
        for (idx_t child_idx : cell.children)
        {
            if (child_idx < 0) break;
            const Cell& child = cell_data[child_idx];
            child_filled_volume_allowance += child.filled_volume_allowance;
        }
        if (cell.filled_volume_allowance < child_filled_volume_allowance - 0.1)
        {
            problems++;
            logError("Cell with depth %i has a children with more volume!\n", cell.depth);
        }
    }
    assert(problems == 0 && "no depth difference problems");
}


void Cross3D::debugCheckHeights(const SliceWalker& sequence, coord_t z) const
{
    for (const Cell* cell : sequence.layer_sequence)
    {
        assert(cell->prism.z_range.inside(z));
    }
}

void Cross3D::debugOutputCell(const Cell& cell, SVG& svg, float drawing_line_width, bool horizontal_connections_only) const
{
    debugOutputTriangle(cell.prism.triangle, svg, drawing_line_width);
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
    svg.writePoint(triangle.a, true, 1);
    svg.writePoint(triangle.b, true, 1);
    svg.writePoint(triangle.straight_corner, true, 1);
//     svg.writePolygon(tri, SVG::Color::GRAY);
    Polygons polys;
    polys.add(tri);
    polys = polys.offset(-80);
//     svg.writeAreas(polys, SVG::Color::GRAY, SVG::Color::BLACK);
    svg.writePolygons(polys, SVG::Color::GRAY);

    svg.writeLine(triangle.getFromEdge().middle(), triangle.getToEdge().middle(), SVG::Color::RED, drawing_line_width);
}

void Cross3D::debugOutputLink(const Link& link, SVG& svg) const
{
    Point a = cell_data[link.getReverse().to_index].prism.triangle.getMiddle();
    Point b = cell_data[link.to_index].prism.triangle.getMiddle();
    Point ab = b - a;
    Point shift = normal(turn90CCW(-ab), vSize(ab) / 20);
    coord_t shortening = vSize(ab) / 10;
    // draw arrow body
    Point c = a + shift + normal(ab, shortening);
    Point d = a + shift + normal(ab, vSize(ab) - shortening);
    svg.writeLine(c, d, SVG::Color::BLUE);
    svg.writePoint(c, false, 5, SVG::Color::BLUE);
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
        debugOutputTriangle(cell.prism.triangle, svg, drawing_line_width);
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
