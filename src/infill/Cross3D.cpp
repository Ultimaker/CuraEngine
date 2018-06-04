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
, min_dist_to_cell_bound(line_width * 0.5 * sqrt2)
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
    logError("Created Cross3D tree with %i nodes and max depth %i in %5.2fs.\n", cell_data.size(), max_depth, tk.restart());
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
    // always subdivide the root, which is a bogus node!
    Cell& root = cell_data[0];
    for (idx_t child_idx : root.children)
    {
        if (child_idx >= 0 && shouldBeSubdivided(cell_data[child_idx]))
        {
            all_to_be_subdivided.push_back(child_idx);
        }
    }
    
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


void Cross3D::createDitheredPattern()
{
    bool midway_decision_boundary = false;
    createMinimalErrorPattern(midway_decision_boundary);

    std::vector<ChildSide> tree_path(max_depth, ChildSide::COUNT);
    dither(cell_data[0], tree_path);
}


void Cross3D::createMinimalErrorPattern(bool middle_decision_boundary)
{
    std::list<Cell*> to_be_checked;
    to_be_checked.push_back(&cell_data[0]);

    while (!to_be_checked.empty())
    {
        Cell* checking = to_be_checked.front();
        to_be_checked.pop_front();

        if (checking->children[0] < 0)
        { // cell has no children
            continue;
        }

        float decision_boundary = (middle_decision_boundary)?
                                    (getActualizedVolume(*checking) + getChildrenActualizedVolume(*checking)) / 2
                                    : getChildrenActualizedVolume(*checking);
        if (canSubdivide(*checking) && checking->filled_volume_allowance > decision_boundary)
        {
            subdivide(*checking);
            for (idx_t child_idx : checking->children)
            {
                if (child_idx < 0)
                {
                    break;
                }
                to_be_checked.push_back(&cell_data[child_idx]);
            }
        }
    }
}

void Cross3D::dither(Cell& parent, std::vector<ChildSide>& tree_path)
{
    if (parent.is_subdivided)
    {
        for (int path_dir = static_cast<int>(ChildSide::FIRST); path_dir < static_cast<int>(ChildSide::COUNT); path_dir++)
        {
            idx_t child_idx = parent.children[path_dir];
            if (child_idx > 0)
            {
                Cell& child = cell_data[child_idx];
                tree_path[parent.depth] = static_cast<ChildSide>(path_dir);
                dither(child, tree_path);
            }
        }
    }
    else
    {
        const float balance = getBalance(parent);
        const float parent_actualized_volume = getActualizedVolume(parent);
        const float subdivided_actualized_volume = getChildrenActualizedVolume(parent);
        const float range = subdivided_actualized_volume - parent_actualized_volume;
//         const float decision_boundary = (rand() % 100) * range / 100;
//         const float decision_boundary = (.01 * (rand() % 101) * .5 + .25) * range;
        const float decision_boundary = range / 2;
        bool do_subdivide = balance > decision_boundary && canSubdivide(parent);
        float actualized_volume = (do_subdivide)? subdivided_actualized_volume : parent_actualized_volume;

        const float left_over = balance - (actualized_volume - parent_actualized_volume); // might be negative

        // divide left_over equally per cell capacity, which is linearly related to the cell volume
        float total_weighted_forward_cell_volume = 0;
        Direction forwards[] = { Direction::RIGHT, Direction::UP };

        /* Floyd Steinberg weights:
         * 3 5 1
         *   * 7
         */

        float direction_weights[] = { 7, 5 }; // TODO: determine reasoned weights!
        float diag_weight = 0;//1;
        float backward_diag_weight = 3; //3; // TODO: error is still being propagated to already processed cells

        for (int side_idx = 0; side_idx < 2; side_idx++)
        {
            Direction direction = forwards[side_idx];
            std::list<Link>& side = parent.adjacent_cells[static_cast<size_t>(direction)];
            for (Link& link : side)
            {
                Cell& neighbor = cell_data[link.to_index];
                assert(neighbor.volume > 0 && "volume must be computed");
                total_weighted_forward_cell_volume += direction_weights[side_idx] * neighbor.volume;
            }
        }
        Link* diag_neighbor = getDiagonalNeighbor(parent, Direction::RIGHT);
        if (diag_neighbor)
        {
            total_weighted_forward_cell_volume += diag_weight * cell_data[diag_neighbor->to_index].volume;
        }
        Link* backward_diag_neighbor = getDiagonalNeighbor(parent, Direction::LEFT);
        if (!canPropagateLU(parent, tree_path))
        {
            backward_diag_neighbor = nullptr;
        }
        if (backward_diag_neighbor)
        {
            /*!
             * TODO avoid error being propagated to already processed cells!!:
             *  ___ ___ ___ ___
             * | 3 | 4 | 7 | 8 |
             * |___|__↖|___|___|
             * | 1 | 2 |↖5 | 6 |
             * |___|___|___|___|
             */
            total_weighted_forward_cell_volume += backward_diag_weight * cell_data[backward_diag_neighbor->to_index].volume;
        }

        assert(total_weighted_forward_cell_volume >= 0.0);
        assert(total_weighted_forward_cell_volume < 27000000.0); // 30x30x30cm
        for (int side_idx = 0; side_idx < 2; side_idx++)
        {
            Direction direction = forwards[side_idx];
            std::list<Link>& side = parent.adjacent_cells[static_cast<size_t>(direction)];
            for (Link& neighbor : side)
            {
                Link& loan_link = (left_over > 0)? neighbor : neighbor.getReverse();
                loan_link.loan += std::abs(left_over) * direction_weights[side_idx] * cell_data[neighbor.to_index].volume / total_weighted_forward_cell_volume;
            }
        }
        if (diag_neighbor)
        {
            Link& diag_loan_link = (left_over > 0)? *diag_neighbor : diag_neighbor->getReverse();
            diag_loan_link.loan += std::abs(left_over) * diag_weight * cell_data[diag_neighbor->to_index].volume / total_weighted_forward_cell_volume;
        }
        if (backward_diag_neighbor)
        {
            Link& diag_loan_link = (left_over > 0)? *backward_diag_neighbor : backward_diag_neighbor->getReverse();
            diag_loan_link.loan += std::abs(left_over) * backward_diag_weight * cell_data[backward_diag_neighbor->to_index].volume / total_weighted_forward_cell_volume;
        }

        if (do_subdivide)
        {
            subdivide(parent);
        }
    }
}

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
        if (sub_tree_root.prism.triangle.dir == Triangle::Direction::AC_TO_BC
            && sub_tree_root.prism.isQuarterCube())
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
                subdivide(sub_tree_root);
            }
        }
    }
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


float Cross3D::getChildrenActualizedVolume(const Cell& cell) const
{
    // The actualized volume of squares doesn't depend on surrounding cells,
    // so we just call getActualizedVolume(.)
    float actualized_volume = 0;
    for (idx_t child_idx : cell.children)
    {
        if (child_idx < 0)
        {
            continue;
        }
        actualized_volume += getActualizedVolume(cell_data[child_idx]);
    }
    return actualized_volume;
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
    const float total_loan_balance_before = getTotalLoanBalance(cell);

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
            transferLoans(neighbor.getReverse(), new_incoming_links);
            transferLoans(neighbor, new_outgoing_links);
            neighboring_edge_links.erase(*neighbor.reverse);
        }
        
        cell.adjacent_cells[side].clear();
        
    }
    
    cell.is_subdivided = true;
    
    
    float total_loan_balance_after = 0.0;
    for (const idx_t child_idx : cell.children)
    {
        if (child_idx < 0)
        {
            break;
        }
        total_loan_balance_after += getTotalLoanBalance(cell_data[child_idx]);
    }
    assert(std::abs(total_loan_balance_after - total_loan_balance_before) < 0.0001);

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


Cross3D::Link* Cross3D::getDiagonalNeighbor(Cell& cell, Direction left_right) const
{ // implementation naming assumes left_right is right
    std::list<Link>& right_side = cell.adjacent_cells[static_cast<size_t>(left_right)];
    if (!right_side.empty())
    {
        const std::list<Link>& right_side_up_side = cell_data[right_side.back().to_index].adjacent_cells[static_cast<size_t>(Direction::UP)];
        if (!right_side_up_side.empty())
        {
            const Link& ru_diag_neighbor = right_side_up_side.front();
            const std::list<Link>& up_side = cell.adjacent_cells[static_cast<size_t>(Direction::UP)];
            if (!up_side.empty())
            {
                const std::list<Link>& up_side_right_side = cell_data[up_side.back().to_index].adjacent_cells[static_cast<size_t>(left_right)];
                if (!up_side_right_side.empty())
                {
                    const Link& ur_diag_neighbor = up_side_right_side.front();
                    if (ru_diag_neighbor.to_index == ur_diag_neighbor.to_index)
                    {
                        return const_cast<Link*>(&ru_diag_neighbor);
                    }
                }
            }
        }
    }
    return nullptr;
}


bool Cross3D::canPropagateLU(Cell& cell, const std::vector<ChildSide>& tree_path)
{
    if (cell.depth == 0)
    {
        return false;
    }
    if (tree_path[cell.depth - 1] == ChildSide::LEFT_BOTTOM)
    {
        return false;
    }
    if (tree_path[cell.depth - 1] != ChildSide::LEFT_TOP)
    {
        return true;
    }
    int tree_path_idx;
    for (tree_path_idx = cell.depth - 1; tree_path_idx >= 0; tree_path_idx--)
    {
        if (tree_path[tree_path_idx] == ChildSide::RIGHT_BOTTOM || tree_path[tree_path_idx] == ChildSide::RIGHT_TOP)
        {
            break;
        }
    }
    for (; tree_path_idx < cell.depth; tree_path_idx++)
    {
        if (tree_path[tree_path_idx] == ChildSide::LEFT_BOTTOM)
        {
            return false;
        }
    }
    return true;
}

/*
 * Tree creation /\                         .
 * 
 * =======================================================
 * 
 * Error redistribution \/                  .
 */

float Cross3D::getBalance(const Cell& cell) const
{
    float balance = cell.filled_volume_allowance - getActualizedVolume(cell) + getTotalLoanBalance(cell);
    return balance;
}

float Cross3D::getTotalLoanBalance(const Cell& cell) const
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
            if (link.getReverse().loan > 0.00001)
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
    const Cell& start_cell = *last_cell;

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
                assert(iter_after == sequence.end() || (*iter_after)->index != (*iter)->index);
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

    LineSegment from_edge = cell.prism.triangle.getFromEdge();
    LineSegment to_edge = cell.prism.triangle.getToEdge();

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
    const LineSegment edge = (after.depth > before.depth)? after.prism.triangle.getFromEdge() : before.prism.triangle.getToEdge();

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

    { // Keep lines away from cell boundary to prevent line overlap
        // TODO: make min_dist_to_cell_bound depend on the triangles involved
        // if it's a 90* corner, it should be sqrt(2)*line_width/2
        // if it's a 135* corner, it should be sqrt(2)*line_width/2
        // if it's a 180* thing, it should be line_width/2
        pos  = std::min(edge_size - min_dist_to_cell_bound, std::max(min_dist_to_cell_bound, pos));
        if (pos < min_dist_to_cell_bound)
        { // edge size is smaller than a line width
            pos = edge_size / 2;
        }
        assert(pos >= 0);
        assert(pos <= edge_size);
    }

    Point ret = edge.from + normal(edge.getVector(), pos);
    assert(aabb.flatten().contains(ret));

    LineSegment edge1 = before.prism.triangle.getToEdge();
    LineSegment edge2 = after.prism.triangle.getFromEdge();
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
    const Cell* prev = sequence.layer_sequence.back();
    for (const Cell* cell : sequence.layer_sequence)
    {
        assert(cell->prism.z_range.inside(z));
        assert(cell->index != prev->index);
        prev = cell;
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
//     svg.writePoint(triangle.a, true, 1);
//     svg.writePoint(triangle.b, true, 1);
//     svg.writePoint(triangle.straight_corner, true, 1);
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
