/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#include "Cross3D.h"

#include "../utils/math.h"


namespace cura {


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
 * |B /     |B /
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

bool Cross3D::Prism::isHalfCube()
{
    return std::abs(vSize(triangle.straight_corner - triangle.b) - (z_max - z_min)) < 10;
}
bool Cross3D::Prism::isQuarterCube()
{
    return std::abs(vSize(triangle.a - triangle.b) - (z_max - z_min)) < 10;
}

uint_fast8_t Cross3D::Cell::getChildCount()
{
    return prism.isHalfCube() ? 2 : 4;
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
    AABB3D aabb3d(Point3(aabb.min.X, aabb.min.Y, cell.prism.z_min), Point3(aabb.max.X, aabb.max.Y, cell.prism.z_max));
    return density_provider(aabb3d);
}


void Cross3D::initialize()
{
    createTree();
}


void Cross3D::createTree()
{
    assert(cell_data.empty());
    cell_data.reserve(2 << (max_depth / 2));
    Prism root_prism; // initialized with invalid data
    cell_data.emplace_back(root_prism, /*index =*/ 0, /* depth =*/ 0);
    Point3 aabb_size = aabb.max - aabb.min;
    cell_data[0].volume = INT2MM(aabb_size.x) * INT2MM(aabb_size.y) * INT2MM(aabb_size.z);
    cell_data[0].children[0] = 1;
    cell_data[0].children[1] = 2;
    cell_data[0].children[2] = -1;
    cell_data[0].children[3] = -1;

    createTree(cell_data[1], max_depth);
    createTree(cell_data[2], max_depth);

    setVolume(cell_data[1]);
    setVolume(cell_data[2]);

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
    const Prism& parent_prism = sub_tree_root.prism;
    std::array<Triangle, 2> subdivided_triangles = parent_prism.triangle.subdivide();

    idx_t child_data_index = cell_data.size();

    uint_fast8_t child_count = sub_tree_root.getChildCount();
    assert(child_count == 2 || child_count == 4);
    coord_t child_z_min = parent_prism.z_min;
    coord_t child_z_max = (child_count == 2)? parent_prism.z_max : (parent_prism.z_max + parent_prism.z_min) / 2;
    for (uint_fast8_t child_z_idx = 0; child_z_idx < child_count / 2; child_z_idx++)
    { // only do a second iteration if there are 4 children
        for (uint_fast8_t child_xy_idx = 0; child_xy_idx < 2; child_xy_idx++)
        {
            bool is_expanding =
                (parent_prism.triangle.dir != Triangle::Direction::AC_TO_BC && child_xy_idx == 1)
                ? !parent_prism.is_expanding // is_expanding flips for these configurations. See class documentation.
                : parent_prism.is_expanding;
            if (child_z_idx == 1)
            { // upper children expand oppositely to lower children
                is_expanding = !is_expanding;
            }
            cell_data.emplace_back(Prism(subdivided_triangles[child_xy_idx], parent_prism.z_min, child_z_max, is_expanding), child_data_index, parent_depth + 1);
            sub_tree_root.children[child_z_idx * 2 + child_xy_idx] = child_data_index;
            child_data_index++;
            createTree(cell_data.back(), max_depth);
        }
        // update z range for the upper children
        child_z_min = child_z_max; // middle of parent z range
        child_z_max = parent_prism.z_max;
    }
}

void Cross3D::setVolume(Cell& sub_tree_root)
{
    Triangle parent_triangle = sub_tree_root.prism.triangle;
    Point ac = parent_triangle.straight_corner - parent_triangle.a;
    float area = 0.5 * INT2MM2(vSize2(ac));
    sub_tree_root.volume = area * INT2MM(sub_tree_root.prism.z_max - sub_tree_root.prism.z_min);

    bool has_children = sub_tree_root.children[0] >= 0;
    if (has_children)
    {
        for (idx_t child_idx : sub_tree_root.children)
        {
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

}; // namespace cura
