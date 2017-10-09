/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "SpaceFillingTreeFill.h"

#include "../utils/linearAlg2D.h" // rotateAround

namespace cura {

SpaceFillingTreeFill::SpaceFillingTreeFill(coord_t line_distance, AABB3D model_aabb) : SpaceFillingTreeFill(line_distance, model_aabb, getTreeParams(line_distance, model_aabb))
{
}

SpaceFillingTreeFill::SpaceFillingTreeFill(coord_t line_distance, AABB3D model_aabb, SpaceFillingTreeFill::TreeParams tree_params)
: model_aabb(model_aabb)
, line_distance(line_distance)
, tree(tree_params.middle, tree_params.radius, tree_params.depth)
{
}

void SpaceFillingTreeFill::generate(const Polygons& outlines, coord_t shift, bool zig_zaggify, double fill_angle, bool alternate, bool use_odd_in_junctions, bool use_odd_out_junctions, coord_t pocket_size, Polygons& result_polygons, Polygons& result_lines) const
{
    Point3 model_middle = model_aabb.getMiddle();
    Point3Matrix transformation = LinearAlg2D::rotateAround(Point(model_middle.x, model_middle.y), fill_angle + 45);

    Polygon infill_poly;
    std::vector<const SpaceFillingTree::Node*> nodes;
    generateTreePath(nodes);
    offsetTreePath(nodes, shift, pocket_size, alternate, use_odd_in_junctions, use_odd_out_junctions, infill_poly);
    infill_poly.applyMatrix(transformation); // apply rotation

    if (zig_zaggify)
    {
        Polygons infill_pattern;
        infill_pattern.add(infill_poly);
        result_polygons = result_polygons.difference(outlines);
        result_polygons = result_polygons.unionPolygons(infill_pattern.intersection(outlines));
    }
    else
    {
        if (infill_poly.size() > 0)
        {
            infill_poly.add(infill_poly[0]);
        }
        Polygons infill_pattern;
        infill_pattern.add(infill_poly);
        Polygons poly_lines = outlines.intersectionPolyLines(infill_pattern);
        for (PolygonRef poly_line : poly_lines)
        {
            for (unsigned int point_idx = 1; point_idx < poly_line.size(); point_idx++)
            {
                result_lines.addLine(poly_line[point_idx - 1], poly_line[point_idx]);
            }
        }
    }
}

SpaceFillingTreeFill::TreeParams SpaceFillingTreeFill::getTreeParams(coord_t line_distance, AABB3D model_aabb)
{
    TreeParams ret;
    AABB aabb(Point(model_aabb.min.x, model_aabb.min.y), Point(model_aabb.max.x, model_aabb.max.y));
    const Point aabb_size = aabb.max - aabb.min;
    coord_t minimal_radius = vSize(aabb_size) / 2; // to account for any possible infill angle

    /*
     * For the normal cross infill the width of the cross is equal to the width of the crosses which are left out.
     * 
     *  ▉  ▄    white cross left out          r/2
     * ▀▉▀▀▉▀ ↙       by the black          ^^^^^^^^
     * ▄▉▄   ▄▉▄                     +--------------+
     *  ▉  ▄  ▉                      :              |
     * ▀▉▀▀▉▀▀▉▀▀                    : --------+    |   quarter of the fractal with depth one
     *                               :,.       |    | the fractal with depth one
     *                              .:  '-.    |    |
     *                            .' :  .' :'. |    |
     *                          .'   :.'...:.,'.....+
     *                        .'           :
     *                         '-.       .':   :
     *                          L  '-, .'  vvvvv
     *                                       n
     * minimum offset = 0
     * maximum offset is such that n = r/2
     * 2n^2 = L/2  ==>  n = 1/4 sqrt(2) L
     * r = .5 sqrt(2) L
     */
    ret.middle = aabb.getMiddle();
    ret.depth = 0;
    ret.radius = line_distance * sqrt(2.0); // doesn't work if it's multiplied by 0.5
    while (ret.radius <= minimal_radius)
    {
        ret.depth++;
        ret.radius *= 2;
    }
    return ret;
}

void SpaceFillingTreeFill::generateTreePath(std::vector<const SpaceFillingTree::Node*>& nodes) const
{
    class Visitor : public SpaceFillingTree::LocationVisitor
    {
        std::vector<const SpaceFillingTree::Node*>& nodes;
        bool is_first_point = true;
    public:
        Visitor(std::vector<const SpaceFillingTree::Node*>& nodes)
        : nodes(nodes)
        {}
        void visit(const SpaceFillingTree::Node* node)
        {
            if (is_first_point)
            { // skip first point because it is the same as the last
                is_first_point = false;
                return;
            }
            nodes.push_back(node);
        }
    };
    Visitor visitor(nodes);
    tree.walk(visitor);
}

void SpaceFillingTreeFill::offsetTreePath(std::vector<const SpaceFillingTree::Node*>& nodes, coord_t offset, coord_t pocket_size, bool alternating, bool use_odd_in_junctions, bool use_odd_out_junctions, PolygonRef infill) const
{
    coord_t corner_bevel = std::max(coord_t(0), pocket_size / 2 - offset) * std::sqrt(2);
    coord_t point_bevel = std::max(coord_t(0), pocket_size / 2 - (line_distance - offset)) * std::sqrt(2);

    coord_t corner_bevel_even = corner_bevel;
    coord_t corner_bevel_odd = 0;
    if (use_odd_in_junctions)
    {
        std::swap(corner_bevel_even, corner_bevel_odd);
    }
    coord_t point_bevel_even = point_bevel;
    coord_t point_bevel_odd = 0;
    if (use_odd_out_junctions)
    {
        std::swap(point_bevel_even, point_bevel_odd);
    }
    for (unsigned int point_idx = 0; point_idx < nodes.size(); point_idx++)
    {
        const SpaceFillingTree::Node* a_node = nodes[point_idx];
        const SpaceFillingTree::Node* b_node = nodes[(point_idx + 1) % nodes.size()];
        const SpaceFillingTree::Node* c_node = nodes[(point_idx + 2) % nodes.size()];

        const Point a = a_node->middle;
        const Point b = b_node->middle;
        const Point c = c_node->middle;

        const Point bc = c - b;
        const Point bc_T = turn90CCW(bc);
        const Point bc_offset = normal(bc_T, offset);

        if (a == c)
        { // pointy case
            //       .                          .
            //      / \                         .
            //     /   \                        .
            //    /  b  \                       .
            //   |   :   |                      .
            //   |   :   |                      .
            //       :
            // ......:......
            //      a c
            const Point left_point = b - bc_offset;
            const Point pointy_point = b - normal(bc, offset);
            const Point right_point = b + bc_offset;

            infill.add(left_point);
            const coord_t point_bevel_here = (alternating)?
                                             ((b_node->parent && b_node->parent_to_here_direction == b_node->parent->parent_to_here_direction)? point_bevel_even : point_bevel_odd)
                                             : point_bevel;
            if (point_bevel_here)
            {
                infill.add(pointy_point - normal(pointy_point - left_point, point_bevel_here));
                infill.add(pointy_point - normal(pointy_point - right_point, point_bevel_here));
            }
            else
            {
                infill.add(pointy_point);
            }
            infill.add(right_point);
        }
        else
        { // corner case
            //      a: |
            //       : |
            //       : L____
            // ......:......
            //      b:     c
            //       :
            //       :
            const Point ab = b - a;
            const Point ab_T = turn90CCW(ab);
            const Point normal_corner = b + normal(ab_T, offset) + bc_offset; // WARNING: offset is not based on the directions of the two segments, rather than assuming 90 degree corners
            const coord_t corner_bevel_here = (alternating)?
                                              ((a_node->distance_depth % 2 == 1)? corner_bevel_even : corner_bevel_odd)
                                              : corner_bevel;
            if (corner_bevel_here)
            {
                infill.add(normal_corner - normal(ab, corner_bevel_here));
                infill.add(normal_corner + normal(bc, corner_bevel_here));
            }
            else
            {
                infill.add(normal_corner);
            }
        }
    }
}



}; // namespace cura
