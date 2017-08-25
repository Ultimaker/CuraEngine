/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "SpaceFillingTreeFill.h"

#include "../utils/linearAlg2D.h" // rotateAround

namespace cura {

SpaceFillingTreeFill::SpaceFillingTreeFill(coord_t line_distance, AABB3D model_aabb)
: model_aabb(model_aabb)
, line_distance(line_distance)
, tree_params(getTreeParams(line_distance, model_aabb))
, tree(tree_params.middle, tree_params.radius, tree_params.depth)
{
}

void SpaceFillingTreeFill::generate(const Polygons& outlines, coord_t shift, bool zig_zaggify, double fill_angle, bool alternate, coord_t pocket_size, Polygons& result_polygons, Polygons& result_lines) const
{
    Point3 model_middle = model_aabb.getMiddle();
    Point3Matrix transformation = LinearAlg2D::rotateAround(Point(model_middle.x, model_middle.y), fill_angle + 45);

    Polygon infill_poly;
    if (alternate)
    {
        std::vector<const SpaceFillingTree::Node*> nodes;
        generateTreePathAndDepths(nodes);
        offsetTreePathAlternating(nodes, shift, line_distance - shift, pocket_size, infill_poly);
    }
    else
    {
        Polygon tree_path;
        generateTreePath(tree_path);
        offsetTreePath(tree_path, shift, pocket_size, infill_poly);
    }
    infill_poly.applyMatrix(transformation); // apply rotation

    if (zig_zaggify)
    {
        Polygons infill_pattern;
        infill_pattern.add(infill_poly);
        result_polygons = infill_pattern.intersection(outlines);
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
    coord_t minimal_radius = std::max(aabb_size.X, aabb_size.Y) / 2;
    minimal_radius *= sqrt(2.0); // to account for any possible infill angle

    /*
     * For the normal cross infill the width of the cross is equal to the width of the crosses which are left out.
     * 
     *  ▉  ▄    white cross left out          r/2
     * ▀▉▀▀▉▀ ↙       by the black          ^^^^^^^^
     * ▄▉▄   ▄▉▄                     +--------------+
     *  ▉  ▄  ▉                      :              |
     * ▀▉▀▀▉▀▀▉▀▀                    :  -------+    |   quarter of the fractal with depth one
     *                               :,'       |    | the fractal with depth one
     *                               :    .'   |    |
     *                            .  :  .' :   |    |
     *                          .    :.'...:.,'.....+
     *                        .            :
     *                         '-.       . :   :
     *                          l  '-, .   vvvvv
     *                                       n
     * minimum offset = 0
     * maximum offset is such that n = r /2
     * 2n^2 = l/2  ==>  n = 1/4 sqrt(2) l
     * r = .5 sqrt(2) l
     */
    ret.middle = aabb.getMiddle();
    ret.depth = -1;
    ret.radius = line_distance / 2 * sqrt(2.0);
    while (ret.radius <= minimal_radius)
    {
        ret.depth++;
        ret.radius *= 2;
    }
    return ret;
}

void SpaceFillingTreeFill::generateTreePath(PolygonRef path) const
{
    class Visitor : public SpaceFillingTree::LocationVisitor
    {
        PolygonRef path;
        bool is_first_point = true;
    public:
        Visitor(PolygonRef path)
        : path(path)
        {}
        void visit(const SpaceFillingTree::Node* node)
        {
            if (is_first_point)
            { // skip first point because it is the same as the last
                is_first_point = false;
                return;
            }
            path.add(node->middle);
        }
    };
    Visitor visitor(path);
    tree.walk(visitor);
}

void SpaceFillingTreeFill::offsetTreePath(const ConstPolygonRef path, coord_t offset, coord_t pocket_size, PolygonRef infill) const
{
    coord_t corner_bevel = std::max(coord_t(0), pocket_size / 2 - offset) * 1.4142;
    coord_t point_bevel = std::max(coord_t(0), pocket_size / 2 - (line_distance - offset)) * 1.4142;
    for (unsigned int point_idx = 0; point_idx < path.size(); point_idx++)
    {
        const Point a = path[point_idx];
        const Point b = path[(point_idx + 1) % path.size()];
        const Point c = path[(point_idx + 2) % path.size()];

        const Point bc = c - b;
        const Point bc_T = turn90CCW(bc);
        const Point bc_offset = normal(bc_T, offset);

        if (a == c)
        { // pointy case
            const Point left_point = b - bc_offset;
            const Point pointy_point = b - normal(bc, offset);
            const Point right_point = b + bc_offset;
            infill.add(left_point);
            if (point_bevel)
            {
                infill.add(pointy_point - normal(pointy_point - left_point, point_bevel));
                infill.add(pointy_point - normal(pointy_point - right_point, point_bevel));
            }
            else
            {
                infill.add(pointy_point);
            }
            infill.add(right_point);
        }
        else
        {
            const Point ab = b - a;
            const Point ab_T = turn90CCW(ab);
            const Point normal_corner = b + normal(ab_T, offset) + bc_offset; // WARNING: offset is not based on the directions of the two segments, rather than assuming 90 degree corners
            if (corner_bevel)
            {
                infill.add(normal_corner - normal(ab, corner_bevel));
                infill.add(normal_corner + normal(bc, corner_bevel));
            }
            else
            {
                infill.add(normal_corner);
            }
        }
    }
}




void SpaceFillingTreeFill::generateTreePathAndDepths(std::vector<const SpaceFillingTree::Node*>& nodes) const
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


void SpaceFillingTreeFill::offsetTreePathAlternating(std::vector<const SpaceFillingTree::Node*>& nodes, coord_t offset, coord_t alternate_offset, coord_t pocket_size, PolygonRef infill) const
{
    constexpr unsigned int algorithm_alternative_beveling = 1;
    std::function<coord_t (unsigned int depth, coord_t bevel)> getBevel;
    switch(algorithm_alternative_beveling)
    {
        case 0:
        default:
            getBevel = [] (unsigned int, coord_t bevel) { return bevel; };
            break;
        case 1:
            getBevel = [] (unsigned int depth, coord_t bevel) { return (depth % 2 == 1)? bevel : 0; };
            break;
    }
    constexpr unsigned int algorithm_alternative_alternating = 0;
    std::function<bool (unsigned int, unsigned int)> is_odd;
    switch(algorithm_alternative_alternating)
    {
        case 0:
        default:
            is_odd = [](unsigned int, unsigned int) { return false; };
            break;
        case 1:
            is_odd = [](unsigned int a_depth, unsigned int b_depth) { return (a_depth + b_depth) % 2 == 1; };
            break;
        case 2:
            is_odd = [](unsigned int a_depth, unsigned int b_depth) { return ((a_depth + b_depth) / 2 ) % 2 == 1; };
            break;
        case 3:
            is_odd = [](unsigned int a_depth, unsigned int b_depth) { return ((a_depth + b_depth + 1) / 2 ) % 2 == 1; };
            break;
        case 4:
            is_odd = [](unsigned int a_depth, unsigned int b_depth) { return std::min(a_depth, b_depth) % 2 == 1; };
            break;
        case 5:
            is_odd = [](unsigned int a_depth, unsigned int b_depth) { return ((std::min(a_depth, b_depth) + 1) / 2) % 2 == 1; };
            break;
        case 6:
            is_odd = [](unsigned int a_depth, unsigned int b_depth) { return ((a_depth + b_depth) / 4) % 2 == 1; };
            break;
        case 7:
            is_odd = [](unsigned int a_depth, unsigned int b_depth) { return ((a_depth + b_depth) / 7 ) % 2 == 1; };
            break;
    }
    constexpr unsigned int algorithm_alternative_point_case = 0;
    coord_t corner_bevel = std::max(coord_t(0), pocket_size / 2 - offset) * 1.4142;
    coord_t point_bevel = std::max(coord_t(0), pocket_size / 2 - (line_distance - offset)) * 1.4142;
    for (unsigned int point_idx = 0; point_idx < nodes.size(); point_idx++)
    {
        const Point a = nodes[point_idx]->middle;
        const Point b = nodes[(point_idx + 1) % nodes.size()]->middle;
        const Point c = nodes[(point_idx + 2) % nodes.size()]->middle;

        unsigned int a_depth = nodes[point_idx]->distance_depth;
        unsigned int b_depth = nodes[(point_idx + 1) % nodes.size()]->distance_depth;
        unsigned int c_depth = nodes[(point_idx + 2) % nodes.size()]->distance_depth;

        bool ab_odd = is_odd(a_depth, b_depth);
        bool bc_odd = is_odd(b_depth, c_depth);

        coord_t bc_offset_length = (bc_odd)? alternate_offset : offset;

        Point bc = c - b;
        Point bc_T = turn90CCW(bc);
        Point bc_offset = normal(bc_T, bc_offset_length);

        if (a == c)
        { // pointy case
            coord_t point_offset_length;
            switch(algorithm_alternative_point_case)
            {
                case 0:
                case 1:
                default:
                    point_offset_length = (bc_odd)? alternate_offset : offset;
                    break;
                case 2:
                    point_offset_length = (bc_odd)? offset : alternate_offset;
                    break;
                case 3:
                    point_offset_length = std::min(offset, alternate_offset);
                    break;
                case 4:
                    point_offset_length = std::max(offset, alternate_offset);
                    break;
            }
            const Point left_point = b - bc_offset - normal(bc, std::max(coord_t(0), point_offset_length - bc_offset_length));
            const Point pointy_point = b - normal(bc, point_offset_length);
            const Point right_point = b + bc_offset - normal(bc, std::max(coord_t(0), point_offset_length - bc_offset_length));

            infill.add(left_point);
            const coord_t point_bevel_here = getBevel(b_depth, point_bevel);
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
        {
            coord_t ab_offset_length = (ab_odd)? alternate_offset : offset;
            Point ab = b - a;
            Point ab_T = turn90CCW(ab);
            const Point normal_corner = b + normal(ab_T, ab_offset_length) + bc_offset; // WARNING: offset is not based on the directions of the two segments, rather than assuming 90 degree corners
            const coord_t corner_bevel_here = getBevel(a_depth, corner_bevel);
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
