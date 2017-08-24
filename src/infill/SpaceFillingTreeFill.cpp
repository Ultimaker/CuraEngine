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

void SpaceFillingTreeFill::generate(const Polygons& outlines, coord_t shift, bool zig_zaggify, double fill_angle, bool alternate, Polygons& result_polygons, Polygons& result_lines) const
{
    Point3 model_middle = model_aabb.getMiddle();
    Point3Matrix transformation = LinearAlg2D::rotateAround(Point(model_middle.x, model_middle.y), fill_angle + 45);

    Polygon infill_poly;
    if (alternate)
    {
        Polygon tree_path;
        std::vector<unsigned int> depths;
        generateTreePathAndDepths(tree_path, depths);
        offsetTreePathAlternating(tree_path, depths, shift, line_distance - shift, infill_poly);
    }
    else
    {
        Polygon tree_path;
        generateTreePath(tree_path);
        offsetTreePath(tree_path, shift, infill_poly);
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
        void visit(Point c, unsigned int)
        {
            if (is_first_point)
            { // skip first point because it is the same as the last
                is_first_point = false;
                return;
            }
            path.add(c);
        }
    };
    Visitor visitor(path);
    tree.walk(visitor);
}

void SpaceFillingTreeFill::offsetTreePath(const ConstPolygonRef path, coord_t offset, PolygonRef infill) const
{
    for (unsigned int point_idx = 0; point_idx < path.size(); point_idx++)
    {
        const Point a = path[point_idx];
        const Point b = path[(point_idx + 1) % path.size()];
        const Point c = path[(point_idx + 2) % path.size()];

        Point bc = c - b;
        Point bc_T = turn90CCW(bc);
        Point bc_offset = normal(bc_T, offset);

        if (a == c)
        { // pointy case
            infill.add(b - bc_offset);
            infill.add(b - normal(bc, offset));
            infill.add(b + bc_offset);
        }
        else
        {
            Point ab = b - a;
            Point ab_T = turn90CCW(ab);
            infill.add(b + normal(ab_T, offset) + bc_offset); // WARNING: offset is not based on the directions of the two segments, rather than assuming 90 degree corners
        }
    }
}




void SpaceFillingTreeFill::generateTreePathAndDepths(PolygonRef path, std::vector<unsigned int>& depths) const
{
    class Visitor : public SpaceFillingTree::LocationVisitor
    {
        PolygonRef path;
        std::vector<unsigned int>& depths;
        bool is_first_point = true;
    public:
        Visitor(PolygonRef path, std::vector<unsigned int>& depths)
        : path(path)
        , depths(depths)
        {}
        void visit(Point c, unsigned int depth)
        {
            if (is_first_point)
            { // skip first point because it is the same as the last
                is_first_point = false;
                return;
            }
            path.add(c);
            depths.push_back(depth);
        }
    };
    Visitor visitor(path, depths);
    tree.walk(visitor);
}


void SpaceFillingTreeFill::offsetTreePathAlternating(const ConstPolygonRef path, const std::vector<unsigned int>& depths, coord_t offset, coord_t alternate_offset, PolygonRef infill) const
{
    constexpr unsigned int algorithm_alternative_alternating = 5;
    std::function<bool (unsigned int, unsigned int)> is_odd;
    switch(algorithm_alternative_alternating)
    {
        case 1:
        default:
            is_odd = [](unsigned int a_depth, unsigned int b_depth) { return (a_depth + b_depth) % 2 == 1; };
            break;
        case 2:
            is_odd = [](unsigned int a_depth, unsigned int b_depth) { return ((a_depth + b_depth) / 2 ) % 2 == 1; };
            break;
        case 3:
            is_odd = [](unsigned int a_depth, unsigned int b_depth) { return ((a_depth + b_depth + 1) / 2 ) % 2 == 1; };
            break;
        case 4:
            is_odd = [](unsigned int a_depth, unsigned int b_depth) { return (a_depth < b_depth)? a_depth % 2 == 1 : b_depth % 2 == 1; };
            break;
        case 5:
            is_odd = [](unsigned int a_depth, unsigned int b_depth) { return std::max(a_depth, b_depth) % 2 == 1; };
            break;
    }
    constexpr unsigned int algorithm_alternative_point_case = 1;
    for (unsigned int point_idx = 0; point_idx < path.size(); point_idx++)
    {
        const Point a = path[point_idx];
        const Point b = path[(point_idx + 1) % path.size()];
        const Point c = path[(point_idx + 2) % path.size()];

        unsigned int a_depth = depths[point_idx];
        unsigned int b_depth = depths[(point_idx + 1) % path.size()];
        unsigned int c_depth = depths[(point_idx + 2) % path.size()];

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
                case 1:
                default:
                    point_offset_length = bc_offset_length;
                    break;
                case 2:
                    point_offset_length = std::min(offset, alternate_offset);
                    break;
                case 3:
                    point_offset_length = (bc_odd)? offset : alternate_offset;
                    break;
                case 4:
                    point_offset_length = std::max(offset, alternate_offset);
                    break;
            }
            infill.add(b - bc_offset - normal(bc, std::max(coord_t(0), point_offset_length - bc_offset_length)));
            infill.add(b - normal(bc, point_offset_length));
            infill.add(b + bc_offset - normal(bc, std::max(coord_t(0), point_offset_length - bc_offset_length)));
        }
        else
        {
            coord_t ab_offset_length = (ab_odd)? alternate_offset : offset;
            Point ab = b - a;
            Point ab_T = turn90CCW(ab);
            infill.add(b + normal(ab_T, ab_offset_length) + bc_offset); // WARNING: offset is not based on the directions of the two segments, rather than assuming 90 degree corners
        }
    }
}



}; // namespace cura
