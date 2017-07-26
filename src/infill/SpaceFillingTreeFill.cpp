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

void SpaceFillingTreeFill::generate(const Polygons& outlines, coord_t shift, bool zig_zaggify, double fill_angle, Polygons& result_polygons, Polygons& result_lines) const
{
    Point3 model_middle = model_aabb.getMiddle();
    Point3Matrix transformation = LinearAlg2D::rotateAround(Point(model_middle.x, model_middle.y), fill_angle + 45);

    Polygon tree_path;
    generateTreePath(tree_path);
    Polygon infill_poly = offsetTreePath(tree_path, shift);
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
        void visit(Point c)
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

Polygon SpaceFillingTreeFill::offsetTreePath(const ConstPolygonRef path, coord_t offset) const
{
    Polygon infill;
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
            infill.add(b + normal(ab_T, offset) + bc_offset); // TODO: calculate offset point correctly
        }
    }
    return infill;
}


}; // namespace cura
