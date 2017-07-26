/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef INFILL_SPACE_FILLING_TREE_FILL_H
#define INFILL_SPACE_FILLING_TREE_FILL_H

#include "../utils/polygon.h"
#include "../utils/AABB.h"
#include "../utils/AABB3D.h"
#include "../utils/SpaceFillingTree.h"

#include "../utils/SVG.h"

namespace cura
{
class SpaceFillingTreeFill
{
    struct TreeParams
    {
        Point middle;
        coord_t radius;
        int depth;
    };
public:
    SpaceFillingTreeFill(coord_t line_distance, AABB3D model_aabb);

    void generate(const Polygons& outlines, coord_t shift, bool zig_zaggify, Polygons& result_polygons, Polygons& result_lines) const;
private:
    AABB3D model_aabb;
    coord_t line_distance;
    TreeParams tree_params;
    SpaceFillingTree tree;

    TreeParams getTreeParams(AABB3D model_aabb);

    void generateTreePath(PolygonRef path) const;

    Polygon offsetTreePath(const ConstPolygonRef path, coord_t offset) const;
};
} // namespace cura


#endif // INFILL_SPACE_FILLING_TREE_FILL_H
