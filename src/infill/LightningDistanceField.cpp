//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "LightningDistanceField.h" //Class we're implementing.
#include "../utils/polygonUtils.h" //For spreadDotsArea helper function.

namespace cura
{

LightningDistanceField::LightningDistanceField
(
    const coord_t& radius,
    const Polygons& current_outline,
    const Polygons& current_overhang
)
: cell_size(radius / 6) // TODO: make configurable!
, grid(cell_size)
, supporting_radius(radius)
, current_outline(current_outline)
, current_overhang(current_overhang)
{
    std::vector<Point> regular_dots = PolygonUtils::spreadDotsArea(current_overhang, cell_size);
    for (Point p : regular_dots)
    {
        const ClosestPolygonPoint cpp = PolygonUtils::findClosest(p, current_outline);
        const coord_t dist_to_boundary = vSize(p - cpp.p());
        unsupported_points.emplace_back(p, dist_to_boundary);
    }
    unsupported_points.sort(
        [](const UnsupCell& a, const UnsupCell& b)
        {
            coord_t da = a.dist_to_boundary + std::hash<Point>{}(a.loc) % 191;
            coord_t db = b.dist_to_boundary + std::hash<Point>{}(b.loc) % 191;
            return da < db;
        });
    for (auto it = unsupported_points.begin(); it != unsupported_points.end(); ++it)
    {
        UnsupCell& cell = *it;
        unsupported_points_grid.emplace(grid.toGridPoint(cell.loc), it);
    }
}

bool LightningDistanceField::tryGetNextPoint(Point* p) const
{
    if (unsupported_points.empty()) return false;
    *p = unsupported_points.front().loc;
    return true;
}

void LightningDistanceField::update(const Point& to_node, const Point& added_leaf)
{
    auto process_func = 
        [added_leaf, this](const SquareGrid::GridPoint& grid_loc)
        {
            auto it = unsupported_points_grid.find(grid_loc);
            if (it != unsupported_points_grid.end())
            {
                std::list<UnsupCell>::iterator& list_it = it->second;
                UnsupCell& cell = *list_it;
                if (shorterThen(cell.loc - added_leaf, supporting_radius))
                {
                    unsupported_points.erase(list_it);
                    unsupported_points_grid.erase(it);
                }
            }
            return true;
        };
    const Point a = to_node;
    const Point b = added_leaf;
    Point ab = b - a;
    Point ab_T = turn90CCW(ab);
    Point extent = normal(ab_T, supporting_radius);
    // TODO: process cells only once; make use of PolygonUtils::spreadDotsArea
    grid.processLineCells(std::make_pair(a + extent, a - extent),
                          [this, ab, extent, &process_func]
                          (GridPoint p)
                          {
                              grid.processLineCells(std::make_pair(p, p + ab), process_func);
                              return true;
                          }
    );
    grid.processNearby(added_leaf, supporting_radius, process_func);
}

}