// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/LightningDistanceField.h" //Class we're implementing.

#include "utils/polygonUtils.h" //For spreadDotsArea helper function.

namespace cura
{

constexpr coord_t radius_per_cell_size = 6; // The cell-size should be small compared to the radius, but not so small as to be inefficient.

LightningDistanceField::LightningDistanceField(const coord_t& radius, const Shape& current_outline, const Shape& current_overhang)
    : cell_size_(radius / radius_per_cell_size)
    , grid_(cell_size_)
    , supporting_radius_(radius)
    , current_outline_(current_outline)
    , current_overhang_(current_overhang)
{
    std::vector<Point2LL> regular_dots = PolygonUtils::spreadDotsArea(current_overhang, cell_size_);
    for (const auto& p : regular_dots)
    {
        const ClosestPointPolygon cpp = PolygonUtils::findClosest(p, current_outline);
        const coord_t dist_to_boundary = vSize(p - cpp.p());
        unsupported_points_.emplace_back(p, dist_to_boundary);
    }
    unsupported_points_.sort(
        [&radius](const UnsupCell& a, const UnsupCell& b)
        {
            constexpr coord_t prime_for_hash = 191;
            return std::abs(b.dist_to_boundary_ - a.dist_to_boundary_) > radius
                     ? a.dist_to_boundary_ < b.dist_to_boundary_
                     : (std::hash<Point2LL>{}(a.loc_) % prime_for_hash) < (std::hash<Point2LL>{}(b.loc_) % prime_for_hash);
        });
    for (auto it = unsupported_points_.begin(); it != unsupported_points_.end(); ++it)
    {
        UnsupCell& cell = *it;
        unsupported_points_grid_.emplace(grid_.toGridPoint(cell.loc_), it);
    }
}

bool LightningDistanceField::tryGetNextPoint(Point2LL* p) const
{
    if (unsupported_points_.empty())
    {
        return false;
    }
    *p = unsupported_points_.front().loc_;
    return true;
}

void LightningDistanceField::update(const Point2LL& to_node, const Point2LL& added_leaf)
{
    auto process_func = [added_leaf, this](const SquareGrid::GridPoint& grid_loc)
    {
        auto it = unsupported_points_grid_.find(grid_loc);
        if (it != unsupported_points_grid_.end())
        {
            std::list<UnsupCell>::iterator& list_it = it->second;
            UnsupCell& cell = *list_it;
            if (shorterThen(cell.loc_ - added_leaf, supporting_radius_))
            {
                unsupported_points_.erase(list_it);
                unsupported_points_grid_.erase(it);
            }
        }
        return true;
    };
    const Point2LL a = to_node;
    const Point2LL b = added_leaf;
    Point2LL ab = b - a;
    Point2LL ab_T = turn90CCW(ab);
    Point2LL extent = normal(ab_T, supporting_radius_);
    // TODO: process cells only once; make use of PolygonUtils::spreadDotsArea
    grid_.processLineCells(
        std::make_pair(a + extent, a - extent),
        [this, ab, extent, &process_func](const SquareGrid::GridPoint& p)
        {
            grid_.processLineCells(std::make_pair(p, p + ab), process_func);
            return true;
        });
    grid_.processNearby(added_leaf, supporting_radius_, process_func);
}

} // namespace cura