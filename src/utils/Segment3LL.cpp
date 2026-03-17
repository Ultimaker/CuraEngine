// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/Segment3LL.h"


namespace cura
{

Segment3LL::Segment3LL(const Point3LL& start, const Point3LL& end)
    : start_(start)
    , end_(end)
{
}

Point3LL Segment3LL::pointAtX(const coord_t x) const
{
    const Point3LL direction = end_ - start_;
    const double factor = static_cast<double>(x - start_.x_) / (direction.x_);
    return Point3LL(x, std::llrint(start_.y_ + factor * direction.y_), std::llrint(start_.z_ + factor * direction.z_));
}

Point3LL Segment3LL::pointAtY(const coord_t y) const
{
    const Point3LL direction = end_ - start_;
    const double factor = static_cast<double>(y - start_.y_) / (direction.y_);
    return Point3LL(std::llrint(start_.x_ + factor * direction.x_), y, std::llrint(start_.z_ + factor * direction.z_));
}

Point3LL Segment3LL::pointAtZ(const coord_t z) const
{
    const Point3LL direction = end_ - start_;
    const double factor = static_cast<double>(z - start_.z_) / (direction.z_);
    return Point3LL(std::llrint(start_.x_ + factor * direction.x_), std::llrint(start_.y_ + factor * direction.y_), z);
}

std::optional<Segment3LL> Segment3LL::intersectionWithLayer(
    const coord_t start_coordinate,
    const coord_t end_coordinate,
    const coord_t layer_start,
    const coord_t layer_end,
    const std::function<Point3LL(const Point3LL& point, const LayerLocation insideness, const coord_t layer_start, const coord_t layer_end)>& function_crop_point) const
{
    const LayerLocation segment_start_inside = pointIsInside(start_coordinate, layer_start, layer_end);
    const LayerLocation segment_end_inside = pointIsInside(end_coordinate, layer_start, layer_end);

    if (segment_end_inside == segment_start_inside)
    {
        if (segment_start_inside == LayerLocation::Inside)
        {
            // Segment is fully inside layer, take it as is
            return *this;
        }

        // Otherwise segment is fully outside, so intersection is empty
        return std::nullopt;
    }

    const Point3LL new_segment_start = function_crop_point(start_, segment_start_inside, layer_start, layer_end);
    const Point3LL new_segment_end = function_crop_point(end_, segment_end_inside, layer_start, layer_end);

    if ((new_segment_end - new_segment_start).vSize2() < EPSILON * EPSILON)
    {
        return std::nullopt;
    }

    return Segment3LL(new_segment_start, new_segment_end);
}

Segment3LL::LayerLocation Segment3LL::pointIsInside(const coord_t point, const coord_t layer_start, const coord_t layer_end)
{
    if (point < layer_start)
    {
        return LayerLocation::Below;
    }

    if (point > layer_end)
    {
        return LayerLocation::Above;
    }

    return LayerLocation::Inside;
}

Point3LL Segment3LL::croppedPoint(
    const Point3LL& point,
    const LayerLocation insideness,
    const coord_t layer_start,
    const coord_t layer_end,
    const std::function<Point3LL(const coord_t)>& function_point_at)
{
    switch (insideness)
    {
    case LayerLocation::Inside:
        return point;
    case LayerLocation::Below:
        return function_point_at(layer_start);
    case LayerLocation::Above:
        return function_point_at(layer_end);
    }

    return Point3LL();
}

Point3LL Segment3LL::croppedPointX(const Point3LL& point, const LayerLocation insideness, const coord_t layer_start, const coord_t layer_end) const
{
    return croppedPoint(point, insideness, layer_start, layer_end, std::bind(&Segment3LL::pointAtX, this, std::placeholders::_1));
}

Point3LL Segment3LL::croppedPointY(const Point3LL& point, const LayerLocation insideness, const coord_t layer_start, const coord_t layer_end) const
{
    return croppedPoint(point, insideness, layer_start, layer_end, std::bind(&Segment3LL::pointAtY, this, std::placeholders::_1));
}

Point3LL Segment3LL::croppedPointZ(const Point3LL& point, const LayerLocation insideness, const coord_t layer_start, const coord_t layer_end) const
{
    return croppedPoint(point, insideness, layer_start, layer_end, std::bind(&Segment3LL::pointAtZ, this, std::placeholders::_1));
}

std::optional<Segment3LL> Segment3LL::intersectionWithXLayer(const coord_t layer_start, const coord_t layer_end) const
{
    return intersectionWithLayer(
        start_.x_,
        end_.x_,
        layer_start,
        layer_end,
        std::bind(&Segment3LL::croppedPointX, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

std::optional<Segment3LL> Segment3LL::intersectionWithYLayer(const coord_t layer_start, const coord_t layer_end) const
{
    return intersectionWithLayer(
        start_.y_,
        end_.y_,
        layer_start,
        layer_end,
        std::bind(&Segment3LL::croppedPointY, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

std::optional<Segment3LL> Segment3LL::intersectionWithZLayer(const coord_t layer_start, const coord_t layer_end) const
{
    return intersectionWithLayer(
        start_.z_,
        end_.z_,
        layer_start,
        layer_end,
        std::bind(&Segment3LL::croppedPointZ, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

} // namespace cura