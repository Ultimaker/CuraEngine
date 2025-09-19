// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/ParameterizedSegment.h"


namespace cura
{

ParameterizedSegment::ParameterizedSegment(const Point3D& start, const Point3D& end)
    : direction_(end - start)
    , start_(start)
    , end_(end)
{
}

Point3D ParameterizedSegment::pointAtX(const double x) const
{
    const double factor = (x - start_.x_) / (direction_.x_);
    return Point3D(x, start_.y_ + factor * direction_.y_, start_.z_ + factor * direction_.z_);
}

Point3D ParameterizedSegment::pointAtY(const double y) const
{
    const double factor = (y - start_.y_) / (direction_.y_);
    return Point3D(start_.x_ + factor * direction_.x_, y, start_.z_ + factor * direction_.z_);
}

std::optional<ParameterizedSegment> ParameterizedSegment::croppedSegmentX(const double layer_start, const double layer_end, const Point3D& p1, const Point3D& p2) const
{
    if (p1.x_ <= layer_end && p2.x_ >= layer_start)
    {
        return ParameterizedSegment(p1.x_ < layer_start ? pointAtX(layer_start) : p1, p2.x_ > layer_end ? pointAtX(layer_end) : p2);
    }

    return std::nullopt;
}

std::optional<ParameterizedSegment> ParameterizedSegment::intersectionWithXLayer(const double layer_start, const double layer_end) const
{
    if (direction_.x_ > 0)
    {
        return croppedSegmentX(layer_start, layer_end, start_, end_);
    }

    if (direction_.x_ < 0)
    {
        return croppedSegmentX(layer_start, layer_end, end_, start_);
    }

    if (start_.x_ >= layer_start && start_.x_ <= layer_end)
    {
        return *this;
    }

    return std::nullopt;
}

std::optional<ParameterizedSegment> ParameterizedSegment::croppedSegmentY(const double layer_start, const double layer_end, const Point3D& p1, const Point3D& p2) const
{
    if (p1.y_ <= layer_end && p2.y_ >= layer_start)
    {
        return ParameterizedSegment(p1.y_ < layer_start ? pointAtY(layer_start) : p1, p2.y_ > layer_end ? pointAtY(layer_end) : p2);
    }

    return std::nullopt;
}

std::optional<ParameterizedSegment> ParameterizedSegment::intersectionWithYLayer(const double layer_start, const double layer_end) const
{
    if (direction_.y_ > 0)
    {
        return croppedSegmentY(layer_start, layer_end, start_, end_);
    }

    if (direction_.y_ < 0)
    {
        return croppedSegmentY(layer_start, layer_end, end_, start_);
    }

    if (start_.y_ >= layer_start && start_.y_ <= layer_end)
    {
        return *this;
    }

    return std::nullopt;
}

} // namespace cura