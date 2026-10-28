// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "bridge/TransformedShape.h"

#include "geometry/Shape.h"


namespace curaengine::bridge
{

TransformedShape::TransformedShape(const ::curaengine::geometry::PointMatrix& matrix)
    : matrix_(matrix)
{
}

TransformedShape::TransformedShape(const ::curaengine::geometry::Shape& shape, const ::curaengine::geometry::PointMatrix& matrix)
    : TransformedShape(matrix)
{
    addShape(shape);
}

TransformedShape::TransformedShape(const ::curaengine::geometry::Polygon& polygon, const ::curaengine::geometry::PointMatrix& matrix)
    : TransformedShape(matrix)
{
    addPolygon(polygon);
}

void TransformedShape::addShape(const ::curaengine::geometry::Shape& shape, const bool filter_out_horizontal)
{
    segments_.reserve(segments_.size() + shape.pointCount());

    for (const ::curaengine::geometry::Polygon& polygon : shape)
    {
        constexpr bool reserve_size = false;
        addPolygon(polygon, filter_out_horizontal, reserve_size);
    }
}


void TransformedShape::addPolygon(const ::curaengine::geometry::Polygon& polygon, const bool filter_out_horizontal, const bool reserve_size)
{
    if (reserve_size)
    {
        segments_.reserve(segments_.size() + polygon.size());
    }

    for (auto iterator = polygon.beginSegments(); iterator != polygon.endSegments(); ++iterator)
    {
        addSegment((*iterator).start, (*iterator).end, filter_out_horizontal);
    }
}

void TransformedShape::addSegment(const ::curaengine::geometry::Point2LL& start, const ::curaengine::geometry::Point2LL& end, const bool filter_out_horizontal)
{
    TransformedSegment segment(start, end, matrix_);
    if (filter_out_horizontal && ::curaengine::utils::fuzzy_equal(segment.minY(), segment.maxY()))
    {
        return;
    }

    min_y_ = std::min(min_y_, segment.minY());
    max_y_ = std::max(max_y_, segment.maxY());
    segments_.push_back(std::move(segment));
}

} // namespace curaengine::bridge
