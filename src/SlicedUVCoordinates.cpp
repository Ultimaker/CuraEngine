// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "SlicedUVCoordinates.h"

#include "slicer.h"

namespace cura
{

SlicedUVCoordinates::SlicedUVCoordinates(const std::vector<SlicerSegment>& segments)
    : located_uv_coordinates_(cell_size)
{
    segments_.reserve(segments.size());

    for (const SlicerSegment& segment : segments)
    {
        if (segment.uv_start.has_value() && segment.uv_end.has_value())
        {
            located_uv_coordinates_.insert(segment.start, segment.uv_start.value());
            located_uv_coordinates_.insert(segment.end, segment.uv_end.value());

            segments_.push_back(Segment{ segment.start, segment.end, segment.uv_start.value(), segment.uv_end.value() });
        }
    }
}

std::optional<Point2F> SlicedUVCoordinates::getClosestUVCoordinates(const Point2LL& position) const
{
    // First try the quick method, which will work in 99% cases
    SparsePointGridInclusiveImpl::SparsePointGridInclusiveElem<Point2F> nearest_uv_coordinates;
    if (located_uv_coordinates_.getNearest(position, search_radius, nearest_uv_coordinates))
    {
        return nearest_uv_coordinates.val;
    }

    // We couldn't find a close point with UV coordinates, so try to find the closest segment and project the point to it
    double closest_distance = std::numeric_limits<double>::max();
    std::optional<Point2F> closest_uv_coordinates;

    for (const Segment& segment : segments_)
    {
        const double segment_length = vSizef(segment.end - segment.start);
        if (std::abs(segment_length) < 0.001)
        {
            continue;
        }

        const double dot_product = dot((position - segment.start), (segment.end - segment.start)) / segment_length;
        double distance_to_segment;
        double interpolate_factor;

        if (dot_product > segment_length)
        {
            interpolate_factor = 1.0;
            distance_to_segment = vSizef(position - segment.end);
        }
        else if (dot_product < 0.0)
        {
            interpolate_factor = 0.0;
            distance_to_segment = vSizef(position - segment.start);
        }
        else
        {
            interpolate_factor = dot_product / segment_length;
            const Point2LL projected_position = cura::lerp(segment.start, segment.end, interpolate_factor);
            distance_to_segment = vSizef(position - projected_position);
        }

        if (distance_to_segment < closest_distance)
        {
            closest_distance = distance_to_segment;
            closest_uv_coordinates = cura::lerp(segment.uv_start, segment.uv_end, static_cast<float>(interpolate_factor));
        }
    }

    return closest_uv_coordinates;
}

} // namespace cura
