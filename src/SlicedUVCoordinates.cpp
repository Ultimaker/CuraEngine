// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "SlicedUVCoordinates.h"

#include "utils/linearAlg2D.h"
#include "slicer.h"

namespace cura
{

SlicedUVCoordinates::SlicedUVCoordinates(const std::vector<SlicerSegment>& segments)
    : located_uv_coords_segs_(cell_size)
{
    segments_.reserve(segments.size());
    for (const SlicerSegment& segment : segments)
    {
        if (segment.uv_start.has_value() && segment.uv_end.has_value())
        {
            segments_.emplace_back(segment.start, segment.end, segment.uv_start.value(), segment.uv_end.value());
            Segment* seg = &segments_.back();

            located_uv_coords_segs_.insert(seg);

            segs_by_point_.insert({ seg->start, seg });
            segs_by_point_.insert({ seg->end, seg });
        }
    }
}

std::optional<Point2F> SlicedUVCoordinates::getClosestUVCoordinates(const Point2LL& position) const
{
    float closest_dist2 = std::numeric_limits<float>::infinity();
    const Segment* res = nullptr;
    float res_param = 0.0f;
    const auto find_closest =
        [&position, &closest_dist2, &res, &res_param](Segment* const& seg)
        {
            const auto on_line = LinearAlg2D::getClosestOnLineSegment(position, seg->start, seg->end);
            const float err2 = vSize2(on_line - position);
            if (err2 < closest_dist2)
            {
                closest_dist2 = err2;
                res = seg;
                res_param = vSize(on_line - seg->start) / static_cast<float>(vSize(seg->end - seg->start));
            }
            return true; // Don't stop searching at any point during this loop.
        };
    located_uv_coords_segs_.processNearby(position, search_radius, find_closest);
    return
        closest_dist2 < std::numeric_limits<float>::infinity() ?
        std::make_optional(res->uv_start + (res->uv_end - res->uv_start) * res_param) :
        std::nullopt;
}

} // namespace cura
