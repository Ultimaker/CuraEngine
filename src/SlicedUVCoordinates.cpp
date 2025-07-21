// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "SlicedUVCoordinates.h"

#include "utils/linearAlg2D.h"
#include "slicer.h"

#include <range/v3/view/take.hpp>

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

std::optional<std::pair<Point2F, Point2F>> SlicedUVCoordinates::getUVCoordsLineSegment(const Point2LL& a, const Point2LL& b) const
{
    // TODO/FIXME: This is currently not optimized.
    // (First tried to write the optimized version but it didn't work out... will have to try again later when more clearheaded.)
    // _ALSO!_ this'll only work reliably for quite simple UV-meshes, so it really _needs_ to change.

    typedef std::tuple<Segment*, bool, float> seg_dist2_t;

    const Point2LL* ptr_pt;
    std::vector<seg_dist2_t>* ptr_dist2_list;
    const auto gather_segments =
        [&ptr_pt, &ptr_dist2_list](Segment* const& seg)
        {
            const auto on_line = LinearAlg2D::getClosestOnLineSegment(*ptr_pt, seg->start, seg->end);
            const float err2 = vSize2(on_line - *ptr_pt);
            const float param = vSize(on_line - seg->start) / static_cast<float>(vSize(seg->end - seg->start));
            ptr_dist2_list->emplace_back(seg, param < 0.5f, err2);
            return true;
        };
    const auto sort_by_dist2 =
        [](const seg_dist2_t& q, const seg_dist2_t& r)
        {
            return std::get<2>(q) < std::get<2>(r);
        };

    std::vector<seg_dist2_t> closest_to_a;
    ptr_pt = &a;
    ptr_dist2_list = &closest_to_a;
    located_uv_coords_segs_.processNearby(a, search_radius, gather_segments);
    std::stable_sort(closest_to_a.begin(), closest_to_a.end(), sort_by_dist2);

    std::vector<seg_dist2_t> closest_to_b;
    ptr_pt = &b;
    ptr_dist2_list = &closest_to_b;
    located_uv_coords_segs_.processNearby(b, search_radius, gather_segments);
    std::stable_sort(closest_to_b.begin(), closest_to_b.end(), sort_by_dist2);

    float closest_uv_dist2 = std::numeric_limits<float>::infinity();
    std::pair<Point2F, Point2F> pts;

    for (const auto& a_seg_info : closest_to_a)
    {
        for (const auto& b_seg_info : closest_to_b)
        {
            Segment* a_seg = std::get<0>(a_seg_info);
            Segment* b_seg = std::get<0>(b_seg_info);

            const bool a_side = std::get<1>(a_seg_info);
            const bool b_side = std::get<1>(b_seg_info);

            const auto& a_uv = a_side ? a_seg->uv_start : a_seg->uv_end;
            const auto& b_uv = b_side ? b_seg->uv_start : b_seg->uv_end;
            //assert(a_uv != b_uv);
            if (a_uv == b_uv)
            {
                return std::nullopt;
            }

            const float uv_dist2 = (b_uv - a_uv).vSize2();
            if (uv_dist2 < closest_uv_dist2)
            {
                closest_uv_dist2 = uv_dist2;
                pts = { a_uv, b_uv };
            }
        }
    }

    return closest_uv_dist2 < std::numeric_limits<float>::infinity() ? std::make_optional(pts) : std::nullopt;
}

} // namespace cura
