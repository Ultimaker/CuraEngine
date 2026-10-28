// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "geometry/PlanarPolygon3LL.h"

#include <range/v3/algorithm/max.hpp>
#include <range/v3/algorithm/min.hpp>
#include <range/v3/algorithm/minmax_element.hpp>
#include <range/v3/view/transform.hpp>

#include "geometry/Segment3LL.h"


namespace curaengine::geometry
{

PlanarPolygon3LL::PlanarPolygon3LL(std::vector<Segment3LL>&& segments)
    : segments_(std::move(segments))
{
}

PlanarPolygon3LL::PlanarPolygon3LL(const std::initializer_list<Segment3LL>& segments)
    : segments_(segments)
{
}

std::optional<PlanarPolygon3LL> PlanarPolygon3LL::intersectionWithLayer(
    const ::curaengine::utils::coord_t layer_start,
    const ::curaengine::utils::coord_t layer_end,
    const std::function<std::optional<Segment3LL>(const Segment3LL&, const ::curaengine::utils::coord_t, const ::curaengine::utils::coord_t)>& function_intersect_with_layer) const
{
    std::vector<Segment3LL> new_segments;

    const auto join_segments = [&new_segments](const Segment3LL& next_segment)
    {
        const ::curaengine::utils::coord_t join_distance = (new_segments.back().end() - next_segment.start()).vSize2();
        if (join_distance > ::curaengine::utils::EPSILON * ::curaengine::utils::EPSILON)
        {
            // Segments are not joined, add a transition segment
            new_segments.push_back(Segment3LL(new_segments.back().end(), next_segment.start()));
        }
        else
        {
            // Segments are almost joined, so slightly change the end of the previous segment to match
            new_segments.back().setEnd(next_segment.start());
        }
    };

    for (const Segment3LL& segment : segments_)
    {
        const std::optional<Segment3LL> cropped_segment = function_intersect_with_layer(segment, layer_start, layer_end);
        if (cropped_segment.has_value())
        {
            if (! new_segments.empty())
            {
                join_segments(*cropped_segment);
            }
            new_segments.push_back(*cropped_segment);
        }
    }

    if (new_segments.size() < 2)
    {
        return std::nullopt;
    }

    // Explicitly close the polygon if not closed yet
    join_segments(new_segments.front());

    return PlanarPolygon3LL(std::move(new_segments));
}

std::optional<PlanarPolygon3LL> PlanarPolygon3LL::intersectionWithXLayer(const ::curaengine::utils::coord_t layer_start_x, const ::curaengine::utils::coord_t layer_end_x) const
{
    return intersectionWithLayer(layer_start_x, layer_end_x, &Segment3LL::intersectionWithXLayer);
}

std::optional<PlanarPolygon3LL> PlanarPolygon3LL::intersectionWithYLayer(const ::curaengine::utils::coord_t layer_start_y, const ::curaengine::utils::coord_t layer_end_y) const
{
    return intersectionWithLayer(layer_start_y, layer_end_y, &Segment3LL::intersectionWithYLayer);
}

std::optional<PlanarPolygon3LL> PlanarPolygon3LL::intersectionWithZLayer(const ::curaengine::utils::coord_t layer_start_z, const ::curaengine::utils::coord_t layer_end_z) const
{
    return intersectionWithLayer(layer_start_z, layer_end_z, &Segment3LL::intersectionWithZLayer);
}

std::tuple<::curaengine::utils::coord_t, ::curaengine::utils::coord_t> PlanarPolygon3LL::minmaxX() const
{
    return minmax(
        [](const Segment3LL& segment)
        {
            return segment.start().x_;
        });
}

std::tuple<::curaengine::utils::coord_t, ::curaengine::utils::coord_t> PlanarPolygon3LL::minmaxY() const
{
    return minmax(
        [](const Segment3LL& segment)
        {
            return segment.start().y_;
        });
}

std::tuple<::curaengine::utils::coord_t, ::curaengine::utils::coord_t> PlanarPolygon3LL::minmaxZ() const
{
    return minmax(
        [](const Segment3LL& segment)
        {
            return segment.start().z_;
        });
}

std::tuple<::curaengine::utils::coord_t, ::curaengine::utils::coord_t> PlanarPolygon3LL::minmax(const std::function<::curaengine::utils::coord_t(const Segment3LL& segment)>& get_coordinate) const
{
    const auto segments_coordinates = segments_ | ranges::views::transform(get_coordinate);
    auto result = ranges::minmax_element(segments_coordinates);
    return std::make_tuple(*result.min, *result.max);
}

} // namespace curaengine::geometry
