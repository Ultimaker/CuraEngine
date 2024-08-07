// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/Polyline.h"

#include <algorithm>
#include <numbers>
#include <numeric>

#include "geometry/LinesSet.h"
#include "geometry/OpenLinesSet.h"
#include "geometry/OpenPolyline.h"
#include "settings/types/Angle.h"
#include "utils/linearAlg2D.h"

namespace cura
{

void Polyline::removeColinearEdges(const AngleRadians max_deviation_angle)
{
    // TODO: Can be made more efficient (for example, use pointer-types for process-/skip-indices, so we can swap them without copy).

    size_t num_removed_in_iteration = 0;
    do
    {
        num_removed_in_iteration = 0;

        std::vector<bool> process_indices(size(), true);

        bool go = true;
        while (go)
        {
            go = false;

            const ClipperLib::Path& rpath = getPoints();
            const size_t pathlen = rpath.size();
            if (pathlen <= 3)
            {
                return;
            }

            std::vector<bool> skip_indices(size(), false);

            std::vector<Point2LL> new_path;
            for (size_t point_idx = 0; point_idx < pathlen; ++point_idx)
            {
                // Don't iterate directly over process-indices, but do it this way, because there are points _in_ process-indices that should nonetheless be skipped:
                if (! process_indices[point_idx])
                {
                    new_path.push_back(rpath[point_idx]);
                    continue;
                }

                // Should skip the last point for this iteration if the old first was removed (which can be seen from the fact that the new first was skipped):
                if (point_idx == (pathlen - 1) && skip_indices[0])
                {
                    skip_indices[new_path.size()] = true;
                    go = true;
                    new_path.push_back(rpath[point_idx]);
                    break;
                }

                const Point2LL& prev = rpath[(point_idx - 1 + pathlen) % pathlen];
                const Point2LL& pt = rpath[point_idx];
                const Point2LL& next = rpath[(point_idx + 1) % pathlen];

                double angle = LinearAlg2D::getAngleLeft(prev, pt, next); // [0 : 2 * pi]
                if (angle >= std::numbers::pi)
                {
                    angle -= std::numbers::pi;
                } // map [pi : 2 * pi] to [0 : pi]

                // Check if the angle is within limits for the point to 'make sense', given the maximum deviation.
                // If the angle indicates near-parallel segments ignore the point 'pt'
                if (angle > max_deviation_angle && angle < std::numbers::pi - max_deviation_angle)
                {
                    new_path.push_back(pt);
                }
                else if (point_idx != (pathlen - 1))
                {
                    // Skip the next point, since the current one was removed:
                    skip_indices[new_path.size()] = true;
                    go = true;
                    new_path.push_back(next);
                    ++point_idx;
                }
            }
            setPoints(std::move(new_path));
            num_removed_in_iteration += pathlen - size();

            process_indices.clear();
            process_indices.insert(process_indices.end(), skip_indices.begin(), skip_indices.end());
        }
    } while (num_removed_in_iteration > 0);
}

Polyline::const_segments_iterator Polyline::beginSegments() const
{
    return const_segments_iterator(begin(), begin(), end());
}

Polyline::const_segments_iterator Polyline::endSegments() const
{
    if (hasClosingSegment())
    {
        return const_segments_iterator(end(), begin(), end());
    }
    return const_segments_iterator(size() > 1 ? std::prev(end()) : end(), begin(), end());
}

Polyline::segments_iterator Polyline::beginSegments()
{
    return segments_iterator(begin(), begin(), end());
}

Polyline::segments_iterator Polyline::endSegments()
{
    if (hasClosingSegment())
    {
        return segments_iterator(end(), begin(), end());
    }
    return segments_iterator(size() > 1 ? std::prev(end()) : end(), begin(), end());
}

coord_t Polyline::length() const
{
    return std::accumulate(
        beginSegments(),
        endSegments(),
        0,
        [](coord_t total, const const_segments_iterator::value_type& segment)
        {
            return total + vSize(segment.end - segment.start);
        });
}

bool Polyline::shorterThan(const coord_t check_length) const
{
    coord_t length = 0;
    auto iterator_segment = std::find_if(
        beginSegments(),
        endSegments(),
        [&length, &check_length](const const_segments_iterator::value_type& segment)
        {
            length += vSize(segment.end - segment.start);
            return length >= check_length;
        });
    return iterator_segment == endSegments();
}

void Polyline::splitIntoSegments(OpenLinesSet& result) const
{
    result.reserve(result.size() + segmentsCount());
    for (auto it = beginSegments(); it != endSegments(); ++it)
    {
        result.emplace_back(OpenPolyline({ (*it).start, (*it).end }));
    }
}

OpenLinesSet Polyline::splitIntoSegments() const
{
    OpenLinesSet result;
    splitIntoSegments(result);
    return result;
}

} // namespace cura
