// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/polyline.h"

#include <numeric>

#include "geometry/lines_set.h"
#include "geometry/open_polyline.h"
#include "settings/types/Angle.h"
#include "utils/linearAlg2D.h"

namespace cura
{

template<PolylineType PolylineTypeVal>
void Polyline<PolylineTypeVal>::removeColinearEdges(const AngleRadians max_deviation_angle)
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

            const ClipperLib::Path& rpath = *this;
            const size_t pathlen = rpath.size();
            if (pathlen <= 3)
            {
                return;
            }

            std::vector<bool> skip_indices(size(), false);

            Polyline<PolylineTypeVal> new_path;
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
            (*this) = new_path;
            num_removed_in_iteration += pathlen - size();

            process_indices.clear();
            process_indices.insert(process_indices.end(), skip_indices.begin(), skip_indices.end());
        }
    } while (num_removed_in_iteration > 0);
}

template<PolylineType PolylineTypeVal>
Polyline<PolylineTypeVal>::const_segments_iterator Polyline<PolylineTypeVal>::beginSegments() const
{
    return const_segments_iterator(begin(), begin(), end());
}

template<PolylineType PolylineTypeVal>
Polyline<PolylineTypeVal>::const_segments_iterator Polyline<PolylineTypeVal>::endSegments() const
{
    if constexpr (type_ == PolylineType::Closed || type_ == PolylineType::Filled)
    {
        return const_segments_iterator(end(), begin(), end());
    }
    else
    {
        return const_segments_iterator(size() > 1 ? std::prev(end()) : end(), begin(), end());
    }
}

template<PolylineType PolylineTypeVal>
Polyline<PolylineTypeVal>::segments_iterator Polyline<PolylineTypeVal>::beginSegments()
{
    return segments_iterator(begin(), begin(), end());
}

template<PolylineType PolylineTypeVal>
Polyline<PolylineTypeVal>::segments_iterator Polyline<PolylineTypeVal>::endSegments()
{
    if constexpr (type_ == PolylineType::Closed || type_ == PolylineType::Filled)
    {
        return segments_iterator(end(), begin(), end());
    }
    else
    {
        return segments_iterator(size() > 1 ? std::prev(end()) : end(), begin(), end());
    }
}

template<PolylineType PolylineTypeVal>
coord_t Polyline<PolylineTypeVal>::length() const
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

template<PolylineType PolylineTypeVal>
bool Polyline<PolylineTypeVal>::shorterThan(const coord_t check_length) const
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

template<PolylineType PolylineTypeVal>
void Polyline<PolylineTypeVal>::splitIntoSegments(LinesSet<OpenPolyline>& result) const
{
    for (auto it = beginSegments(); it != endSegments(); ++it)
    {
        result.emplace_back(std::initializer_list<Point2LL>{ (*it).start, (*it).end });
    }
}

template<PolylineType PolylineTypeVal>
LinesSet<OpenPolyline> Polyline<PolylineTypeVal>::splitIntoSegments() const
{
    LinesSet<OpenPolyline> result;
    splitIntoSegments(result);
    return result;
}

template void Polyline<PolylineType::Open>::removeColinearEdges(const AngleRadians max_deviation_angle);
template Polyline<PolylineType::Open>::const_segments_iterator Polyline<PolylineType::Open>::beginSegments() const;
template Polyline<PolylineType::Open>::const_segments_iterator Polyline<PolylineType::Open>::endSegments() const;
template Polyline<PolylineType::Open>::segments_iterator Polyline<PolylineType::Open>::beginSegments();
template Polyline<PolylineType::Open>::segments_iterator Polyline<PolylineType::Open>::endSegments();
template coord_t Polyline<PolylineType::Open>::length() const;
template bool Polyline<PolylineType::Open>::shorterThan(const coord_t check_length) const;
template void Polyline<PolylineType::Open>::splitIntoSegments(LinesSet<OpenPolyline>& result) const;
template LinesSet<OpenPolyline> Polyline<PolylineType::Open>::splitIntoSegments() const;

template void Polyline<PolylineType::Closed>::removeColinearEdges(const AngleRadians max_deviation_angle);
template Polyline<PolylineType::Closed>::const_segments_iterator Polyline<PolylineType::Closed>::beginSegments() const;
template Polyline<PolylineType::Closed>::const_segments_iterator Polyline<PolylineType::Closed>::endSegments() const;
template Polyline<PolylineType::Closed>::segments_iterator Polyline<PolylineType::Closed>::beginSegments();
template Polyline<PolylineType::Closed>::segments_iterator Polyline<PolylineType::Closed>::endSegments();
template coord_t Polyline<PolylineType::Closed>::length() const;
template bool Polyline<PolylineType::Closed>::shorterThan(const coord_t check_length) const;
template void Polyline<PolylineType::Closed>::splitIntoSegments(LinesSet<OpenPolyline>& result) const;
template LinesSet<OpenPolyline> Polyline<PolylineType::Closed>::splitIntoSegments() const;

template void Polyline<PolylineType::Filled>::removeColinearEdges(const AngleRadians max_deviation_angle);
template Polyline<PolylineType::Filled>::const_segments_iterator Polyline<PolylineType::Filled>::beginSegments() const;
template Polyline<PolylineType::Filled>::const_segments_iterator Polyline<PolylineType::Filled>::endSegments() const;
template Polyline<PolylineType::Filled>::segments_iterator Polyline<PolylineType::Filled>::beginSegments();
template Polyline<PolylineType::Filled>::segments_iterator Polyline<PolylineType::Filled>::endSegments();
template coord_t Polyline<PolylineType::Filled>::length() const;
template bool Polyline<PolylineType::Filled>::shorterThan(const coord_t check_length) const;
template void Polyline<PolylineType::Filled>::splitIntoSegments(LinesSet<OpenPolyline>& result) const;
template LinesSet<OpenPolyline> Polyline<PolylineType::Filled>::splitIntoSegments() const;

} // namespace cura
