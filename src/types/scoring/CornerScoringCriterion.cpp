// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/CornerScoringCriterion.h"

#include <algorithm>
#include <cstdlib>
#include <numbers>

#include <range/v3/algorithm/find_if.hpp>

#include "types/geometry/PointsSet.h"
#include "types/geometry/conversions/Point2D_Point2LL.h"
#include "utils/linear_alg_2d.h"
#include "utils/math.h"


namespace curaengine::types::scoring
{

using namespace types;
using namespace types::geometry;
using namespace utils;

CornerScoringCriterion::CornerScoringCriterion(const PointsSet& points, const EZSeamCornerPrefType corner_preference, const AngleDegrees& min_sharp_angle)
    : PositionBasedScoringCriterion(points)
    , corner_preference_(corner_preference)
    , segments_data_(points.size())
    , min_sharp_angle_(min_sharp_angle)
{
    switch (corner_preference_)
    {
    case EZSeamCornerPrefType::ZSeamCornerPrefInner:
        // Give advantage to concave corners. More advantage for sharper corners.
        concave_angle_score_ = 1.0;
        convex_angle_score_ = 0.0;
        break;
    case EZSeamCornerPrefType::ZSeamCornerPrefOuter:
        // Give advantage to convex corners. More advantage for sharper corners.
        concave_angle_score_ = 0.0;
        convex_angle_score_ = 1.0;
        break;
    case EZSeamCornerPrefType::ZSeamCornerPrefAny:
        // Still give sharper corners more advantage.
        concave_angle_score_ = 1.0;
        convex_angle_score_ = 1.0;
        break;
    case EZSeamCornerPrefType::ZSeamCornerPrefWeighted:
        // Give sharper corners some advantage, but concave sharper corners even more.
        concave_angle_score_ = 1.0;
        convex_angle_score_ = 0.5;
        break;
    case EZSeamCornerPrefType::Plugin:
        break;
    }

    // Pre-calculate the segments lengths because we are going to need them multiple times
    for (size_t i = 0; i < points.size(); ++i)
    {
        const Point2LL& here = getPoints().at(i);
        const Point2LL& next = getPoints().at((i + 1) % getPoints().size());
        const Point2LL vector = next - here;
        const coord_t segment_size = vSize(vector);
        segments_data_[i].size = segment_size;
        segments_data_[i].direction = conversions::toPoint2D(vector).vNormalized().value_or(Point2D());
        total_length_ += segment_size;
    }
}

double CornerScoringCriterion::computeScore(const size_t candidate_index) const
{
    const double corner_angle = cornerAngle(candidate_index, { 100, 5000 });

    // angles < 0 are concave (left turning)
    // angles > 0 are convex (right turning)
    const bool convex = corner_angle > 0;

    // Ignore very small angles, we cannot consider them as anyhow sharp
    const double min_angle_score{ min_sharp_angle_ / 180.0 };
    const double angle_weighted{ std::max(0.0, inverse_lerp(min_angle_score, 1.0, std::abs(corner_angle))) };

    return convex ? convex_angle_score_ * angle_weighted : concave_angle_score_ * angle_weighted;
}

double CornerScoringCriterion::cornerAngle(const size_t vertex_index, const std::vector<coord_t>& angle_query_distances) const
{
    std::vector<coord_t> filtered_angle_query_distances = angle_query_distances;
    const coord_t max_length = total_length_ / 2;
    const auto iterator = ranges::find_if(
        filtered_angle_query_distances,
        [&max_length](const coord_t distance)
        {
            return distance > max_length;
        });
    filtered_angle_query_distances.erase(iterator, filtered_angle_query_distances.end());
    if (filtered_angle_query_distances.empty())
    {
        filtered_angle_query_distances.push_back(max_length);
    }

    const std::vector<Point2LL> next_points = findNeighbourPoints(vertex_index, filtered_angle_query_distances, 1);
    const std::vector<Point2LL> previous_points = findNeighbourPoints(vertex_index, filtered_angle_query_distances, -1);

    double total_angle = 0.0;
    const Point2LL& here = getPoints().at(vertex_index);
    for (size_t i = 0; i < filtered_angle_query_distances.size(); ++i)
    {
        const Point2LL& next = next_points[i];
        const Point2LL& previous = previous_points[i];

        const double angle = linear_alg_2d::getAngleLeft(previous, here, next) - std::numbers::pi;

        total_angle += angle / std::numbers::pi;
    }

    return total_angle / filtered_angle_query_distances.size();
}

std::vector<Point2LL> CornerScoringCriterion::findNeighbourPoints(const size_t vertex_index, const std::vector<coord_t>& distances, const ptrdiff_t direction) const
{
    const size_t points_count = getPoints().size();

    // Segment i goes from point i to point i + 1, so when going forward the segment we just crossed starts one point before the current one
    const ptrdiff_t segment_offset = direction > 0 ? -1 : 0;

    std::vector<Point2LL> result;
    result.reserve(distances.size());

    // Travel on the path only once, and pick the points for each (ascending) distance on the way
    ptrdiff_t actual_delta = 0;
    coord_t travelled_distance = 0;
    const SegmentData* segment_data;
    for (const coord_t distance : distances)
    {
        while (travelled_distance < distance)
        {
            actual_delta += direction;
            segment_data = &segments_data_[wrapIndex(vertex_index, actual_delta + segment_offset, points_count)];
            travelled_distance += segment_data->size;
        }

        const Point2LL& next_pos = getPoints().at(wrapIndex(vertex_index, actual_delta, points_count));

        if (travelled_distance > distance) [[likely]]
        {
            // We have overtaken the required distance, go backward on the last segment
            const size_t prev = wrapIndex(vertex_index, actual_delta - direction, points_count);
            const Point2LL& prev_pos = getPoints().at(prev);
            const Point2D vector_delta{ segment_data->direction * static_cast<double>(direction) * (segment_data->size - (travelled_distance - distance)) };
            result.push_back(prev_pos + conversions::toPoint2LL(vector_delta));
        }
        else
        {
            // Luckily, the required distance stops exactly on an existing point
            result.push_back(next_pos);
        }
    }

    return result;
}

size_t CornerScoringCriterion::wrapIndex(const size_t base, const ptrdiff_t delta, const size_t count)
{
    const auto signed_count = static_cast<ptrdiff_t>(count);
    const ptrdiff_t index = (static_cast<ptrdiff_t>(base) + delta) % signed_count;
    return static_cast<size_t>(index < 0 ? index + signed_count : index);
}

} // namespace curaengine::types::scoring
