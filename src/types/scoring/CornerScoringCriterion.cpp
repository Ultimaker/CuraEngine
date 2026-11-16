// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/scoring/CornerScoringCriterion.h"

#include <numbers>

#include <range/v3/algorithm/find_if.hpp>

#include "types/geometry/PointsSet.h"
#include "utils/linear_alg_2d.h"
#include "utils/math.h"


namespace curaengine::types::scoring
{

using namespace types;
using namespace types::geometry;
using namespace utils;

CornerScoringCriterion::CornerScoringCriterion(const PointsSet& points, const EZSeamCornerPrefType corner_preference)
    : PositionBasedScoringCriterion(points)
    , corner_preference_(corner_preference)
    , segments_sizes_(points.size())
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
        const coord_t segment_size = types::geometry::vSize(next - here);
        segments_sizes_[i] = segment_size;
        total_length_ += segment_size;
    }
}

double CornerScoringCriterion::computeScore(const size_t candidate_index) const
{
    const double corner_angle = cornerAngle(candidate_index, { 100, 1000 });
    // angles < 0 are concave (left turning)
    // angles > 0 are convex (right turning)

    return corner_angle > 0 ? convex_angle_score_ * corner_angle : concave_angle_score_ * (-corner_angle);
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

std::vector<Point2LL> CornerScoringCriterion::findNeighbourPoints(const size_t vertex_index, const std::vector<coord_t>& distances, const int direction) const
{
    const size_t points_count = getPoints().size();
    const int size_delta = direction > 0 ? -1 : 0;

    std::vector<Point2LL> result;
    result.reserve(distances.size());

    // Travel on the path only once, and pick the points for each (ascending) distance on the way
    int actual_delta = 0;
    coord_t travelled_distance = 0;
    coord_t segment_size = 0;
    for (const coord_t distance : distances)
    {
        while (travelled_distance < distance)
        {
            actual_delta += direction;
            segment_size = segments_sizes_[(vertex_index + actual_delta + size_delta + points_count) % points_count];
            travelled_distance += segment_size;
        }

        const Point2LL& next_pos = getPoints().at((vertex_index + actual_delta + points_count) % points_count);

        if (travelled_distance > distance) [[likely]]
        {
            // We have overtaken the required distance, go backward on the last segment
            const size_t prev = (vertex_index + actual_delta - direction + points_count) % points_count;
            const Point2LL& prev_pos = getPoints().at(prev);

            const Point2LL vector = next_pos - prev_pos;
            const Point2LL unit_vector = (vector * 1000) / segment_size;
            const Point2LL vector_delta = unit_vector * (segment_size - (travelled_distance - distance));
            result.push_back(prev_pos + vector_delta / 1000);
        }
        else
        {
            // Luckily, the required distance stops exactly on an existing point
            result.push_back(next_pos);
        }
    }

    return result;
}

} // namespace curaengine::types::scoring
