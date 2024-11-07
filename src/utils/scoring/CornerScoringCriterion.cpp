// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/scoring/CornerScoringCriterion.h"

#include "geometry/PointsSet.h"
#include "utils/linearAlg2D.h"
#include "utils/math.h"


namespace cura
{

CornerScoringCriterion::CornerScoringCriterion(const PointsSet& points, const EZSeamCornerPrefType corner_preference)
    : points_(points)
    , corner_preference_(corner_preference)
    , segments_sizes_(points.size())
{
    // Pre-calculate the segments lengths because we are going to need them multiple times
    for (size_t i = 0; i < points.size(); ++i)
    {
        const Point2LL& here = points_.at(i);
        const Point2LL& next = points_.at((i + 1) % points_.size());
        const coord_t segment_size = vSize(next - here);
        segments_sizes_[i] = segment_size;
        total_length_ += segment_size;
    }
}

double CornerScoringCriterion::computeScore(const size_t candidate_index) const
{
    double corner_angle = cornerAngle(candidate_index);
    // angles < 0 are concave (left turning)
    // angles > 0 are convex (right turning)

    double score = 0.0;

    switch (corner_preference_)
    {
    case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_INNER:
        // Give advantage to concave corners. More advantage for sharper corners.
        score = cura::inverse_lerp(1.0, -1.0, corner_angle);
        break;
    case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_OUTER:
        // Give advantage to convex corners. More advantage for sharper corners.
        score = cura::inverse_lerp(-1.0, 1.0, corner_angle);
        break;
    case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_ANY:
        // Still give sharper corners more advantage.
        score = std::abs(corner_angle);
        break;
    case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_WEIGHTED:
        // Give sharper corners some advantage, but sharper concave corners even more.
        if (corner_angle < 0)
        {
            score = -corner_angle;
        }
        else
        {
            score = corner_angle / 2.0;
        }
        break;
    case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE:
    case EZSeamCornerPrefType::PLUGIN:
        break;
    }

    return score;
}

double CornerScoringCriterion::cornerAngle(size_t vertex_index, const coord_t angle_query_distance) const
{
    const coord_t bounded_distance = std::min(angle_query_distance, total_length_ / 2);
    const Point2LL& here = points_.at(vertex_index);
    const Point2LL next = findNeighbourPoint(vertex_index, bounded_distance);
    const Point2LL previous = findNeighbourPoint(vertex_index, -bounded_distance);

    double angle = LinearAlg2D::getAngleLeft(previous, here, next) - std::numbers::pi;

    return angle / std::numbers::pi;
}

Point2LL CornerScoringCriterion::findNeighbourPoint(size_t vertex_index, coord_t distance) const
{
    const int direction = distance > 0 ? 1 : -1;
    const int size_delta = distance > 0 ? -1 : 0;
    distance = std::abs(distance);

    // Travel on the path until we reach the distance
    int actual_delta = 0;
    coord_t travelled_distance = 0;
    coord_t segment_size = 0;
    while (travelled_distance < distance)
    {
        actual_delta += direction;
        segment_size = segments_sizes_[(vertex_index + actual_delta + size_delta + points_.size()) % points_.size()];
        travelled_distance += segment_size;
    }

    const Point2LL& next_pos = points_.at((vertex_index + actual_delta + points_.size()) % points_.size());

    if (travelled_distance > distance) [[likely]]
    {
        // We have overtaken the required distance, go backward on the last segment
        int prev = (vertex_index + actual_delta - direction + points_.size()) % points_.size();
        const Point2LL& prev_pos = points_.at(prev);

        const Point2LL vector = next_pos - prev_pos;
        const Point2LL unit_vector = (vector * 1000) / segment_size;
        const Point2LL vector_delta = unit_vector * (segment_size - (travelled_distance - distance));
        return prev_pos + vector_delta / 1000;
    }
    else
    {
        // Luckily, the required distance stops exactly on an existing point
        return next_pos;
    }
}

} // namespace cura
