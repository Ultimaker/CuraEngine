// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef CORNERSCORINGCRITERION_H
#define CORNERSCORINGCRITERION_H

#include <stddef.h>

#include "geometry/Point2LL.h"
#include "settings/EnumSettings.h"
#include "utils/Coord_t.h"
#include "utils/scoring/ScoringCriterion.h"

namespace cura
{
class PointsSet;

/*!
 * Criterion that will give a score according to whether the point is creating a corner or lies on a flat line.
 * Depending on the given preference, concave or convex corners may get a higher score.
 */
class CornerScoringCriterion : public ScoringCriterion
{
private:
    const PointsSet& points_;
    const EZSeamCornerPrefType corner_preference_;
    std::vector<coord_t> segments_sizes_;
    coord_t total_length_{ 0 };

public:
    explicit CornerScoringCriterion(const PointsSet& points, const EZSeamCornerPrefType corner_preference);

    virtual double computeScore(const size_t candidate_index) const override;

private:
    /*!
     * Some models have very sharp corners, but also have a high resolution. If a sharp corner
     * consists of many points each point individual might have a shallow corner, but the
     * collective angle of all nearby points is greater. To counter this the cornerAngle is
     * calculated from two points within angle_query_distance of the query point, no matter
     * what segment this leads us to
     * \param vertex_index index of the query point
     * \param angle_query_distance query range (default to 1mm)
     * \return angle between the reference point and the two sibling points, weighed to [-1.0 ; 1.0]
     */
    double cornerAngle(size_t vertex_index, const coord_t angle_query_distance = 1000) const;

    /*!
     * Finds a neighbour point on the path, located before or after the given reference point. The neighbour point
     * is computed by travelling on the path and stopping when the distance has been reached, For example:
     * |------|---------|------|--------------*---|
     * H      A         B      C              N   D
     * In this case, H is the start point of the path and ABCD are the actual following points of the path.
     * The neighbour point N is found by reaching point D then going a bit backward on the previous segment.
     * This approach gets rid of the mesh actual resolution and gives a neighbour point that is on the path
     * at a given physical distance.
     * \param vertex_index The starting point index
     * \param distance The distance we want to travel on the path, which may be positive to go forward
     * or negative to go backward
     * \return The position of the path a the given distance from the reference point
     */
    Point2LL findNeighbourPoint(size_t vertex_index, coord_t distance) const;
};

} // namespace cura

#endif // CORNERSCORINGCRITERION_H
