// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <cstddef>
#include <vector>

#include "types/AngleRadians.h"
#include "types/Coord_t.h"
#include "types/geometry/Point2LL.h"

namespace curaengine::types::geometry
{
class PointsSet;
struct ExtrusionLine;
} // namespace curaengine::types::geometry

namespace curaengine::seam
{

/*! A single corner found on a closed path, which can be used as a seam candidate */
struct SeamCorner
{
    types::geometry::Point2LL position; // The position of the corner, at the center of its turning angle
    size_t segment_index; // Index of the input point that starts the segment containing the position, or of the point at the position if on_vertex is true
    bool on_vertex; // True if the position coincides with an existing input point, so no junction needs to be inserted
    double turn; // Total signed turn in radians. Positive is left-turning (concave), negative is right-turning (convex)
    types::coord_t extent; // Length of the path section over which the turn happens, in micrometers. Zero for a perfectly sharp corner
    double score; // Score of the corner in [0 ; max(weights)], the higher the sharper
    types::geometry::ExtrusionLine* extrusion_line{ nullptr }; // The extrusion line this corner belongs to, whose seam will be set at this corner. Not owned, it has to remain alive and in place
    size_t layer_index{ 0 }; // The index of the layer this corner is at
};

/*!
 * Finds the corners of a closed path, with one single entry per corner whatever its rounding. A corner is modeled as a fillet, which is an arc of circle prolonged by its two
 * tangents. Each turning vertex belongs to exactly one corner, and the fewest corners that describe the path within the maximum deviation are used. So a rounded corner gives one
 * candidate at the middle of its arc, a sharp corner gives one candidate on the vertex, and corners that are too close to be told apart are merged.
 * @param points The points of the closed path. It must not be explicitly closed (no repeated last point) and must not contain consecutive duplicates
 * @param min_vertex_turn Vertices turning by less than this angle (radians) are considered as straight (dead band), so they never belong to a corner and they end the current one
 * @param max_radius The radius of the arc of a corner cannot exceed this value, flatter arcs are not corners
 * @param max_deviation The maximum distance (micrometers) between the path and the fillet that models a corner. Corners that are so close that a single fillet fits them
 * within this distance are merged
 * @return The corners, in the order they are met on the path
 */
std::vector<SeamCorner> findSeamCorners(
    const types::geometry::PointsSet& points,
    const types::AngleRadians& min_vertex_turn,
    const double max_radius,
    const double max_deviation);

} // namespace curaengine::seam
