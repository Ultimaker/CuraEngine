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
 * Finds the corners of a closed path, with one single entry per corner whatever its rounding. Consecutive
 * vertices that keep turning in the same direction are grouped together (the "window" thus adapts to the corner),
 * and the corner position is the centroid of their turning angle. So a fillet gives one candidate at its center,
 * and a sharp corner gives one candidate on the vertex.
 * @param points The points of the closed path. It must not be explicitly closed (no repeated last point) and must not contain consecutive duplicates
 * @param min_vertex_turn Vertices turning by less than this angle (radians) are considered as straight (dead band), so they never belong to a corner and they end the current one
 * @param max_radius Two consecutive turning vertices belong to the same corner only if the radius of the arc between them is lower than this value. This radius is estimated by the
 * length of the segment between them divided by their average turning angle, so the tolerance adapts to the resolution of the path
 * @param max_sharp_turn A vertex that turns by more than this angle is a very sharp corner by itself, so it is never aggregated with any other vertex and always makes a single
 * corner, even if the vertices around it turn in the same direction
 * @param max_sharp_link_distance A very sharp vertex can still be aggregated with the vertex next to it if the segment between them is shorter than this distance, because corners that
 * are closer than the width of an extrusion line cannot be told apart when printing (this is typically the case of a chamfer, that is made of two vertices turning in the same direction)
 * @param max_turn_jump A vertex belongs to the current corner only if its turning angle does not differ by more than this value from the one predicted by the curvature
 * of the corner built so far. It separates a sharp turn from the smooth rounding that precedes or follows it, even if they turn in the same direction
 * @param max_corner_length A corner that extends over more than this length along the path is considered as too flat, and is discarded as a whole
 * @param length_penalty_reference Length at which the score of a corner is halved, favoring short corners. Disabled if <= 0
 * @return The corners, in the order they are met on the path
 */
std::vector<SeamCorner> findSeamCorners(
    const types::geometry::PointsSet& points,
    const types::AngleRadians& min_vertex_turn,
    const double max_radius,
    const types::AngleRadians& max_turn_jump,
    const types::AngleRadians& max_sharp_turn,
    const double max_sharp_link_distance,
    const double max_corner_length,
    const double length_penalty_reference);

} // namespace curaengine::seam
