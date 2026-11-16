// Copyright (c) 2025 UltiMaker and contributors
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/ArcFitter.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace cura
{

namespace
{
constexpr double two_pi = 2.0 * std::numbers::pi;
constexpr double max_step_angle = 100.0 * std::numbers::pi / 180.0; // A single input segment should not span more than this.
constexpr double max_length_deviation = 0.03; // The arc may be at most 3% longer or shorter than the polyline it replaces.
constexpr double max_sweep = two_pi * 0.97; // Stay clear of a full circle, where start and end point coincide.

/*! Difference between two angles, wrapped to (-pi, pi]. */
double angleDelta(double from, double to)
{
    double delta = to - from;
    while (delta > std::numbers::pi)
    {
        delta -= two_pi;
    }
    while (delta <= -std::numbers::pi)
    {
        delta += two_pi;
    }
    return delta;
}
} // namespace

bool ArcFitter::tryFit(
    const std::vector<Point2LL>& points,
    const size_t first,
    const size_t last,
    const coord_t tolerance,
    const coord_t max_radius,
    const coord_t min_radius,
    Segment& result)
{
    // Circle through the first, middle and last point.
    const Point2LL& a = points[first];
    const Point2LL& b = points[(first + last) / 2];
    const Point2LL& c = points[last];

    const double bx = static_cast<double>(b.X - a.X);
    const double by = static_cast<double>(b.Y - a.Y);
    const double cx = static_cast<double>(c.X - a.X);
    const double cy = static_cast<double>(c.Y - a.Y);
    const double d = 2.0 * (bx * cy - by * cx);
    if (std::abs(d) < 1e-6)
    {
        return false; // Collinear: this is a straight line, not an arc.
    }
    const double b2 = bx * bx + by * by;
    const double c2 = cx * cx + cy * cy;
    const double ux = (cy * b2 - by * c2) / d;
    const double uy = (bx * c2 - cx * b2) / d;
    const double radius = std::hypot(ux, uy);
    if (radius > static_cast<double>(max_radius) || radius < static_cast<double>(min_radius))
    {
        return false;
    }
    const double center_x = static_cast<double>(a.X) + ux;
    const double center_y = static_cast<double>(a.Y) + uy;

    // The tolerance is the largest allowed distance between the arc and the original polyline. Half of it is spent on
    // the points not lying exactly on the circle, the other half on the arc bulging out between two points.
    const double point_tolerance = static_cast<double>(tolerance) / 2.0;
    const double bulge_tolerance = static_cast<double>(tolerance) / 2.0;

    // Every point has to be close to the circle, the points have to keep turning the same way and not too sharply.
    double polyline_length = 0.0;
    double sweep = 0.0;
    int direction = 0;
    double previous_angle = std::atan2(static_cast<double>(a.Y) - center_y, static_cast<double>(a.X) - center_x);
    for (size_t i = first + 1; i <= last; ++i)
    {
        const double px = static_cast<double>(points[i].X);
        const double py = static_cast<double>(points[i].Y);
        if (std::abs(std::hypot(px - center_x, py - center_y) - radius) > point_tolerance)
        {
            return false;
        }

        const double chord = std::hypot(px - static_cast<double>(points[i - 1].X), py - static_cast<double>(points[i - 1].Y));
        polyline_length += chord;

        // How far the arc runs from the straight segment between these two points (the sagitta of the chord).
        // Without this a corner made of a few points would also "fit" a circle and get rounded off.
        const double bulge = radius - std::sqrt(std::max(0.0, radius * radius - chord * chord / 4.0));
        if (bulge > bulge_tolerance)
        {
            return false;
        }

        const double angle = std::atan2(py - center_y, px - center_x);
        const double delta = angleDelta(previous_angle, angle);
        if (delta == 0.0 || std::abs(delta) > max_step_angle)
        {
            return false;
        }
        const int step_direction = delta > 0 ? 1 : -1;
        if (direction == 0)
        {
            direction = step_direction;
        }
        else if (step_direction != direction)
        {
            return false;
        }
        sweep += std::abs(delta);
        previous_angle = angle;
    }

    if (sweep > max_sweep || polyline_length <= 0.0)
    {
        return false;
    }
    const double arc_length = radius * sweep;
    if (std::abs(arc_length - polyline_length) > max_length_deviation * polyline_length)
    {
        return false;
    }

    result.first_index = first;
    result.last_index = last;
    result.is_arc = true;
    result.center_x = center_x;
    result.center_y = center_y;
    result.radius = radius;
    result.clockwise = direction < 0;
    result.sweep = sweep;
    return true;
}

std::vector<ArcFitter::Segment> ArcFitter::fit(const std::vector<Point2LL>& points, const coord_t tolerance, const coord_t max_radius, const coord_t min_radius)
{
    std::vector<Segment> segments;
    if (points.size() < 2)
    {
        return segments;
    }

    size_t i = 0;
    while (i + 1 < points.size())
    {
        // Grow an arc starting at point i for as long as one circle keeps fitting.
        Segment best;
        bool found = false;
        for (size_t j = i + 2; j < points.size(); ++j)
        {
            Segment candidate;
            if (! tryFit(points, i, j, tolerance, max_radius, min_radius, candidate))
            {
                // A longer arc that fails may still be followed by one that fits (rounding noise), but that is
                // rare and not worth the cost; stop at the first failure like the original algorithm.
                break;
            }
            best = candidate;
            found = true;
        }

        if (found)
        {
            segments.push_back(best);
            i = best.last_index;
        }
        else
        {
            // Not part of an arc: continue the current straight stretch, or start one.
            if (! segments.empty() && ! segments.back().is_arc)
            {
                segments.back().last_index = i + 1;
            }
            else
            {
                Segment line;
                line.first_index = i;
                line.last_index = i + 1;
                segments.push_back(line);
            }
            ++i;
        }
    }
    return segments;
}

} // namespace cura
