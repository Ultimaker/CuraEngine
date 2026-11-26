// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "seam/seam.h"

#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <numbers>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <range/v3/algorithm/any_of.hpp>
#include <range/v3/algorithm/contains.hpp>
#include <range/v3/algorithm/find.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/view/drop_last.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/sliding.hpp>
#include <spdlog/spdlog.h>

#include "Application.h"
#include "ExtruderTrain.h"
#include "Slice.h"
#include "SliceDataStorage.h"
#include "SliceLayer.h"
#include "SliceMeshStorage.h"
#include "seam/seam_corners.h"
#include "types/AngleDegrees.h"
#include "types/CuraViz.h"
#include "types/EZSeamCornerPrefType.h"
#include "types/EZSeamType.h"
#include "types/OBJ.h"
#include "types/SVG.h"
#include "types/geometry/Point2D.h"
#include "types/geometry/PointsSet.h"
#include "types/geometry/conversions/Point2D_Point2LL.h"
#include "types/scoring/BestElementFinder.h"
#include "types/scoring/CornerScoringCriterion.h"
#include "types/scoring/DistanceScoringCriterion.h"
#include "types/scoring/ExclusionAreaScoringCriterion.h"
#include "types/scoring/RandomScoringCriterion.h"
#include "types/scoring/TextureScoringCriterion.h"
#include "types/scoring/VerticalAlignmentScoringCriterion.h"
#include "utils/linear_alg_2d.h"
#include "utils/math.h"


namespace curaengine::seam
{
using namespace types;
using namespace types::geometry;
using namespace types::geometry::conversions;
using namespace types::scoring;
using namespace utils;

struct AddSplitJunctionCandidate
{
    Point2LL reference_position;
    Point2LL split_position;
};

using SegmentEvaluator = std::function<std::optional<AddSplitJunctionCandidate>(const Point2LL&, const Point2LL&)>;

/*! Data of the segment that goes from a vertex to the next one */
struct Segment
{
    double length;
    Point2D direction; // Normalized direction
};

/*! The data of a closed path that is needed to find its corners */
struct PathData
{
    std::vector<Point2D> positions;
    std::vector<Segment> segments; // Segment i goes from vertex i to vertex i + 1
    std::vector<double> turns; // Signed turn of each vertex, positive is a left turn, which is concave
};

/*! A fillet is an arc of circle that is prolonged by its two tangents, which is how a rounded corner looks like */
struct Fillet
{
    Point2D tangent_in; // Where the arc starts
    Point2D direction_in;
    Point2D tangent_out; // Where the arc ends
    Point2D direction_out;
    Point2D center;
    double radius;
    double turn; // Signed turn of the arc
};

/*! The result of fitting a fillet on a group of consecutive vertices */
struct FilletFit
{
    double turn; // Total signed turn of the vertices
    double residual; // Maximum distance between the path and the fillet
    size_t nearest_vertex; // Start of the segment that contains the point of the path that is the closest to the apex of the fillet
    double nearest_distance; // Distance of that point to the start of the segment
};

static double cross(const Point2D& a, const Point2D& b)
{
    return a.x() * b.y() - a.y() * b.x();
}

static Point2D rotated(const Point2D& vector, const double angle)
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return Point2D(vector.x() * cosine - vector.y() * sine, vector.x() * sine + vector.y() * cosine);
}

static double distanceToRay(const Point2D& point, const Point2D& origin, const Point2D& direction)
{
    const Point2D vector = point - origin;
    return Point2D::dot(vector, direction) <= 0.0 ? vector.vSize() : std::abs(cross(direction, vector));
}

static double distanceToFillet(const Fillet& fillet, const Point2D& point)
{
    double distance = std::min(distanceToRay(point, fillet.tangent_in, -fillet.direction_in), distanceToRay(point, fillet.tangent_out, fillet.direction_out));

    if (fillet.radius > 0.0)
    {
        const Point2D from_center = point - fillet.center;
        const Point2D start = fillet.tangent_in - fillet.center;
        double angle = std::atan2(cross(start, from_center), Point2D::dot(start, from_center)) * sign(fillet.turn);
        if (angle < 0.0)
        {
            angle += 2.0 * std::numbers::pi;
        }

        if (angle <= std::abs(fillet.turn))
        {
            distance = std::min(distance, std::abs(from_center.vSize() - fillet.radius));
        }
    }

    return distance;
}

/*!
 * Fits a fillet on a group of consecutive vertices. The two tangents of the fillet are the directions of the path at the first
 * and last vertices of the group (the mean of the directions of the segments around them), which gives the apex. The radius is then
 * deduced from how deep the group is from the apex.
 * @param path The path
 * @param first The first vertex of the group
 * @param size The number of vertices in the group
 * @param max_radius See findSeamCorners()
 * @param max_deviation See findSeamCorners()
 * @return The fit, or nullopt if the vertices cannot be described by a single fillet
 */
static std::optional<FilletFit> fitFillet(const PathData& path, const size_t first, const size_t size, const double max_radius, const double max_deviation)
{
    const size_t count = path.positions.size();
    const size_t last = (first + size - 1) % count;
    const size_t previous = (first + count - 1) % count;

    double total_turn = 0.0;
    for (size_t i = 0; i < size; ++i)
    {
        total_turn += path.turns[(first + i) % count];
    }

    if (size == 1)
    {
        // A single vertex is a perfectly sharp corner
        return FilletFit{ .turn = total_turn, .residual = 0.0, .nearest_vertex = first, .nearest_distance = 0.0 };
    }

    // The tangents are the directions of the path at the first and last vertices of the group
    const Point2D direction_in = rotated(path.segments[previous].direction, path.turns[first] / 2.0);
    const Point2D direction_out = rotated(path.segments[last].direction, -path.turns[last] / 2.0);
    const double arc_turn = total_turn - (path.turns[first] + path.turns[last]) / 2.0;

    constexpr double max_arc_turn = std::numbers::pi - 0.1; // The tangents get parallel for a U-turn, so the apex is not defined
    if (std::abs(arc_turn) >= max_arc_turn)
    {
        return std::nullopt;
    }

    // The apex is the intersection of the tangents. It must be ahead of the first vertex and behind the last one
    const Point2D chord = path.positions[last] - path.positions[first];
    const double determinant = cross(direction_in, direction_out);
    const double along_in = cross(chord, direction_out) / determinant;
    const double along_out = cross(direction_in, chord) / determinant;
    if (along_in < -static_cast<double>(epsilon) || along_out < -static_cast<double>(epsilon))
    {
        return std::nullopt;
    }
    const Point2D apex = path.positions[first] + direction_in * std::max(along_in, 0.0);

    // How deep the path is from the apex, along the bisector of the tangents, gives the radius, because the middle of an arc is at a distance of
    // tangent_length * tan(turn / 4) from the apex
    const Point2D bisector = (direction_out - direction_in) / (direction_out - direction_in).vSize();
    double depth = std::numeric_limits<double>::max();
    size_t nearest_vertex = first;
    double nearest_distance = 0.0;
    for (size_t i = 0; i + 1 < size; ++i)
    {
        const size_t vertex = (first + i) % count;
        const Segment& segment = path.segments[vertex];
        const Point2D to_start = path.positions[vertex] - apex;
        const double denominator = cross(bisector, segment.direction);
        if (std::abs(denominator) < 1e-9)
        {
            continue;
        }
        const double along_bisector = cross(to_start, segment.direction) / denominator;
        const double along_segment = cross(to_start, bisector) / denominator;
        if (along_bisector >= 0.0 && along_bisector < depth && along_segment >= -static_cast<double>(epsilon) && along_segment <= segment.length + static_cast<double>(epsilon))
        {
            depth = along_bisector;
            nearest_vertex = vertex;
            nearest_distance = std::clamp(along_segment, 0.0, segment.length);
        }
    }

    if (depth == std::numeric_limits<double>::max())
    {
        return std::nullopt; // The bisector does not meet the path, so it is not a rounded corner
    }

    const double abs_turn = std::abs(arc_turn);
    const double tangent_length = depth / std::tan(abs_turn / 4.0);
    const double radius = tangent_length / std::tan(abs_turn / 2.0);
    if (radius > max_radius)
    {
        return std::nullopt;
    }

    Fillet fillet{ .tangent_in = apex - direction_in * tangent_length,
                   .direction_in = direction_in,
                   .tangent_out = apex + direction_out * tangent_length,
                   .direction_out = direction_out,
                   .center = Point2D(),
                   .radius = radius,
                   .turn = arc_turn };
    fillet.center = fillet.tangent_in + direction_in.rotated90CCW() * (sign(arc_turn) * radius);

    // The vertices and the segments between them must stay close to the fillet
    double residual = 0.0;
    for (size_t i = 0; i + 1 < size; ++i)
    {
        const size_t vertex = (first + i) % count;
        const Point2D& start = path.positions[vertex];
        const Point2D end = path.positions[(vertex + 1) % count];
        residual = std::max({ residual, distanceToFillet(fillet, start), distanceToFillet(fillet, (start + end) / 2.0), distanceToFillet(fillet, end) });
        if (residual > max_deviation)
        {
            return std::nullopt;
        }
    }

    return FilletFit{ .turn = total_turn, .residual = residual, .nearest_vertex = nearest_vertex, .nearest_distance = nearest_distance };
}

/*!
 * Converts a group of vertices that has been fitted into a corner
 * @param fit The fit of the vertices
 * @param first The first vertex of the group
 * @param size The number of vertices in the group
 * @param points The points of the path
 * @param path The data of the path
 * @param length_penalty_reference Length at which the score of a corner is halved
 */
static SeamCorner
    makeCornerFromFit(const FilletFit& fit, const size_t first, const size_t size, const PointsSet& points, const PathData& path, const double length_penalty_reference)
{
    const size_t count = points.size();
    const size_t vertex = fit.nearest_vertex;
    const double distance_to_next_vertex = path.segments[vertex].length - fit.nearest_distance;

    SeamCorner corner;
    if (fit.nearest_distance <= static_cast<double>(epsilon))
    {
        corner.position = points.at(vertex);
        corner.segment_index = vertex;
        corner.on_vertex = true;
    }
    else if (distance_to_next_vertex <= static_cast<double>(epsilon))
    {
        const size_t next_vertex = (vertex + 1) % count;
        corner.position = points.at(next_vertex);
        corner.segment_index = next_vertex;
        corner.on_vertex = true;
    }
    else
    {
        corner.position = points.at(vertex) + toPoint2LL(path.segments[vertex].direction * fit.nearest_distance);
        corner.segment_index = vertex;
        corner.on_vertex = false;
    }

    double extent = 0.0;
    for (size_t i = 0; i + 1 < size; ++i)
    {
        extent += path.segments[(first + i) % count].length;
    }
    corner.turn = fit.turn;
    corner.extent = std::llrint(extent);

    // Angle score, from 0 for the minimum turn to 1 for a U-turn
    const double angle_score = std::clamp(std::abs(fit.turn) / std::numbers::pi, 0.0, 1.0);

    // Favor corners that are concentrated on a short section
    const double length_score = length_penalty_reference > 0.0 ? 1.0 / (1.0 + extent / length_penalty_reference) : 1.0;

    corner.score = sign(fit.turn) * angle_score * length_score;

    return corner;
}

/*!
 * Finds the corners in a run of consecutive turning vertices. All the vertices of the run are covered by groups, each group being
 * a single corner. The fewest groups that fit are used, which merges the corners that are too close to be told apart.
 * @param run_first The first vertex of the run
 * @param run_size The number of vertices in the run
 * @param corners [out] The corners are appended in the order of the path
 * See findSeamCorners() for the other parameters
 */
static void findCornersOfRun(
    const PointsSet& points,
    const PathData& path,
    const size_t run_first,
    const size_t run_size,
    const double max_radius,
    const double max_deviation,
    std::vector<SeamCorner>& corners)
{
    const size_t count = path.positions.size();
    const double max_extent = max_radius * std::numbers::pi; // Length of half a circle of the maximum radius

    // The groups of vertices that can be described by a fillet, by index of their last vertex in the run
    struct Group
    {
        size_t size;
        FilletFit fit;
    };
    std::vector<std::vector<Group>> groups_by_end(run_size);

    for (size_t start = 0; start < run_size; ++start)
    {
        double extent = 0.0;
        size_t consecutive_misses = 0;
        for (size_t size = 1; start + size <= run_size; ++size)
        {
            if (size > 1)
            {
                extent += path.segments[(run_first + start + size - 2) % count].length;
                if (extent > max_extent)
                {
                    break;
                }
            }

            const std::optional<FilletFit> fit = fitFillet(path, (run_first + start) % count, size, max_radius, max_deviation);
            if (! fit.has_value())
            {
                // Adding more vertices to a group that does not fit hardly ever makes it fit again
                if (++consecutive_misses >= 2)
                {
                    break;
                }
                continue;
            }

            consecutive_misses = 0;
            groups_by_end[start + size - 1].push_back(Group{ .size = size, .fit = fit.value() });
        }
    }

    // Find the covering of the run with the fewest groups, and then with the smallest deviation
    struct Best
    {
        size_t group_count{ std::numeric_limits<size_t>::max() };
        double residual{ 0.0 };
        const Group* group{ nullptr };
    };
    std::vector<Best> best(run_size + 1);
    best[0] = Best{ .group_count = 0 };
    for (size_t end = 1; end <= run_size; ++end)
    {
        for (const Group& group : groups_by_end[end - 1])
        {
            const Best& before = best[end - group.size];
            const size_t group_count = before.group_count + 1;
            const double residual = before.residual + group.fit.residual;
            if (group_count < best[end].group_count || (group_count == best[end].group_count && residual < best[end].residual))
            {
                best[end] = Best{ .group_count = group_count, .residual = residual, .group = &group };
            }
        }
    }

    std::vector<SeamCorner> run_corners;
    for (size_t end = run_size; end > 0; end -= best[end].group->size)
    {
        const Group& group = *best[end].group;
        run_corners.push_back(makeCornerFromFit(group.fit, (run_first + end - group.size) % count, group.size, points, path, max_radius));
    }
    corners.insert(corners.end(), run_corners.rbegin(), run_corners.rend());
}

std::vector<SeamCorner> findSeamCorners(const PointsSet& points, const AngleRadians& min_vertex_turn, const double max_radius, const double max_deviation)
{
    const size_t points_count = points.size();
    if (points_count < 3)
    {
        return {};
    }

    PathData path;
    path.positions.reserve(points_count);
    for (size_t i = 0; i < points_count; ++i)
    {
        path.positions.push_back(toPoint2D(points.at(i)));
    }

    path.segments.resize(points_count);
    for (size_t i = 0; i < points_count; ++i)
    {
        const Point2D vector = path.positions[(i + 1) % points_count] - path.positions[i];
        const double length = vector.vSize();
        path.segments[i] = Segment{ .length = length, .direction = is_zero(length) ? Point2D() : vector / length };
    }

    path.turns.resize(points_count);
    for (size_t i = 0; i < points_count; ++i)
    {
        const Segment& previous = path.segments[(i + points_count - 1) % points_count];
        path.turns[i] = std::atan2(cross(previous.direction, path.segments[i].direction), Point2D::dot(previous.direction, path.segments[i].direction));
    }

    // Two consecutive vertices are linked if they turn in the same direction and are close enough to be on the same arc of the maximum radius
    std::vector<bool> is_linked_to_next(points_count);
    for (size_t i = 0; i < points_count; ++i)
    {
        const double turn = path.turns[i];
        const double next_turn = path.turns[(i + 1) % points_count];
        const bool is_turning = std::abs(turn) >= min_vertex_turn && std::abs(next_turn) >= min_vertex_turn && sign(turn) == sign(next_turn);
        is_linked_to_next[i] = is_turning && path.segments[i].length <= max_radius * (std::abs(turn) + std::abs(next_turn)) / 2.0;
    }

    // Start just after a segment that is not linked, so that no run is split by the start of the loop
    std::optional<size_t> start;
    for (size_t i = 0; i < points_count; ++i)
    {
        if (! is_linked_to_next[(i + points_count - 1) % points_count])
        {
            start = i;
            break;
        }
    }

    if (! start.has_value())
    {
        // The whole path is a single continuous arc, it cannot have any corner
        return {};
    }

    std::vector<SeamCorner> corners;
    for (size_t rank = 0; rank < points_count;)
    {
        const size_t vertex = (start.value() + rank) % points_count;
        if (std::abs(path.turns[vertex]) < min_vertex_turn)
        {
            ++rank;
            continue;
        }

        size_t run_end = rank;
        while (run_end + 1 < points_count && is_linked_to_next[(start.value() + run_end) % points_count])
        {
            ++run_end;
        }

        findCornersOfRun(points, path, vertex, run_end - rank + 1, max_radius, max_deviation, corners);
        rank = run_end + 1;
    }

    return corners;
}

/*!
 * Evaluates all the segments of the extrusion line with the given function, and inserts a junction in the best one (the shortest distance).
 * @param segment_evaluator Function called with the start and end points of each segment, which returns either nullopt if the segment is not a candidate, or the candidate data
 * @return The index of the junction at the best split position, or nullopt if no segment was a candidate
 */
static std::optional<size_t> insertBestNewJunction(ExtrusionLine& extrusion_line, const SegmentEvaluator& segment_evaluator)
{
    if (extrusion_line.size() < 3 || ! extrusion_line.is_closed_)
    {
        return std::nullopt;
    }

    struct BestCandidate
    {
        size_t start_index;
        Point2LL segment_start;
        Point2LL segment_end;
        Point2LL split_position;
        double distance_to_ref_squared;
    };

    std::optional<BestCandidate> best_candidate;

    // Extrusion lines are explicitly closed, so there is no need to loop
    for (const auto&& [segment_index, segment] : extrusion_line | ranges::views::sliding(2) | ranges::views::enumerate)
    {
        const Point2LL& segment_start = segment[0].p_;
        const Point2LL& segment_end = segment[1].p_;

        const std::optional<AddSplitJunctionCandidate> candidate = segment_evaluator(segment_start, segment_end);
        if (! candidate.has_value())
        {
            continue;
        }

        const double distance_to_ref_squared = vSize2(candidate->split_position - candidate->reference_position);
        if (! best_candidate.has_value() || distance_to_ref_squared < best_candidate->distance_to_ref_squared)
        {
            best_candidate = BestCandidate{ .start_index = segment_index,
                                            .segment_start = segment_start,
                                            .segment_end = segment_end,
                                            .split_position = candidate->split_position,
                                            .distance_to_ref_squared = distance_to_ref_squared };
        }
    }

    if (! best_candidate.has_value())
    {
        return std::nullopt;
    }

    const Point2LL& split_position = best_candidate->split_position;
    if (vSize2(split_position - best_candidate->segment_start) <= epsilon_squared)
    {
        return best_candidate->start_index;
    }
    if (vSize2(split_position - best_candidate->segment_end) <= epsilon_squared)
    {
        return best_candidate->start_index + 1;
    }

    return extrusion_line.insertJunction(best_candidate->start_index, split_position);
}

static void addCloseToSeamPositionCandidate(ExtrusionLine& extrusion_line, const std::vector<Point2LL>& last_layer_seams)
{
    for (const Point2LL& last_layer_seam : last_layer_seams)
    {
        insertBestNewJunction(
            extrusion_line,
            [&last_layer_seam](const Point2LL& segment_start, const Point2LL& segment_end) -> std::optional<AddSplitJunctionCandidate>
            {
                const Point2LL closest_on_segment = linear_alg_2d::getClosestOnLineSegment(last_layer_seam, segment_start, segment_end);
                return AddSplitJunctionCandidate{ .reference_position = last_layer_seam, .split_position = closest_on_segment };
            });
    }
}

static bool addFixedPositionCandidate(
    ExtrusionLine& extrusion_line,
    std::optional<size_t>& force_start_index,
    const EZSeamType z_seam_type,
    const Point2LL& z_seam_position,
    const Point2LL& models_center)
{
    if (z_seam_type != EZSeamType::UserSpecified)
    {
        return false;
    }

    // For user-defined seams you usually don't _actually_ want the _closest_ point, per-se,
    // since you want the seam-line to be continuous in 3D space.
    // To that end, take the center of the 3D model (not of the current polygon, as that would give the same problems)
    // and project the point along the ray from the center to the request_point.
    const Point2LL ray_origin = models_center;
    const Point2LL request_point = ray_origin + (z_seam_position - ray_origin) * 10;

    const std::optional<size_t> inserted_index = insertBestNewJunction(
        extrusion_line,
        [&ray_origin, &request_point](const Point2LL& segment_start, const Point2LL& segment_end) -> std::optional<AddSplitJunctionCandidate>
        {
            float t, u;
            if (! linear_alg_2d::segmentSegmentIntersection(ray_origin, request_point, segment_start, segment_end, t, u))
            {
                return std::nullopt;
            }

            const Point2LL intersection = ray_origin + (request_point - ray_origin) * t;
            return AddSplitJunctionCandidate{ .reference_position = request_point, .split_position = intersection };
        });

    if (! inserted_index.has_value())
    {
        return false;
    }

    force_start_index = inserted_index;
    return true;
}

static PointsSet makePositionCandidates(
    ExtrusionLine& extrusion_line,
    std::optional<size_t>& force_start_index,
    const EZSeamType z_seam_type,
    const Point2LL& z_seam_position,
    const bool z_seam_on_vertex,
    const Point2LL& models_center,
    const std::vector<Point2LL>& last_layer_seams)
{
    PointsSet points;

    if (! z_seam_on_vertex)
    {
        addCloseToSeamPositionCandidate(extrusion_line, last_layer_seams);
        addFixedPositionCandidate(extrusion_line, force_start_index, z_seam_type, z_seam_position, models_center);
    }

    for (const ExtrusionJunction& junction : extrusion_line)
    {
        points.push_back(junction.p_);
    }

    return points;
}

static void addExclusionAreaCriterion(BestElementFinder::CriteriaPass& pass, const PointsSet& points, const Shape& exclusion_area)
{
    if (! exclusion_area.empty())
    {
        constexpr double overhang_exclusion_weight{ 2.0 };
        pass.appendCriterion(std::make_shared<ExclusionAreaScoringCriterion>(points, exclusion_area), overhang_exclusion_weight);
    }
}

static BestElementFinder makeBestCandidateFinderOuterWall(
    BestElementFinder& best_candidate_finder,
    std::shared_ptr<TextureDataProvider> texture_data_provider,
    const PointsSet& points,
    const std::vector<Point2LL>& layer_below_seams,
    const EZSeamType seam_type,
    const EZSeamCornerPrefType corner_pref,
    const double z_seam_alignment_weight,
    const Point2LL& target_pos,
    const std::optional<size_t>& force_start_index,
    const coord_t wall_line_width_0,
    const Shape& seam_overhang_exclusion_areas)
{
    // ## Step 1: define the main criteria to be applied and their weights
    // Standard weight for the "main" selection criterion, depending on the selected strategy. There should be
    // exactly one calculation using this criterion.
    {
        BestElementFinder::CriteriaPass main_criteria_pass;
        main_criteria_pass.outsider_delta_threshold = z_seam_alignment_weight;

        if (texture_data_provider)
        {
            constexpr double texture_weight{ 5.0 };
            main_criteria_pass.appendCriterion(std::make_shared<TextureScoringCriterion>(points, texture_data_provider, "seam"), texture_weight);
        }

        BestElementFinder::WeighedCriterion main_criterion;

        if (force_start_index.has_value())
        {
            // Use a much smaller distance divider because we want points around the forced points to be filtered out very easily
            constexpr double distance_divider = 1.0;
            constexpr auto distance_type = DistanceScoringCriterion::DistanceType::Euclidian;
            main_criterion.criterion = std::make_shared<DistanceScoringCriterion>(points, points.at(force_start_index.value()), distance_type, distance_divider);
        }
        else if (seam_type == EZSeamType::UserSpecified)
        {
            main_criterion.criterion = std::make_shared<DistanceScoringCriterion>(points, target_pos);
        }
        else if (seam_type == EZSeamType::SharpestCorner && corner_pref != EZSeamCornerPrefType::Plugin)
        {
            main_criterion.criterion = std::make_shared<CornerScoringCriterion>(points, corner_pref, AngleDegrees{ 18 });
        }
        else if (seam_type == EZSeamType::Random)
        {
            main_criterion.criterion = std::make_shared<RandomScoringCriterion>();
        }

        if (main_criterion.criterion)
        {
            main_criteria_pass.criteria.push_back(main_criterion);
        }
        else
        {
            spdlog::warn("Missing main criterion calculator");
        }

        // Second criterion with higher weight to avoid overhanging areas
        addExclusionAreaCriterion(main_criteria_pass, points, seam_overhang_exclusion_areas);

        best_candidate_finder.appendCriteriaPass(main_criteria_pass);
    }

    best_candidate_finder.appendSingleCriterionPass(std::make_shared<VerticalAlignmentScoringCriterion>(points, layer_below_seams, wall_line_width_0 * 4));

    // ########## Step 2: add fallback passes for criteria with very similar scores (e.g. corner on a cylinder)
    if (seam_type == EZSeamType::SharpestCorner)
    {
        const AABB path_bounding_box(points);

        { // First fallback strategy is to take points on the back-most position
            const auto fallback_criterion = std::make_shared<DistanceScoringCriterion>(points, path_bounding_box.max_, DistanceScoringCriterion::DistanceType::YOnly);
            constexpr double outsider_delta_threshold = 0.01;
            best_candidate_finder.appendSingleCriterionPass(fallback_criterion, outsider_delta_threshold);
        }

        { // Second fallback strategy, in case we still have multiple points that are aligned on Y (e.g. cube), take the right-most point
            const auto fallback_criterion = std::make_shared<DistanceScoringCriterion>(points, path_bounding_box.max_, DistanceScoringCriterion::DistanceType::XOnly);
            best_candidate_finder.appendSingleCriterionPass(fallback_criterion);
        }
    }

    return best_candidate_finder;
}

static void makeBestCandidateFinderInnerWall(BestElementFinder& best_candidate_finder, const PointsSet& points, const std::vector<Point2LL>& layer_seams)
{
    // For inner walls, we only try to be close to an outer wall seam. If it is not the case, in the end the starting position will not be set and the 'shortest' strategy
    // will be applied, which is the best we can do in such cases anyway.
    best_candidate_finder.appendSingleCriterionPass(std::make_shared<DistanceScoringCriterion>(points, layer_seams));
}

/*!
 * Checks that the given position is close enough to at least one of the seam positions of the outer walls
 * @param position The candidate position to be checked
 * @param layer_seams The seam positions of the outer walls
 * @param max_distance The maximum allowed distance to any of the seam positions
 */
static bool isCloseToOuterWallSeam(const Point2LL& position, const std::vector<Point2LL>& layer_seams, const coord_t max_distance)
{
    const coord_t max_distance_squared = max_distance * max_distance;
    return ranges::any_of(
        layer_seams,
        [&position, max_distance_squared](const Point2LL& seam)
        {
            return vSize2(position - seam) <= max_distance_squared;
        });
}

static std::vector<std::vector<SeamCorner>> makeSeamCandidatesAcrossLayers(const SliceMeshStorage::Ptr& mesh, const double concave_angle_weight, const double convex_angle_weight)
{
    std::vector<std::vector<SeamCorner>> seam_corners(mesh->layers.size()); // Good-enough seam candidates, per layer
    const auto wall_line_width_0 = mesh->settings.get<coord_t>("wall_line_width_0");

    for (auto [layer_index, layer] : mesh->layers | ranges::views::enumerate)
    {
        std::vector<SeamCorner>& layer_seam_corners = seam_corners[layer_index];

        for (SliceLayerPart& part : layer.parts)
        {
            for (VariableWidthLines& extrusion_lines : part.wall_toolpaths)
            {
                for (ExtrusionLine& extrusion_line : extrusion_lines)
                {
                    if (! extrusion_line.is_outer_wall() || ! extrusion_line.is_closed_)
                    {
                        continue;
                    }

                    // std::optional<size_t> force_start_index;
                    // const PointsSet points{ makePositionCandidates(
                    //     extrusion_line,
                    //     force_start_index,
                    //     seam_type,
                    //     target_pos,
                    //     z_seam_on_vertex,
                    //     models_center,
                    //     extrusion_line.is_outer_wall() ? layer_seams : std::vector<Point2LL>{}) };
                    PointsSet points;
                    for (const ExtrusionJunction& junction : extrusion_line | ranges::views::drop_last(1))
                    {
                        points.push_back(junction.p_);
                    }

                    BestElementFinder best_candidate_finder;
                    Shape seam_overhang_exclusion_areas; // Don't move to inner scope, it is used as a reference in the criterion and has to remain alive long enough

                    // if (! machine_borders.empty())
                    // {
                    //     seam_overhang_exclusion_areas = machine_borders.difference(part.getSupportedRegion(layer_height, seam_overhang_angle));
                    // }
                    //
                    // makeBestCandidateFinderOuterWall(
                    //     best_candidate_finder,
                    //     layer.texture_data_provider_,
                    //     points,
                    //     layer_seams,
                    //     seam_type,
                    //     corner_pref,
                    //     z_seam_alignment_weight,
                    //     target_pos,
                    //     force_start_index,
                    //     wall_line_width_0,
                    //     seam_overhang_exclusion_areas);

                    // All lengths are in micrometers, all angles in radians
                    const AngleRadians min_vertex_turn{ AngleDegrees{ 1 } }; // 1°, below this a turn is considered as straight
                    constexpr double max_radius = 2000.0; // Arcs flatter than a 2 mm radius are not corners
                    const double max_deviation
                        = static_cast<double>(wall_line_width_0) * 0.25; // Corners that are closer than this deviation from a single rounded corner cannot be told apart

                    std::vector<SeamCorner> corners = findSeamCorners(points, min_vertex_turn, max_radius, max_deviation);

                    // static SVG svg("/tmp/seam_polygons.svg", AABB(Point2LL(0, 0), Point2LL(400000, 400000)), 0.001);
                    const auto layer_z_debug = { 5000, 5200 };
                    if (ranges::contains(layer_z_debug, layer.printZ))
                    {
                        CuraViz::send(extrusion_line, "extrusion");

                        // svg.write(extrusion_line.toPolygon(), { .line = SVG::LineAttributes{ SVG::Color::Red, 0.1 } });
                        // static std::ofstream wkt_file("/tmp/seam_polygons.wkt", std::ios::app);
                        // Shape shape;
                        // shape.push_back(extrusion_line.toPolygon());
                        // shape.writeWkt(wkt_file);
                        // wkt_file << '\n';
                    }
                    for (SeamCorner& corner : corners)
                    {
                        // angles > 0 are concave (left turning)
                        // angles < 0 are convex (right turning)
                        const bool convex = corner.score < 0;
                        corner.score *= convex ? convex_angle_weight : concave_angle_weight;
                        if (ranges::contains(layer_z_debug, layer.printZ))
                        {
                            CuraViz::send(corner.position, fmt::format("corner_{}_{}", layer_index, corner.score));
                        }

                        if (std::abs(corner.score) > 0.15)
                        {
                            if (ranges::contains(layer_z_debug, layer.printZ))
                            {
                                // svg.write(corner.position, SVG::VerticesAttributes{ SVG::Color::Blue, 0.05 });
                            }
                            SeamCorner& layer_seam_corner = layer_seam_corners.emplace_back(std::move(corner));
                            layer_seam_corner.extrusion_line = &extrusion_line;
                            layer_seam_corner.layer_index = layer_index;
                        }
                    }
                }
            }
        }
    }

    return seam_corners;
}

struct SeamCornerNode;

/*! A link from a seam corner to a corner of the layer above that it could be vertically aligned with */
struct SeamCornerLink
{
    std::shared_ptr<SeamCornerNode> node; // The corner above
    double weight; // 1 if the corners are perfectly aligned, down to 0 when they are as far as the maximum distance allowed for the link
};

/*! A seam corner of a layer, with the corners of the layer above that it could be vertically aligned with */
struct SeamCornerNode
{
    const SeamCorner* seam_corner; // Not owned, it points to the input corners, which have to remain alive and unchanged
    std::vector<SeamCornerLink> seam_corners_above;
};

/*!
 * Links the seam corners of each layer to the nearby seam corners of the layer just above, which builds all the possible "strings" of vertically aligned corners.
 * A node may be linked to several corners above, and a corner of the layer above may be linked by several nodes below, so this is a graph where all
 * the paths that go upward from a node are the potential strings.
 * @param seam_corners_per_layer The seam corners of each layer, in layer order. They have to remain alive and unchanged as long as the returned nodes are used
 * @param max_distance The maximum horizontal distance between two corners of consecutive layers to be linked. The weight of a link decreases linearly with the distance, from 1
 * for corners that are perfectly aligned to 0 for corners that are at this distance
 * @return The root nodes, which are the corners that no corner of the layer below is linked to (all the corners of the first layer, and the ones that start a new string
 * higher). All the other nodes are reachable from them, and are kept alive by them
 */
static std::vector<std::shared_ptr<SeamCornerNode>> makeSeamCornerTree(const std::vector<std::vector<SeamCorner>>& seam_corners_per_layer, const coord_t max_distance)
{
    const coord_t max_distance_squared = max_distance * max_distance;

    std::vector<std::vector<std::shared_ptr<SeamCornerNode>>> nodes_per_layer;
    nodes_per_layer.reserve(seam_corners_per_layer.size());

    for (const std::vector<SeamCorner>& layer_seam_corners : seam_corners_per_layer)
    {
        std::vector<std::shared_ptr<SeamCornerNode>>& layer_nodes = nodes_per_layer.emplace_back();
        layer_nodes.reserve(layer_seam_corners.size());

        for (const SeamCorner& seam_corner : layer_seam_corners)
        {
            layer_nodes.push_back(std::make_shared<SeamCornerNode>(SeamCornerNode{ .seam_corner = &seam_corner, .seam_corners_above = {} }));
        }
    }

    // Tells for each node whether at least one corner of the layer below is linked to it. The first layer has no layer below, so all its nodes are roots
    std::vector<std::vector<bool>> has_parent_per_layer;
    has_parent_per_layer.reserve(nodes_per_layer.size());
    for (const std::vector<std::shared_ptr<SeamCornerNode>>& layer_nodes : nodes_per_layer)
    {
        has_parent_per_layer.emplace_back(layer_nodes.size(), false);
    }

    // There is a small number of corners per layer, so a direct comparison of each corner with those of the next layer is good enough
    for (size_t layer_index = 0; layer_index + 1 < nodes_per_layer.size(); ++layer_index)
    {
        const std::vector<std::shared_ptr<SeamCornerNode>>& layer_above_nodes = nodes_per_layer[layer_index + 1];
        std::vector<bool>& has_parent_above = has_parent_per_layer[layer_index + 1];

        for (const std::shared_ptr<SeamCornerNode>& node : nodes_per_layer[layer_index])
        {
            for (const auto& [node_above_index, node_above] : layer_above_nodes | ranges::views::enumerate)
            {
                const coord_t distance_squared = vSize2(node_above->seam_corner->position - node->seam_corner->position);
                if (distance_squared <= max_distance_squared)
                {
                    const double distance = std::sqrt(static_cast<double>(distance_squared));
                    const double weight = max_distance > 0 ? 1.0 - distance / static_cast<double>(max_distance) : 1.0;
                    node->seam_corners_above.push_back(SeamCornerLink{ .node = node_above, .weight = weight });
                    has_parent_above[node_above_index] = true;
                }
            }
        }
    }

    std::vector<std::shared_ptr<SeamCornerNode>> root_nodes;
    for (const auto& [layer_index, layer_nodes] : nodes_per_layer | ranges::views::enumerate)
    {
        for (const auto& [node_index, node] : layer_nodes | ranges::views::enumerate)
        {
            if (! has_parent_per_layer[layer_index][node_index])
            {
                root_nodes.push_back(node);
            }
        }
    }

    return root_nodes;
}

/*! A vertical string of seam corners, from a root node up to a node that has no corner above */
struct SeamCornersString
{
    std::shared_ptr<SeamCornerNode> root_node;
    std::vector<size_t> corners_path; // When reaching nodes that have multiple children, this is the index of the child this path moves to
    double total_score{ 0.0 }; // Score of the string, which depends on the alignment weight. See findBestStringFromCornersTree()
};

/*!
 * Finds the best vertical string of corners, which is the path that goes from one of the root nodes up to a node that has no corner above, and that has the highest total
 * score. With S the sum of the absolute scores of the corners of the string, and N the number of corners, the total score of a string is S / N^(1 - alignment weight). When the
 * alignment weight is 1, this is the sum, which favors long strings. When it is 0, this is the average of the scores, so the length doesn't matter. The alignment weight also
 * tells how much the links count: the score of a corner is multiplied by the weight of the link that goes to it from the corner below (so that well aligned corners count more),
 * but this influence fades to nothing when the alignment weight goes to 0. The score of the root corner always counts fully.
 * The graph can contain a huge number of paths, so they are not listed. Instead, since the strings go up one layer at a time, the length of a string is determined by its
 * first and last layers. So the best path from each node to each of the layers it can end at is calculated once from the top to the bottom, and reused by all the paths that
 * join this node.
 * @param root_nodes The root nodes of the corners graph
 * @param z_seam_alignment_weight The importance of the alignment, between 0 and 1
 * @return The best string, or nullopt if there is no root node. If several strings have the same score, the first found is returned
 */
static std::optional<SeamCornersString> findBestStringFromCornersTree(const std::vector<std::shared_ptr<SeamCornerNode>>& root_nodes, const double z_seam_alignment_weight)
{
    // Gather all the nodes that can be reached from the root nodes, without any recursion
    std::vector<const SeamCornerNode*> nodes;
    std::unordered_set<const SeamCornerNode*> visited_nodes;
    std::vector<const SeamCornerNode*> pending_nodes;

    for (const std::shared_ptr<SeamCornerNode>& root_node : root_nodes)
    {
        if (visited_nodes.insert(root_node.get()).second)
        {
            pending_nodes.push_back(root_node.get());
        }
    }

    while (! pending_nodes.empty())
    {
        const SeamCornerNode* node = pending_nodes.back();
        pending_nodes.pop_back();
        nodes.push_back(node);

        for (const SeamCornerLink& link_above : node->seam_corners_above)
        {
            if (visited_nodes.insert(link_above.node.get()).second)
            {
                pending_nodes.push_back(link_above.node.get());
            }
        }
    }

    // The corners above a node are always in the next layer, so by starting from the top layer, the best path of all the corners above a node is already known when it is evaluated
    ranges::sort(
        nodes,
        [](const SeamCornerNode* node1, const SeamCornerNode* node2)
        {
            return node1->seam_corner->layer_index > node2->seam_corner->layer_index;
        });

    const double alignment_weight = std::clamp(z_seam_alignment_weight, 0.0, 1.0);

    // The best path from a node to the top, for a given layer where the path ends. All the paths go up one layer at a time, so the end layer also gives the length of the path
    struct BestPath
    {
        size_t end_layer; // The layer of the last corner of the path
        double score_above; // Weighted sum of the absolute scores of the corners of the best path to this end layer, this node excluded
        size_t child_index; // The index, in the corners above, of the child that this best path goes through
    };

    std::unordered_map<const SeamCornerNode*, std::vector<BestPath>> best_paths;
    best_paths.reserve(nodes.size());

    for (const SeamCornerNode* node : nodes)
    {
        std::vector<BestPath> node_best_paths;

        if (node->seam_corners_above.empty())
        {
            node_best_paths.push_back(BestPath{ .end_layer = node->seam_corner->layer_index, .score_above = 0.0, .child_index = 0 });
        }

        for (const auto& [child_index, link] : node->seam_corners_above | ranges::views::enumerate)
        {
            // The influence of the link fades when we don't care about the alignment
            const double link_weight = 1.0 - alignment_weight * (1.0 - link.weight);
            const double child_score = link_weight * std::abs(link.node->seam_corner->score);

            for (const BestPath& child_best_path : best_paths.at(link.node.get()))
            {
                const double score_through_child = child_score + child_best_path.score_above;

                // There are few possible end layers per node, so a linear search is fine
                auto same_end = ranges::find(node_best_paths, child_best_path.end_layer, &BestPath::end_layer);
                if (same_end == node_best_paths.end())
                {
                    node_best_paths.push_back(BestPath{ .end_layer = child_best_path.end_layer, .score_above = score_through_child, .child_index = child_index });
                }
                else if (score_through_child > same_end->score_above)
                {
                    *same_end = BestPath{ .end_layer = child_best_path.end_layer, .score_above = score_through_child, .child_index = child_index };
                }
            }
        }

        best_paths.emplace(node, std::move(node_best_paths));
    }

    // Select the best root and the best end layer for it. The root has no link that goes to it, so its own score is not weighted
    std::shared_ptr<SeamCornerNode> best_root_node;
    size_t best_end_layer = 0;
    double best_total_score = 0.0;

    for (const std::shared_ptr<SeamCornerNode>& root_node : root_nodes)
    {
        const size_t root_layer = root_node->seam_corner->layer_index;

        for (const BestPath& best_path : best_paths.at(root_node.get()))
        {
            const double corners_count = static_cast<double>(best_path.end_layer - root_layer + 1);
            const double total_score = (std::abs(root_node->seam_corner->score) + best_path.score_above) / std::pow(corners_count, 1.0 - alignment_weight);

            if (! best_root_node || total_score > best_total_score)
            {
                best_root_node = root_node;
                best_end_layer = best_path.end_layer;
                best_total_score = total_score;
            }
        }
    }

    if (! best_root_node)
    {
        return std::nullopt;
    }

    // Build the string by following the best children from the root, towards the best end layer
    SeamCornersString best_string{ .root_node = best_root_node, .corners_path = {}, .total_score = best_total_score };

    for (const SeamCornerNode* node = best_root_node.get(); ! node->seam_corners_above.empty();)
    {
        const std::vector<BestPath>& node_best_paths = best_paths.at(node);
        const size_t child_index = ranges::find(node_best_paths, best_end_layer, &BestPath::end_layer)->child_index;

        // The path only stores the choices that are made at the nodes that have multiple children
        if (node->seam_corners_above.size() > 1)
        {
            best_string.corners_path.push_back(child_index);
        }

        node = node->seam_corners_above[child_index].node.get();
    }

    return best_string;
}

/*!
 * Lists the corners of a string, from the bottom to the top
 * @param string The string, as built by makeStringFromCornersTree()
 * @return The corners of the string, in the order they are met when going upward
 */
static std::vector<const SeamCorner*> getCornersOfString(const SeamCornersString& string)
{
    std::vector<const SeamCorner*> corners;

    size_t path_index = 0;
    for (const SeamCornerNode* node = string.root_node.get(); node != nullptr;)
    {
        corners.push_back(node->seam_corner);

        if (node->seam_corners_above.empty())
        {
            break;
        }

        // The path only stores the choices that were made at the nodes that have multiple children
        const size_t child_index = node->seam_corners_above.size() > 1 ? string.corners_path[path_index++] : 0;
        node = node->seam_corners_above[child_index].node.get();
    }

    return corners;
}

/*!
 * Makes the extrusion line of the given corner start at the position of this corner
 */
static void applySeamCorner(const SeamCorner& corner)
{
    ExtrusionLine& extrusion_line = *corner.extrusion_line;

    size_t start_index = corner.segment_index;
    if (! corner.on_vertex)
    {
        start_index = extrusion_line.insertJunction(corner.segment_index, corner.position);
    }

    extrusion_line.setStartJunction(start_index);
    extrusion_line.has_start_position_set_ = true;
}

/*!
 * Gets the number of different junctions of a closed extrusion line. Closed lines are explicitly closed, so the last junction is the same as the first one
 */
static size_t getUniqueJunctionsCount(const ExtrusionLine& extrusion_line)
{
    return extrusion_line.size() > 1 && extrusion_line.front().p_ == extrusion_line.back().p_ ? extrusion_line.size() - 1 : extrusion_line.size();
}

/*!
 * Finds the position of an extrusion line that is the closest to one of the given seams, and returns the index of the junction at this position. If the position is not
 * on an existing junction, a new one is inserted, unless it is requested to stick to the existing junctions.
 * @param extrusion_line The closed extrusion line to search a position in
 * @param seams The positions of the seams (typically the ones of the layer below or above) that the line should be aligned with
 * @param max_distance The maximum distance between the seam and the position found on the line
 * @param z_seam_on_vertex If true, the position is always on an existing junction, otherwise it is the closest point of the line
 * @return The index of the junction, or nullopt if no position of the line is close enough to a seam
 */
static std::optional<size_t> findJunctionCloseToSeams(ExtrusionLine& extrusion_line, const std::vector<Point2LL>& seams, const coord_t max_distance, const bool z_seam_on_vertex)
{
    const coord_t max_distance_squared = max_distance * max_distance;

    if (z_seam_on_vertex)
    {
        const size_t unique_junctions_count = getUniqueJunctionsCount(extrusion_line);

        std::optional<size_t> best_index;
        coord_t best_distance_squared = max_distance_squared;
        for (size_t junction_index = 0; junction_index < unique_junctions_count; ++junction_index)
        {
            for (const Point2LL& seam : seams)
            {
                const coord_t distance_squared = vSize2(extrusion_line[junction_index].p_ - seam);
                if (distance_squared <= best_distance_squared)
                {
                    best_index = junction_index;
                    best_distance_squared = distance_squared;
                }
            }
        }

        return best_index;
    }

    return insertBestNewJunction(
        extrusion_line,
        [&seams, max_distance_squared](const Point2LL& segment_start, const Point2LL& segment_end) -> std::optional<AddSplitJunctionCandidate>
        {
            // For this segment, keep the seam that is the closest to it, if there is any close enough
            std::optional<AddSplitJunctionCandidate> best_candidate;
            coord_t best_distance_squared = max_distance_squared;
            for (const Point2LL& seam : seams)
            {
                const Point2LL closest_on_segment = linear_alg_2d::getClosestOnLineSegment(seam, segment_start, segment_end);
                const coord_t distance_squared = vSize2(closest_on_segment - seam);
                if (distance_squared <= best_distance_squared)
                {
                    best_candidate = AddSplitJunctionCandidate{ .reference_position = seam, .split_position = closest_on_segment };
                    best_distance_squared = distance_squared;
                }
            }
            return best_candidate;
        });
}

/*!
 * Sets the seam of the outer walls of a layer that don't have any, by projecting the seams of another layer onto them. The walls that already have a seam are not changed.
 * @param source_layer The layer that holds the seams to be projected
 * @param target_layer The layer whose outer walls without seam will get one, if a seam of the source layer is close enough
 * @param max_distance The maximum distance between the projected seam and the new seam position
 * @param z_seam_on_vertex Whether the new seam has to be on an existing junction
 */
static void projectSeamsOnLayer(const SliceLayer& source_layer, SliceLayer& target_layer, const coord_t max_distance, const bool z_seam_on_vertex)
{
    std::vector<Point2LL> seams;
    for (const SliceLayerPart& part : source_layer.parts)
    {
        for (const VariableWidthLines& extrusion_lines : part.wall_toolpaths)
        {
            for (const ExtrusionLine& extrusion_line : extrusion_lines)
            {
                if (extrusion_line.is_outer_wall() && extrusion_line.is_closed_ && extrusion_line.has_start_position_set_ && ! extrusion_line.empty())
                {
                    seams.push_back(extrusion_line.front().p_);
                }
            }
        }
    }

    if (seams.empty())
    {
        return;
    }

    for (SliceLayerPart& part : target_layer.parts)
    {
        for (VariableWidthLines& extrusion_lines : part.wall_toolpaths)
        {
            for (ExtrusionLine& extrusion_line : extrusion_lines)
            {
                if (! extrusion_line.is_outer_wall() || ! extrusion_line.is_closed_ || extrusion_line.has_start_position_set_)
                {
                    continue;
                }

                if (const std::optional<size_t> start_index = findJunctionCloseToSeams(extrusion_line, seams, max_distance, z_seam_on_vertex))
                {
                    extrusion_line.setStartJunction(start_index.value());
                    extrusion_line.has_start_position_set_ = true;
                }
            }
        }
    }
}

/*!
 * Gives a seam to the outer walls that don't have any yet, by propagating the existing seams from layer to layer, first from the bottom to the top, and then from the top to
 * the bottom. The walls that cannot be reached because no seam is close enough are left unchanged.
 */
static void propagateSeamsAcrossLayers(SliceMeshStorage::Ptr& mesh, const coord_t max_distance, const bool z_seam_on_vertex)
{
    std::vector<SliceLayer>& layers = mesh->layers;

    for (size_t layer_index = 1; layer_index < layers.size(); ++layer_index)
    {
        projectSeamsOnLayer(layers[layer_index - 1], layers[layer_index], max_distance, z_seam_on_vertex);
    }

    for (size_t layer_index = layers.size(); layer_index-- > 1;)
    {
        projectSeamsOnLayer(layers[layer_index], layers[layer_index - 1], max_distance, z_seam_on_vertex);
    }
}

/*!
 * Sets the seam of the outer walls of a layer that don't have any, on a random junction
 */
static void setRandomSeamsOnLayer(SliceLayer& layer)
{
    for (SliceLayerPart& part : layer.parts)
    {
        for (VariableWidthLines& extrusion_lines : part.wall_toolpaths)
        {
            for (ExtrusionLine& extrusion_line : extrusion_lines)
            {
                if (! extrusion_line.is_outer_wall() || ! extrusion_line.is_closed_ || extrusion_line.has_start_position_set_)
                {
                    continue;
                }

                const size_t junctions_count = getUniqueJunctionsCount(extrusion_line);
                if (junctions_count == 0)
                {
                    continue;
                }

                const size_t junction_index = std::min(static_cast<size_t>(randf<double>() * static_cast<double>(junctions_count)), junctions_count - 1);
                extrusion_line.setStartJunction(junction_index);
                extrusion_line.has_start_position_set_ = true;
            }
        }
    }
}

/*!
 * Gives a seam to all the outer walls that still don't have any, which are the ones that have no corner to align to and no seam close enough to be projected from.
 * Going from the bottom to the top, the seams of the layer below are projected as usual, and the walls that still have no seam get a random one. This random seam is then
 * used as a starting point by the layers above to align with it.
 */
static void fillRemainingSeams(SliceMeshStorage::Ptr& mesh, const coord_t max_distance, const bool z_seam_on_vertex)
{
    std::vector<SliceLayer>& layers = mesh->layers;

    for (size_t layer_index = 0; layer_index < layers.size(); ++layer_index)
    {
        if (layer_index > 0)
        {
            projectSeamsOnLayer(layers[layer_index - 1], layers[layer_index], max_distance, z_seam_on_vertex);
        }

        setRandomSeamsOnLayer(layers[layer_index]);
    }
}

static void calculateWallsSeamPosition(
    SliceMeshStorage::Ptr& mesh,
    const std::vector<Point2LL>& layer_seams,
    const EZSeamType seam_type,
    const double concave_angle_weight,
    const double convex_angle_weight,
    const double z_seam_alignment_weight,
    const Point2LL& target_pos,
    const coord_t wall_line_width_0,
    const bool z_seam_on_vertex,
    const AngleDegrees& seam_overhang_angle,
    const coord_t layer_height,
    const Shape& machine_borders,
    const Point2LL& models_center,
    const coord_t max_inner_wall_seam_distance,
    const bool outer_walls)
{
    std::vector<std::vector<SeamCorner>> remaining_seam_corners = makeSeamCandidatesAcrossLayers(mesh, concave_angle_weight, convex_angle_weight);
    // const coord_t max_alignment_distance = wall_line_width_0 * 4; // Used both to link the corners of consecutive layers and to project the seams from a layer to the next one
    const coord_t max_alignment_distance = 1600; // Used both to link the corners of consecutive layers and to project the seams from a layer to the next one

    OBJ mesh_obj("/tmp/seam.obj", 0.1);
    mesh_obj.write(Application::getInstance().current_slice_->scene.mesh_groups.front().meshes.front());
    for (const auto& [layer_index, corner_per_layer] : remaining_seam_corners | ranges::views::enumerate)
    {
        const coord_t z = mesh->layers[layer_index].printZ;
        for (const auto& corner : corner_per_layer)
        {
            mesh_obj.writeSphere(Point3D(Point3LL(corner.position, z)), 0.1, SVG::Color::Red);
        }
    }

    // Each loop applies the best string to the extrusion lines. Since an extrusion line can have only one seam, the corners of the lines that have been done are removed, which
    // cuts or discards some of the remaining strings. Rebuilding the strings from the remaining corners is the simplest way to get the new valid ones. Each loop removes at
    // least one line, so this always ends
    while (true)
    {
        const std::vector<std::shared_ptr<SeamCornerNode>> seam_corners_tree = makeSeamCornerTree(remaining_seam_corners, max_alignment_distance);
        const std::optional<SeamCornersString> best_string = findBestStringFromCornersTree(seam_corners_tree, z_seam_alignment_weight);

        if (! best_string.has_value())
        {
            break;
        }

        std::unordered_set<const ExtrusionLine*> done_extrusion_lines;
        for (const SeamCorner* corner : getCornersOfString(best_string.value()))
        {
            applySeamCorner(*corner);
            done_extrusion_lines.insert(corner->extrusion_line);
        }

        for (std::vector<SeamCorner>& layer_seam_corners : remaining_seam_corners)
        {
            std::erase_if(
                layer_seam_corners,
                [&done_extrusion_lines](const SeamCorner& corner)
                {
                    return done_extrusion_lines.contains(corner.extrusion_line);
                });
        }
    }

    // The layers that have no corner to align with don't have seams yet, so get them from the neighbour layers
    propagateSeamsAcrossLayers(mesh, max_alignment_distance, z_seam_on_vertex);

    // Some lines may still have no seam, because there was no corner nor any other seam close enough
    fillRemainingSeams(mesh, max_alignment_distance, z_seam_on_vertex);
}

#if 0
static std::vector<Point2LL> calculateWallsSeamPosition(
    SliceMeshStorage::Ptr& mesh,
    const std::vector<Point2LL>& layer_seams,
    const EZSeamType seam_type,
    const EZSeamCornerPrefType corner_pref,
    const double z_seam_alignment_weight,
    const Point2LL& target_pos,
    const coord_t wall_line_width_0,
    const bool z_seam_on_vertex,
    const AngleDegrees& seam_overhang_angle,
    const coord_t layer_height,
    const Shape& machine_borders,
    const Point2LL& models_center,
    const coord_t max_inner_wall_seam_distance,
    const bool outer_walls)
{
    std::vector<Point2LL> new_layer_seams;

    std::vector<std::vector<SeamCorner>> seam_corners = makeSeamCandidatesAcrossLayers(mesh);

    for (auto [layer_index, layer] : mesh->layers | ranges::views::enumerate)
    {
        std::vector<SeamCorner>& layer_seam_corners = seam_corners[layer_index];

        for (SliceLayerPart& part : layer.parts)
        {
            for (VariableWidthLines& extrusion_lines : part.wall_toolpaths)
            {
                for (ExtrusionLine& extrusion_line : extrusion_lines)
                {
                    if (extrusion_line.is_outer_wall() != outer_walls || ! extrusion_line.is_closed_) // Only inner walls may be non-closed, so for those we can't do much
                    {
                        continue;
                    }

                    std::optional<size_t> force_start_index;
                    // const PointsSet points{ makePositionCandidates(
                    //     extrusion_line,
                    //     force_start_index,
                    //     seam_type,
                    //     target_pos,
                    //     z_seam_on_vertex,
                    //     models_center,
                    //     extrusion_line.is_outer_wall() ? layer_seams : std::vector<Point2LL>{}) };
                    PointsSet points;
                    for (const ExtrusionJunction& junction : extrusion_line | ranges::views::drop_last(1))
                    {
                        points.push_back(junction.p_);
                    }

                    BestElementFinder best_candidate_finder;
                    Shape seam_overhang_exclusion_areas; // Don't move to inner scope, it is used as a reference in the criterion and has to remain alive long enough
                    if (outer_walls)
                    {
                        // if (! machine_borders.empty())
                        // {
                        //     seam_overhang_exclusion_areas = machine_borders.difference(part.getSupportedRegion(layer_height, seam_overhang_angle));
                        // }
                        //
                        // makeBestCandidateFinderOuterWall(
                        //     best_candidate_finder,
                        //     layer.texture_data_provider_,
                        //     points,
                        //     layer_seams,
                        //     seam_type,
                        //     corner_pref,
                        //     z_seam_alignment_weight,
                        //     target_pos,
                        //     force_start_index,
                        //     wall_line_width_0,
                        //     seam_overhang_exclusion_areas);

                        // All lengths are in micrometers, all angles in radians
                        const AngleRadians min_vertex_turn{ AngleDegrees{ 1 } }; // 1°, below this a turn is considered as straight
                        constexpr double max_radius = 2000.0; // Arcs flatter than a 2 mm radius are not corners
                        const AngleRadians max_turn_jump{ AngleDegrees{ 40.0 } }; // Turns that differ by more than 40° between two vertices are separate corners
                        constexpr double max_corner_length = 3500.0; // A 2 mm radius quarter turn is about 3.1 mm, anything longer is too flat
                        constexpr double length_penalty_reference = 2000.0; // A corner spread over 2 mm gets half the score of a perfectly sharp one

                        const std::vector<SeamCorner> corners = findSeamCorners(points, min_vertex_turn, max_radius, max_turn_jump, max_corner_length, length_penalty_reference);

                        for (const SeamCorner& corner : corners)
                        {
                            if (std::abs(corner.score) > 0.25)
                            {
                                layer_seam_corners.push_back(corner);
                            }
                        }

                        if (! corners.empty())
                        {
                            static int send_corners = 0;
                            constexpr int send_island_index = 200;
                            if (send_corners <= send_island_index)
                            {
                                CuraViz::send(extrusion_line, "extrusion");
                            }
                            for (const SeamCorner& corner : corners)
                            {
                                if (send_corners <= send_island_index && std::abs(corner.score) > 0.25)
                                {
                                    CuraViz::send(corner.position, fmt::format("corner_{}", corner.score));
                                    spdlog::info(corner.score);
                                }
                            }

                            send_corners++;

                            // const SeamCorner& selected_corner = corners[1 % corners.size()];
                            // // const SeamCorner& selected_corner = *iterator;
                            // extrusion_line.has_start_position_set_ = true;
                            // if (selected_corner.on_vertex)
                            // {
                            //     extrusion_line.setStartJunction(selected_corner.segment_index);
                            // }
                            // else
                            // {
                            //     extrusion_line.insertJunction(selected_corner.segment_index, selected_corner.position);
                            //     extrusion_line.setStartJunction(selected_corner.segment_index + 1);
                            // }
                            continue;
                        }

                        // spdlog::warn("No single sharp corner found");
                    }
                    else
                    {
                        makeBestCandidateFinderInnerWall(best_candidate_finder, points, layer_seams);
                    }

                    // const std::optional<size_t> best_seam_candidate = best_candidate_finder.findBestElement(points.size());
                    // if (best_seam_candidate.has_value() && (outer_walls || isCloseToOuterWallSeam(points[*best_seam_candidate], layer_seams, max_inner_wall_seam_distance)))
                    // {
                    //     if (outer_walls)
                    //     {
                    //         new_layer_seams.push_back(points[*best_seam_candidate]);
                    //     }
                    //
                    //     // Re-order the toolpath so that it starts with the most appropriate seam position
                    //     extrusion_line.has_start_position_set_ = true;
                    //     extrusion_line.setStartJunction(*best_seam_candidate);
                    // }
                }
            }
        }
    }

    return new_layer_seams;
}
#endif

std::tuple<double, double> makeConcaveConvexCornersWeights(const EZSeamCornerPrefType corner_preference)
{
    double concave_angle_weight{ 1.0 };
    double convex_angle_weight{ 1.0 };

    switch (corner_preference)
    {
    case EZSeamCornerPrefType::ZSeamCornerPrefInner:
        // Give advantage to concave corners. More advantage for sharper corners.
        concave_angle_weight = 1.0;
        convex_angle_weight = 0.0;
        break;
    case EZSeamCornerPrefType::ZSeamCornerPrefOuter:
        // Give advantage to convex corners. More advantage for sharper corners.
        concave_angle_weight = 0.0;
        convex_angle_weight = 1.0;
        break;
    case EZSeamCornerPrefType::ZSeamCornerPrefAny:
        // Still give sharper corners more advantage.
        concave_angle_weight = 1.0;
        convex_angle_weight = 1.0;
        break;
    case EZSeamCornerPrefType::ZSeamCornerPrefWeighted:
        // Give sharper corners some advantage, but concave sharper corners even more.
        concave_angle_weight = 1.0;
        convex_angle_weight = 0.5;
        break;
    case EZSeamCornerPrefType::Plugin:
        break;
    }

    return { concave_angle_weight, convex_angle_weight };
}

void calculateWallsSeamPositions(SliceDataStorage& storage)
{
    const Point2LL models_center = storage.getModelBoundingBox().flatten().getMiddle();

    for (SliceMeshStorage::Ptr& mesh : storage.meshes)
    {
        const Settings& settings = mesh->settings;
        const auto seam_type = settings.get<EZSeamType>("z_seam_type");

        if (seam_type == EZSeamType::Shortest)
        {
            // We can't calculate shortest beforehand, leave it undone for later
            continue;
        }

        const auto corner_pref = settings.get<EZSeamCornerPrefType>("z_seam_corner");
        const auto [concave_angle_weight, convex_angle_weight] = makeConcaveConvexCornersWeights(corner_pref);
        const auto z_seam_alignment_weight = settings.get<double>("z_seam_alignment_weight");
        const Point2LL target_pos = mesh->getZSeamHint();
        const auto wall_line_width_0 = settings.get<coord_t>("wall_line_width_0");
        const bool z_seam_on_vertex = settings.get<bool>("z_seam_on_vertex");
        const AngleDegrees seam_overhang_angle{ settings.get<AngleDegrees>("seam_overhang_angle") };
        const coord_t layer_height{ settings.get<coord_t>("layer_height") };
        const auto wall_line_count = settings.get<size_t>("wall_line_count");
        const auto wall_line_width_x = settings.get<coord_t>("wall_line_width_x");
        const size_t inner_walls_count = wall_line_count > 0 ? wall_line_count - 1 : 0;
        const coord_t max_inner_wall_seam_distance = (wall_line_width_0 / 2) + (static_cast<coord_t>(inner_walls_count) * wall_line_width_x);
        const size_t wall_0_extruder_nr{ settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr_ };
        const Shape machine_borders{ seam_overhang_angle < 90.0 ? storage.getMachineBorder(static_cast<int>(wall_0_extruder_nr)) : Shape{} };


        std::vector<Point2LL> layer_seams;

        // for (SliceLayer& layer : mesh->layers)
        {
            constexpr bool outer_walls = true;
            calculateWallsSeamPosition(
                mesh,
                layer_seams,
                seam_type,
                concave_angle_weight,
                convex_angle_weight,
                z_seam_alignment_weight,
                target_pos,
                wall_line_width_0,
                z_seam_on_vertex,
                seam_overhang_angle,
                layer_height,
                machine_borders,
                models_center,
                max_inner_wall_seam_distance,
                outer_walls);

            // constexpr bool not_outer_walls = false;
            // calculateWallsSeamPosition(
            //     layer,
            //     layer_seams,
            //     seam_type,
            //     corner_pref,
            //     z_seam_alignment_weight,
            //     target_pos,
            //     wall_line_width_0,
            //     z_seam_on_vertex,
            //     seam_overhang_angle,
            //     layer_height,
            //     machine_borders,
            //     models_center,
            //     max_inner_wall_seam_distance,
            //     not_outer_walls);
        }
    }
}

size_t calculateShortestSeamPosition(const PointsSet& candidates, const Point2LL& target_pos, const Shape& seam_exclusion_areas)
{
    BestElementFinder best_candidate_finder;
    BestElementFinder::CriteriaPass pass;

    auto shortest_distance_criterion = std::make_shared<DistanceScoringCriterion>(candidates, target_pos);
    pass.criteria.push_back(BestElementFinder::WeighedCriterion(shortest_distance_criterion));

    addExclusionAreaCriterion(pass, candidates, seam_exclusion_areas);

    best_candidate_finder.appendCriteriaPass(pass);

    return best_candidate_finder.findBestElement(candidates.size()).value_or(0);
}

} // namespace curaengine::seam