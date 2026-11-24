// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "seam/seam_corners.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numbers>
#include <optional>
#include <random>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>
#include <range/v3/view/enumerate.hpp>

#include "types/AngleDegrees.h"
#include "types/geometry/Point2D.h"
#include "types/geometry/PointsSet.h"
#include "types/geometry/conversions/Point2D_Point2LL.h"

namespace curaengine::seam
{

using namespace types;
using namespace types::geometry;
using namespace types::geometry::conversions;

namespace
{

// The SVG files are written by the SVG class of the engine, with a bounding box that covers the whole 400mm bed, and a scale of 0.001, which means that the SVG units are
// millimeters and that the Y axis is flipped around the bed size
constexpr double svg_bed_size_micrometers = 400000.0;
constexpr double svg_unit_micrometers = 1000.0;

// The corners in the SVG files are the expected ones, they have to match the calculated ones within this radius (in micrometers)
constexpr double matching_radius = 200.0;

// A calculated corner has to be located on a segment of the polygon, with a tolerance to compensate the rounding to integer coordinates (in micrometers)
constexpr double on_segment_tolerance = 5.0;

// When a polygon is transformed in a way that should not change its corners, the positions of the corners have to be the same, with a tolerance for the rounding (in micrometers)
constexpr double invariance_tolerance = 3.0;

// The line width that is used when it is not specified by the name of the SVG file, which is "<name>_lw<width in micrometers>.svg"
constexpr double default_wall_line_width = 400.0;

// The thresholds are the same as the ones that are used in makeSeamCandidatesAcrossLayers(), which have to be kept in sync
const AngleRadians min_vertex_turn{ AngleDegrees{ 1 } };
constexpr double max_radius = 2000.0;
const AngleRadians max_turn_jump{ AngleDegrees{ 50.0 } };
const AngleRadians max_sharp_turn{ AngleDegrees{ 50.0 } };
constexpr double max_sharp_link_distance_factor = 1.0; // Multiplied by the line width
constexpr double max_corner_length = 3500.0;
constexpr double length_penalty_reference = 2000.0;

// The corners that have a lower absolute score (once weighted by the corner type) are not kept as seam candidates. The weights are the ones of the "weighted" corner preference
constexpr double concave_angle_weight = 1.0;
constexpr double convex_angle_weight = 0.5;
constexpr double min_corner_score = 0.1;

// ################################################## Corners calculation ##################################################

/*!
 * Calculates the corners of the polygon, without any filtering
 */
std::vector<SeamCorner> findRawCorners(const PointsSet& polygon, const double wall_line_width)
{
    return findSeamCorners(
        polygon,
        min_vertex_turn,
        max_radius,
        max_turn_jump,
        max_sharp_turn,
        wall_line_width * max_sharp_link_distance_factor,
        max_corner_length,
        length_penalty_reference);
}

/*!
 * Calculates the corners of the polygon, the same way as they are kept as seam candidates in the engine
 */
std::vector<SeamCorner> calculateCorners(const PointsSet& polygon, const double wall_line_width)
{
    std::vector<SeamCorner> kept_corners;
    for (SeamCorner& corner : findRawCorners(polygon, wall_line_width))
    {
        const bool convex = corner.score < 0;
        corner.score *= convex ? convex_angle_weight : concave_angle_weight;

        if (std::abs(corner.score) > min_corner_score)
        {
            kept_corners.push_back(corner);
        }
    }

    return kept_corners;
}

std::vector<Point2D> toPositions(const std::vector<SeamCorner>& corners)
{
    std::vector<Point2D> positions;
    for (const SeamCorner& corner : corners)
    {
        positions.push_back(toPoint2D(corner.position));
    }
    return positions;
}

double toDegrees(const double radians)
{
    return radians * 180.0 / std::numbers::pi;
}

std::string formatPosition(const Point2D& position)
{
    return fmt::format("({:.3f}, {:.3f}) mm", position.x() / svg_unit_micrometers, position.y() / svg_unit_micrometers);
}

// ################################################## Geometry helpers ##################################################

double distanceToSegment(const Point2D& point, const Point2LL& segment_start, const Point2LL& segment_end)
{
    const Point2D start = toPoint2D(segment_start);
    const Point2D vector = toPoint2D(segment_end) - start;
    const double length_squared = vector.vSize2();
    const double t = length_squared > 0.0 ? std::clamp(Point2D::dot(point - start, vector) / length_squared, 0.0, 1.0) : 0.0;
    return (point - (start + vector * t)).vSize();
}

/*!
 * Gets the distance between a point and the closest segment of a closed polygon
 */
double distanceToPolygon(const Point2D& point, const PointsSet& polygon)
{
    double distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < polygon.size(); ++i)
    {
        distance = std::min(distance, distanceToSegment(point, polygon.at(i), polygon.at((i + 1) % polygon.size())));
    }
    return distance;
}

struct CornerPairing
{
    std::vector<bool> expected_matched;
    std::vector<bool> calculated_matched;
};

/*!
 * Pairs the expected corners with the calculated ones. Each corner has to be paired with exactly one corner of the other list, and may be paired only if it is within the given
 * radius. A corner may have several others within the radius when they are close to each other, so the pairs are made from the closest to the farthest, and every corner can
 * be used only once
 */
CornerPairing pairCorners(const std::vector<Point2D>& expected_corners, const std::vector<Point2D>& calculated_corners, const double radius)
{
    struct Candidate
    {
        size_t expected_index;
        size_t calculated_index;
        double distance;
    };

    std::vector<Candidate> candidates;
    for (const auto& [expected_index, expected] : expected_corners | ranges::views::enumerate)
    {
        for (const auto& [calculated_index, calculated] : calculated_corners | ranges::views::enumerate)
        {
            const double distance = (calculated - expected).vSize();
            if (distance <= radius)
            {
                candidates.push_back(Candidate{ .expected_index = expected_index, .calculated_index = calculated_index, .distance = distance });
            }
        }
    }
    std::stable_sort(
        candidates.begin(),
        candidates.end(),
        [](const Candidate& candidate1, const Candidate& candidate2)
        {
            return candidate1.distance < candidate2.distance;
        });

    CornerPairing pairing{ .expected_matched = std::vector<bool>(expected_corners.size(), false), .calculated_matched = std::vector<bool>(calculated_corners.size(), false) };
    for (const Candidate& candidate : candidates)
    {
        if (! pairing.expected_matched[candidate.expected_index] && ! pairing.calculated_matched[candidate.calculated_index])
        {
            pairing.expected_matched[candidate.expected_index] = true;
            pairing.calculated_matched[candidate.calculated_index] = true;
        }
    }

    return pairing;
}

/*!
 * Checks that the calculated corners are exactly the expected ones, within the given radius
 */
void expectSamePositions(const std::vector<Point2D>& expected_corners, const std::vector<Point2D>& calculated_corners, const double radius)
{
    const CornerPairing pairing = pairCorners(expected_corners, calculated_corners, radius);

    for (const auto& [expected_index, expected] : expected_corners | ranges::views::enumerate)
    {
        EXPECT_TRUE(pairing.expected_matched[expected_index]) << "The expected corner at " << formatPosition(expected) << " has no calculated corner within " << radius << " micrometers";
    }

    for (const auto& [calculated_index, calculated] : calculated_corners | ranges::views::enumerate)
    {
        EXPECT_TRUE(pairing.calculated_matched[calculated_index]) << "The calculated corner at " << formatPosition(calculated) << " has no expected corner within " << radius
                                                                  << " micrometers";
    }
}

// ################################################## SVG files ##################################################

struct SeamSvg
{
    std::vector<PointsSet> polygons; // The closed polygons, in the order of the file, without any repeated closing point
    std::vector<Point2D> expected_corners; // The positions of the expected corners, in micrometers
    std::string error;
};

Point2D svgToMicrometers(const double x, const double y)
{
    return Point2D(x * svg_unit_micrometers, svg_bed_size_micrometers - y * svg_unit_micrometers);
}

/*!
 * Reads the points of a path that is only made of absolute move and line commands, with a closing command
 */
std::optional<PointsSet> parsePath(const std::string& path_data, std::string& error)
{
    static const std::regex token_regex(R"re(([A-Za-z])|([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?))re");

    PointsSet points;
    std::vector<double> numbers;

    const auto flush_numbers = [&points, &numbers, &error]() -> bool
    {
        if (numbers.size() % 2 != 0)
        {
            error = "A path has an odd number of coordinates";
            return false;
        }

        for (size_t i = 0; i < numbers.size(); i += 2)
        {
            points.push_back(toPoint2LL(svgToMicrometers(numbers[i], numbers[i + 1])));
        }
        numbers.clear();
        return true;
    };

    for (std::sregex_iterator it(path_data.begin(), path_data.end(), token_regex), end; it != end; ++it)
    {
        if ((*it)[1].matched)
        {
            if (! flush_numbers())
            {
                return std::nullopt;
            }

            const char command = (*it)[1].str()[0];
            if (command != 'M' && command != 'L' && command != 'Z')
            {
                error = fmt::format("Unsupported path command '{}', only absolute M, L and Z are supported", command);
                return std::nullopt;
            }
        }
        else
        {
            numbers.push_back(std::stod((*it)[2].str()));
        }
    }

    if (! flush_numbers())
    {
        return std::nullopt;
    }

    // Paths are written with a closing point that repeats the first one, which findSeamCorners() doesn't want
    if (points.size() > 1 && points.front() == points.back())
    {
        points.pop_back();
    }

    return points;
}

SeamSvg loadSeamSvg(const std::filesystem::path& filepath)
{
    SeamSvg result;

    std::ifstream file(filepath);
    if (! file)
    {
        result.error = fmt::format("Cannot open {}", filepath.string());
        return result;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    const std::string content = buffer.str();

    static const std::regex path_regex(R"re(<path\b[^>]*?\bd="([^"]*)")re");
    for (std::sregex_iterator it(content.begin(), content.end(), path_regex), end; it != end; ++it)
    {
        const std::optional<PointsSet> points = parsePath((*it)[1].str(), result.error);
        if (! points.has_value())
        {
            return result;
        }
        result.polygons.push_back(points.value());
    }

    static const std::regex circle_regex(R"re(<circle\b[^>]*?/>)re");
    static const std::regex cx_regex(R"re(\bcx="([^"]*)")re");
    static const std::regex cy_regex(R"re(\bcy="([^"]*)")re");
    for (std::sregex_iterator it(content.begin(), content.end(), circle_regex), end; it != end; ++it)
    {
        const std::string circle = it->str();
        std::smatch cx_match;
        std::smatch cy_match;
        if (! std::regex_search(circle, cx_match, cx_regex) || ! std::regex_search(circle, cy_match, cy_regex))
        {
            result.error = "A circle has no center";
            return result;
        }
        result.expected_corners.push_back(svgToMicrometers(std::stod(cx_match[1].str()), std::stod(cy_match[1].str())));
    }

    return result;
}

/*!
 * Lists all the SVG files of the resources folder, which are the test cases
 */
std::vector<std::filesystem::path> findSeamSvgFiles()
{
    std::vector<std::filesystem::path> files;

    const std::filesystem::path folder = std::filesystem::path(__FILE__).parent_path() / "resources" / "seam";
    if (std::filesystem::exists(folder))
    {
        for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(folder))
        {
            if (entry.is_regular_file() && entry.path().extension() == ".svg")
            {
                files.push_back(entry.path());
            }
        }
    }

    std::sort(files.begin(), files.end());
    return files;
}

/*!
 * Gets the wall line width that has to be used with an SVG file, which may be given by a "_lw<width in micrometers>" suffix of the file name
 */
double getWallLineWidth(const std::filesystem::path& svg_file)
{
    static const std::regex line_width_regex(R"re(_lw(\d+)$)re");

    const std::string name = svg_file.stem().string();
    std::smatch match;
    if (std::regex_search(name, match, line_width_regex))
    {
        return std::stod(match[1].str());
    }

    return default_wall_line_width;
}

/*!
 * Assigns each expected corner to the polygon that is the closest to it
 * @return For each polygon, the expected corners that belong to it
 */
std::vector<std::vector<Point2D>> splitExpectedCornersPerPolygon(const SeamSvg& seam_svg)
{
    std::vector<std::vector<Point2D>> expected_corners(seam_svg.polygons.size());

    for (const Point2D& corner : seam_svg.expected_corners)
    {
        size_t closest_polygon = 0;
        double closest_distance = std::numeric_limits<double>::max();
        for (const auto& [polygon_index, polygon] : seam_svg.polygons | ranges::views::enumerate)
        {
            const double distance = distanceToPolygon(corner, polygon);
            if (distance < closest_distance)
            {
                closest_distance = distance;
                closest_polygon = polygon_index;
            }
        }
        expected_corners[closest_polygon].push_back(corner);
    }

    return expected_corners;
}

// ################################################## Synthetic polygons ##################################################

double cross(const Point2D& a, const Point2D& b)
{
    return a.x() * b.y() - a.y() * b.x();
}

Point2D directionFromHeading(const double heading_degrees)
{
    const double heading = heading_degrees * std::numbers::pi / 180.0;
    return Point2D(std::cos(heading), std::sin(heading));
}

/*!
 * Draws a polyline by moving a pen forward and turning it, like a turtle. The vertices are the end of each forward move, plus the starting point. Turning angles are cumulative
 * between two moves, and apply to the last vertex. Positive angles turn left.
 */
class Turtle
{
public:
    explicit Turtle(const double heading_degrees)
        : heading_(heading_degrees)
        , vertices_{ Point2D(0.0, 0.0) }
    {
    }

    void turn(const double angle_degrees)
    {
        heading_ += angle_degrees;
    }

    void forward(const double length)
    {
        vertices_.push_back(vertices_.back() + directionFromHeading(heading_) * length);
    }

    /*!
     * Draws a circular arc, the way a mesh would be tessellated: the vertices are on the circle, and the chords are as long as possible while keeping the given maximum deviation
     * to the circle
     */
    void arc(const double radius, const double angle_degrees, const double tolerance)
    {
        const double max_step = 2.0 * std::acos(1.0 - tolerance / radius) * 180.0 / std::numbers::pi;
        const int segments_count = std::max(1, static_cast<int>(std::ceil(std::abs(angle_degrees) / max_step)));
        const double step = angle_degrees / segments_count;
        const double chord = 2.0 * radius * std::sin(std::abs(step) * std::numbers::pi / 360.0);

        turn(step / 2.0);
        for (int i = 0; i < segments_count; ++i)
        {
            forward(chord);
            if (i < segments_count - 1)
            {
                turn(step);
            }
        }
        turn(step / 2.0);
    }

    double heading() const
    {
        return heading_;
    }

    const std::vector<Point2D>& vertices() const
    {
        return vertices_;
    }

private:
    double heading_;
    std::vector<Point2D> vertices_;
};

/*!
 * Makes the vertices of a corner of a polygon, which is drawn with a turtle and then placed so that its incoming and outgoing edges meet at the given position, as if the corner
 * were perfectly sharp. This way the edges that connect the corners stay perfectly straight.
 */
template<typename BuildCorner>
std::vector<Point2D> makeCorner(const Point2D& corner_position, const double incoming_heading, BuildCorner&& build_corner)
{
    Turtle turtle(incoming_heading);
    build_corner(turtle);

    const Point2D incoming_direction = directionFromHeading(incoming_heading);
    const Point2D outgoing_direction = directionFromHeading(turtle.heading());
    const Point2D start = turtle.vertices().front();
    const Point2D end = turtle.vertices().back();

    const double t = cross(end - start, outgoing_direction) / cross(incoming_direction, outgoing_direction);
    const Point2D virtual_corner = start + incoming_direction * t;

    std::vector<Point2D> vertices;
    for (const Point2D& vertex : turtle.vertices())
    {
        vertices.push_back(vertex + (corner_position - virtual_corner));
    }
    return vertices;
}

struct SyntheticPolygon
{
    PointsSet polygon;
    std::vector<Point2D> corner_centers; // The position that is the middle of the path of each corner
    std::vector<double> corner_lengths; // The length of the path of each corner
};

Point2D middleOfPath(const std::vector<Point2D>& vertices, double& length)
{
    length = 0.0;
    for (size_t i = 1; i < vertices.size(); ++i)
    {
        length += (vertices[i] - vertices[i - 1]).vSize();
    }

    double travelled = 0.0;
    for (size_t i = 1; i < vertices.size(); ++i)
    {
        const double segment_length = (vertices[i] - vertices[i - 1]).vSize();
        if (travelled + segment_length >= length / 2.0)
        {
            const double ratio = segment_length > 0.0 ? (length / 2.0 - travelled) / segment_length : 0.0;
            return vertices[i - 1] + (vertices[i] - vertices[i - 1]) * ratio;
        }
        travelled += segment_length;
    }

    return vertices.front();
}

/*!
 * Makes a rectangle whose 4 corners all have the same shape
 */
template<typename BuildCorner>
SyntheticPolygon makeRectangle(const double width, const double height, BuildCorner&& build_corner)
{
    // Counter-clockwise, starting from the bottom left corner. The heading is the one of the edge that goes to the corner
    const std::vector<std::pair<Point2D, double>> corners{ { Point2D(-width / 2.0, -height / 2.0), 270.0 },
                                                           { Point2D(width / 2.0, -height / 2.0), 0.0 },
                                                           { Point2D(width / 2.0, height / 2.0), 90.0 },
                                                           { Point2D(-width / 2.0, height / 2.0), 180.0 } };

    SyntheticPolygon result;
    for (const auto& [position, heading] : corners)
    {
        const std::vector<Point2D> vertices = makeCorner(position, heading, build_corner);
        for (const Point2D& vertex : vertices)
        {
            result.polygon.push_back(toPoint2LL(vertex));
        }

        double length = 0.0;
        result.corner_centers.push_back(middleOfPath(vertices, length));
        result.corner_lengths.push_back(length);
    }

    return result;
}

/*!
 * Checks that the given corners are exactly one per corner of the synthetic polygon, which are located at the middle of the corner, and which turn by the given angle
 * @param extra_tolerance Additional distance (in micrometers) allowed between the calculated corner and the middle of the corner, for corners that are not symmetrical and whose
 * position is thus closer to their sharpest part
 */
void expectOneCornerPerPolygonCorner(
    const SyntheticPolygon& synthetic_polygon,
    const std::vector<SeamCorner>& corners,
    const double expected_turn_degrees,
    const double extra_tolerance = 0.0)
{
    ASSERT_EQ(corners.size(), synthetic_polygon.corner_centers.size());

    for (const auto& [corner_index, center] : synthetic_polygon.corner_centers | ranges::views::enumerate)
    {
        const double tolerance = 0.1 * synthetic_polygon.corner_lengths[corner_index] + 20.0 + extra_tolerance;

        const auto closest = std::min_element(
            corners.begin(),
            corners.end(),
            [&center](const SeamCorner& corner1, const SeamCorner& corner2)
            {
                return (toPoint2D(corner1.position) - center).vSize() < (toPoint2D(corner2.position) - center).vSize();
            });

        EXPECT_LE((toPoint2D(closest->position) - center).vSize(), tolerance) << "No corner near " << formatPosition(center);
        EXPECT_NEAR(std::abs(toDegrees(closest->turn)), expected_turn_degrees, 5.0) << "Wrong turn of the corner near " << formatPosition(center);
    }
}

/*!
 * Gets the corners that are the closest to the given position, within the given radius
 */
std::vector<SeamCorner> getCornersNear(const std::vector<SeamCorner>& corners, const Point2D& position, const double radius)
{
    std::vector<SeamCorner> corners_near;
    for (const SeamCorner& corner : corners)
    {
        if ((toPoint2D(corner.position) - position).vSize() <= radius)
        {
            corners_near.push_back(corner);
        }
    }
    return corners_near;
}

// ################################################## Transformations ##################################################

template<typename Transform>
PointsSet transformed(const PointsSet& polygon, Transform&& transform)
{
    PointsSet result;
    for (const Point2LL& point : polygon)
    {
        result.push_back(transform(point));
    }
    return result;
}

} // namespace

// ################################################## Tests based on SVG files ##################################################

class SeamCornersSvgTest : public testing::TestWithParam<std::filesystem::path>
{
};

INSTANTIATE_TEST_SUITE_P(
    SvgFiles,
    SeamCornersSvgTest,
    testing::ValuesIn(findSeamSvgFiles()),
    [](const testing::TestParamInfo<std::filesystem::path>& info)
    {
        std::string name = info.param.stem().string();
        std::replace_if(
            name.begin(),
            name.end(),
            [](const char c)
            {
                return ! std::isalnum(static_cast<unsigned char>(c));
            },
            '_');
        return name;
    });

TEST_P(SeamCornersSvgTest, CalculatedCornersMatchTheExpectedOnes)
{
    const SeamSvg seam_svg = loadSeamSvg(GetParam());
    ASSERT_TRUE(seam_svg.error.empty()) << seam_svg.error;

    const double wall_line_width = getWallLineWidth(GetParam());
    const std::vector<std::vector<Point2D>> expected_corners_per_polygon = splitExpectedCornersPerPolygon(seam_svg);

    for (const auto& [polygon_index, polygon] : seam_svg.polygons | ranges::views::enumerate)
    {
        SCOPED_TRACE(fmt::format("Polygon {} ({} points)", polygon_index, polygon.size()));

        const std::vector<SeamCorner> calculated_seam_corners = calculateCorners(polygon, wall_line_width);
        const std::vector<Point2D> calculated_corners = toPositions(calculated_seam_corners);
        const std::vector<Point2D>& expected_corners = expected_corners_per_polygon[polygon_index];

        // Each expected corner has to be paired with exactly one calculated corner, and vice versa
        const CornerPairing pairing = pairCorners(expected_corners, calculated_corners, matching_radius);

        for (const auto& [expected_index, expected] : expected_corners | ranges::views::enumerate)
        {
            EXPECT_TRUE(pairing.expected_matched[expected_index]) << "The expected corner at " << formatPosition(expected) << " has no calculated corner within " << matching_radius
                                                                  << " micrometers";
        }

        for (const auto& [calculated_index, calculated] : calculated_corners | ranges::views::enumerate)
        {
            const SeamCorner& seam_corner = calculated_seam_corners[calculated_index];
            EXPECT_TRUE(pairing.calculated_matched[calculated_index])
                << "The calculated corner at " << formatPosition(calculated) << " has no expected corner within " << matching_radius << " micrometers (turn "
                << toDegrees(seam_corner.turn) << " deg, extent " << seam_corner.extent << " micrometers, score " << seam_corner.score << ")";
        }
    }
}

TEST_P(SeamCornersSvgTest, CalculatedCornersAreOnSegments)
{
    const SeamSvg seam_svg = loadSeamSvg(GetParam());
    ASSERT_TRUE(seam_svg.error.empty()) << seam_svg.error;

    const double wall_line_width = getWallLineWidth(GetParam());

    for (const auto& [polygon_index, polygon] : seam_svg.polygons | ranges::views::enumerate)
    {
        SCOPED_TRACE(fmt::format("Polygon {} ({} points)", polygon_index, polygon.size()));

        for (const SeamCorner& corner : calculateCorners(polygon, wall_line_width))
        {
            const Point2D position = toPoint2D(corner.position);
            const double distance = distanceToPolygon(position, polygon);
            EXPECT_LE(distance, on_segment_tolerance) << "The corner at " << formatPosition(position) << " is " << distance << " micrometers away from the polygon";

            EXPECT_LT(corner.segment_index, polygon.size());
            if (corner.on_vertex && corner.segment_index < polygon.size())
            {
                EXPECT_EQ(polygon.at(corner.segment_index), corner.position) << "The corner at " << formatPosition(position) << " is said to be on a vertex, but it is not";
            }
        }
    }
}

TEST_P(SeamCornersSvgTest, CornersDoNotDependOnTheStartVertex)
{
    const SeamSvg seam_svg = loadSeamSvg(GetParam());
    ASSERT_TRUE(seam_svg.error.empty()) << seam_svg.error;

    const double wall_line_width = getWallLineWidth(GetParam());

    for (const auto& [polygon_index, polygon] : seam_svg.polygons | ranges::views::enumerate)
    {
        SCOPED_TRACE(fmt::format("Polygon {} ({} points)", polygon_index, polygon.size()));

        const std::vector<Point2D> reference_corners = toPositions(findRawCorners(polygon, wall_line_width));

        for (const size_t shift : { size_t{ 1 }, polygon.size() / 3, polygon.size() / 2 })
        {
            PointsSet shifted_polygon;
            for (size_t i = 0; i < polygon.size(); ++i)
            {
                shifted_polygon.push_back(polygon.at((i + shift) % polygon.size()));
            }

            SCOPED_TRACE(fmt::format("Start vertex shifted by {}", shift));
            expectSamePositions(reference_corners, toPositions(findRawCorners(shifted_polygon, wall_line_width)), invariance_tolerance);
        }
    }
}

TEST_P(SeamCornersSvgTest, CornersDoNotDependOnTheOrientation)
{
    const SeamSvg seam_svg = loadSeamSvg(GetParam());
    ASSERT_TRUE(seam_svg.error.empty()) << seam_svg.error;

    const double wall_line_width = getWallLineWidth(GetParam());

    for (const auto& [polygon_index, polygon] : seam_svg.polygons | ranges::views::enumerate)
    {
        SCOPED_TRACE(fmt::format("Polygon {} ({} points)", polygon_index, polygon.size()));

        const std::vector<Point2D> reference_corners = toPositions(findRawCorners(polygon, wall_line_width));

        {
            // Going the other way around
            PointsSet reversed_polygon;
            for (size_t i = polygon.size(); i > 0; --i)
            {
                reversed_polygon.push_back(polygon.at(i - 1));
            }

            SCOPED_TRACE("Reversed polygon");
            expectSamePositions(reference_corners, toPositions(findRawCorners(reversed_polygon, wall_line_width)), invariance_tolerance);
        }

        {
            // Mirrored, which also changes the direction of the polygon
            const PointsSet mirrored_polygon = transformed(
                polygon,
                [](const Point2LL& point)
                {
                    return Point2LL(-point.X, point.Y);
                });
            std::vector<Point2D> mirrored_reference_corners;
            for (const Point2D& corner : reference_corners)
            {
                mirrored_reference_corners.emplace_back(-corner.x(), corner.y());
            }

            SCOPED_TRACE("Mirrored polygon");
            expectSamePositions(mirrored_reference_corners, toPositions(findRawCorners(mirrored_polygon, wall_line_width)), invariance_tolerance);
        }

        {
            // Rotated by 90 degrees, which doesn't introduce any rounding
            const PointsSet rotated_polygon = transformed(
                polygon,
                [](const Point2LL& point)
                {
                    return Point2LL(-point.Y, point.X);
                });
            std::vector<Point2D> rotated_reference_corners;
            for (const Point2D& corner : reference_corners)
            {
                rotated_reference_corners.emplace_back(-corner.y(), corner.x());
            }

            SCOPED_TRACE("Polygon rotated by 90 degrees");
            expectSamePositions(rotated_reference_corners, toPositions(findRawCorners(rotated_polygon, wall_line_width)), invariance_tolerance);
        }
    }
}

// ################################################## Tests based on synthetic polygons ##################################################

TEST(SeamCornersSyntheticTest, SharpRectangleHasFourCornersOnItsVertices)
{
    const SyntheticPolygon rectangle = makeRectangle(
        40000.0,
        30000.0,
        [](Turtle& turtle)
        {
            turtle.turn(90.0);
        });

    const std::vector<SeamCorner> corners = findRawCorners(rectangle.polygon, default_wall_line_width);
    expectOneCornerPerPolygonCorner(rectangle, corners, 90.0);
    for (const SeamCorner& corner : corners)
    {
        EXPECT_TRUE(corner.on_vertex);
        EXPECT_EQ(corner.extent, 0);
    }
}

TEST(SeamCornersSyntheticTest, RoundedCornersGiveASingleCornerAtTheirMiddle)
{
    // The radius of the roundings that are detected is a bit lower than the maximum radius, because the first and last vertices of a rounding turn by half as much as the others
    for (const double radius : { 300.0, 500.0, 800.0, 1000.0, 1500.0 })
    {
        for (const double tolerance : { 5.0, 10.0, 20.0 })
        {
            SCOPED_TRACE(fmt::format("Radius {} micrometers, tessellation tolerance {} micrometers", radius, tolerance));

            const SyntheticPolygon rectangle = makeRectangle(
                40000.0,
                30000.0,
                [radius, tolerance](Turtle& turtle)
                {
                    turtle.arc(radius, 90.0, tolerance);
                });

            expectOneCornerPerPolygonCorner(rectangle, findRawCorners(rectangle.polygon, default_wall_line_width), 90.0);
        }
    }
}

// This is the limit of the radius, which is currently not well defined: the vertices at the ends of a rounding are not linked to the others, because their turn is half of the
// others. Remove the "DISABLED_" prefix to see it
TEST(SeamCornersSyntheticTest, DISABLED_RoundedCornersAtTheMaximumRadiusGiveASingleCorner)
{
    for (const double tolerance : { 5.0, 10.0, 20.0 })
    {
        SCOPED_TRACE(fmt::format("Tessellation tolerance {} micrometers", tolerance));

        const SyntheticPolygon rectangle = makeRectangle(
            40000.0,
            30000.0,
            [tolerance](Turtle& turtle)
            {
                turtle.arc(max_radius, 90.0, tolerance);
            });

        expectOneCornerPerPolygonCorner(rectangle, findRawCorners(rectangle.polygon, default_wall_line_width), 90.0);
    }
}

TEST(SeamCornersSyntheticTest, TooLargeRoundedCornersAreNotKept)
{
    for (const double radius : { 3000.0, 5000.0 })
    {
        SCOPED_TRACE(fmt::format("Radius {} micrometers", radius));

        const SyntheticPolygon rectangle = makeRectangle(
            40000.0,
            30000.0,
            [radius](Turtle& turtle)
            {
                turtle.arc(radius, 90.0, 10.0);
            });

        EXPECT_TRUE(calculateCorners(rectangle.polygon, default_wall_line_width).empty());
    }
}

TEST(SeamCornersSyntheticTest, ChamfersCloserThanTheLineWidthAreSingleCorners)
{
    // A corner that is cut by a short segment is made of two vertices that turn in the same direction. They are the same corner if they are closer than a line width, otherwise
    // they are two corners
    for (const double wall_line_width : { 400.0, 600.0 })
    {
        for (const double first_turn : { 55.0, 60.0, 65.0 })
        {
            for (const double gap_ratio : { 0.4, 0.6, 0.8, 1.25, 1.5, 2.0 })
            {
                SCOPED_TRACE(fmt::format("Line width {}, first turn {}, distance between the vertices {} times the line width", wall_line_width, first_turn, gap_ratio));

                const double gap = gap_ratio * wall_line_width;
                const SyntheticPolygon rectangle = makeRectangle(
                    40000.0,
                    30000.0,
                    [first_turn, gap](Turtle& turtle)
                    {
                        turtle.turn(first_turn);
                        turtle.forward(gap);
                        turtle.turn(90.0 - first_turn);
                    });

                const std::vector<SeamCorner> corners = findRawCorners(rectangle.polygon, wall_line_width);
                if (gap_ratio < 1.0 / max_sharp_link_distance_factor)
                {
                    expectOneCornerPerPolygonCorner(rectangle, corners, 90.0, 0.4 * gap);
                }
                else
                {
                    EXPECT_EQ(corners.size(), 8);
                }
            }
        }
    }
}

TEST(SeamCornersSyntheticTest, SharpCornerBetweenTwoRoundedOnesIsAlone)
{
    for (const double arc_turn : { 10.0, 15.0, 20.0 })
    {
        SCOPED_TRACE(fmt::format("Rounded parts turning by {} degrees", arc_turn));

        const SyntheticPolygon rectangle = makeRectangle(
            40000.0,
            30000.0,
            [arc_turn](Turtle& turtle)
            {
                turtle.arc(800.0, arc_turn, 10.0);
                turtle.turn(90.0 - 2.0 * arc_turn);
                turtle.arc(800.0, arc_turn, 10.0);
            });

        const std::vector<SeamCorner> corners = findRawCorners(rectangle.polygon, default_wall_line_width);
        EXPECT_EQ(corners.size(), 12);

        for (const Point2D& center : rectangle.corner_centers)
        {
            const std::vector<SeamCorner> corners_near = getCornersNear(corners, center, 2000.0);
            ASSERT_EQ(corners_near.size(), 3) << "Wrong number of corners near " << formatPosition(center);

            const auto sharpest = std::max_element(
                corners_near.begin(),
                corners_near.end(),
                [](const SeamCorner& corner1, const SeamCorner& corner2)
                {
                    return std::abs(corner1.turn) < std::abs(corner2.turn);
                });
            EXPECT_GE(std::abs(toDegrees(sharpest->turn)), 90.0 - 2.0 * arc_turn);
            EXPECT_EQ(sharpest->extent, 0);
        }
    }
}

TEST(SeamCornersSyntheticTest, EveryTipOfAZigzagIsACorner)
{
    // A gear-like polygon, where the vertices alternate between an outer and an inner radius
    constexpr int teeth_count = 30;
    PointsSet gear;
    for (int i = 0; i < 2 * teeth_count; ++i)
    {
        const double radius = i % 2 == 0 ? 15000.0 : 14000.0;
        const double angle = static_cast<double>(i) * std::numbers::pi / teeth_count;
        gear.push_back(toPoint2LL(Point2D(radius * std::cos(angle), radius * std::sin(angle))));
    }

    const std::vector<SeamCorner> corners = findRawCorners(gear, default_wall_line_width);
    EXPECT_EQ(corners.size(), static_cast<size_t>(2 * teeth_count));
    for (const SeamCorner& corner : corners)
    {
        EXPECT_GT(std::abs(toDegrees(corner.turn)), 40.0);
    }
}

TEST(SeamCornersSyntheticTest, NoiseOnStraightEdgesIsNotKeptAsCorners)
{
    const SyntheticPolygon rectangle = makeRectangle(
        40000.0,
        30000.0,
        [](Turtle& turtle)
        {
            turtle.turn(90.0);
        });

    // Subdivide the edges with a small distance between the vertices, and move them randomly by a few micrometers
    std::mt19937 random_generator(42);
    PointsSet noisy_polygon;
    for (size_t i = 0; i < rectangle.polygon.size(); ++i)
    {
        const Point2LL start = rectangle.polygon.at(i);
        const Point2LL end = rectangle.polygon.at((i + 1) % rectangle.polygon.size());

        noisy_polygon.push_back(start);

        const int segments_count = static_cast<int>((end - start).X == 0 && (end - start).Y == 0 ? 1 : std::llround(vSize(end - start) / 150.0));
        for (int k = 1; k < segments_count; ++k)
        {
            const double ratio = static_cast<double>(k) / segments_count;
            const Point2LL position = start + toPoint2LL(toPoint2D(end - start) * ratio);
            noisy_polygon.push_back(position + Point2LL(static_cast<coord_t>(random_generator() % 7) - 3, static_cast<coord_t>(random_generator() % 7) - 3));
        }
    }

    const std::vector<SeamCorner> corners = calculateCorners(noisy_polygon, default_wall_line_width);
    ASSERT_EQ(corners.size(), 4);
    for (const Point2D& center : rectangle.corner_centers)
    {
        EXPECT_EQ(getCornersNear(corners, center, 400.0).size(), 1) << "Wrong number of corners near " << formatPosition(center);
    }
}

} // namespace curaengine::seam
