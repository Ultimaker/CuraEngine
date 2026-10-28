// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "PathOrderMonotonic.h"

#include <filesystem>
#include <numbers>
#include <polyclipping/clipper.hpp>
#include <string>

#include <scripta/logger.h>

#include <gtest/gtest.h>

#include "ReadTestPolygons.h"
#include "geometry/OpenPolyline.h"
#include "geometry/Polygon.h"
#include "infill/Infill.h"
#include "Slicer.h"
#include "utils/Coord_t.h"
#include "utils/math.h"

// To diagnose failing tests with visual images, uncomment the following line:
// #define TEST_PATHS_SVG_OUTPUT
#ifdef TEST_PATHS_SVG_OUTPUT
#include <cstdlib>

#include "utils/SVG.h"
#endif // TEST_PATHS_SVG_OUTPUT

namespace curaengine
{
/* Fixture to allow parameterized tests.
 */
class PathOrderMonotonicTest : public testing::TestWithParam<std::tuple<std::string, ::curaengine::types::AngleRadians>>
{
};

inline ::curaengine::geometry::Point2LL startVertex(const PathOrdering<const ::curaengine::geometry::OpenPolyline*>& path)
{
    return (*path.vertices_)[path.start_vertex_];
}

inline ::curaengine::geometry::Point2LL endVertex(const PathOrdering<const ::curaengine::geometry::OpenPolyline*>& path)
{
    return (*path.vertices_)[path.vertices_->size() - (1 + path.start_vertex_)];
}

::curaengine::utils::coord_t projectPathAlongAxis(const PathOrdering<const ::curaengine::geometry::OpenPolyline*>& path, const ::curaengine::geometry::Point2LL& vector)
{
    return ::curaengine::geometry::dot(startVertex(path), vector);
}

::curaengine::utils::coord_t projectEndAlongAxis(const PathOrdering<const ::curaengine::geometry::OpenPolyline*>& path, const ::curaengine::geometry::Point2LL& vector)
{
    return ::curaengine::geometry::dot(endVertex(path), vector);
}

bool rangeOverlaps(const std::pair<::curaengine::utils::coord_t, ::curaengine::utils::coord_t>& range_b, const std::pair<::curaengine::utils::coord_t, ::curaengine::utils::coord_t>& range_a)
{
    const ::curaengine::utils::coord_t len_b = std::abs(range_b.first - range_b.second);
    const ::curaengine::utils::coord_t len_a = std::abs(range_a.first - range_a.second);
    const ::curaengine::utils::coord_t len_total
        = std::max({ range_b.first, range_b.second, range_a.first, range_a.second }) - std::min({ range_b.first, range_b.second, range_a.first, range_a.second });
    return len_total < (len_b + len_a);
}

constexpr ::curaengine::types::EFillMethod pattern = ::curaengine::types::EFillMethod::LINES;
constexpr bool zig_zagify = false;
constexpr bool connect_polygons = false;
constexpr ::curaengine::utils::coord_t line_distance = 350;
constexpr ::curaengine::utils::coord_t outline_offset = 0;
constexpr ::curaengine::utils::coord_t infill_line_width = 350;
constexpr ::curaengine::utils::coord_t infill_overlap = 0;
constexpr size_t infill_multiplier = 1;
constexpr ::curaengine::utils::coord_t z = 2;
constexpr ::curaengine::utils::coord_t shift = 0;
constexpr ::curaengine::utils::coord_t max_resolution = 10;
constexpr ::curaengine::utils::coord_t max_deviation = 5;

bool getInfillLines(const std::string& filename, const ::curaengine::types::AngleRadians& angle, ::curaengine::geometry::OpenLinesSet& output)
{
    std::vector<::curaengine::geometry::Shape> shapes;
    if (! readTestPolygons(filename, shapes))
    {
        return false;
    }

    for (const auto& shape : shapes)
    {
        ::curaengine::infill::Infill infill_comp(
            pattern,
            zig_zagify,
            connect_polygons,
            shape,
            infill_line_width,
            line_distance,
            infill_overlap,
            infill_multiplier,
            ::curaengine::types::AngleDegrees(angle),
            z,
            shift,
            max_resolution,
            max_deviation);
        Settings infill_settings;
        std::vector<::curaengine::utils::VariableWidthLines> result_paths;
        ::curaengine::geometry::Shape dummy_polys;
        infill_comp.generate(result_paths, dummy_polys, output, infill_settings, 1, ::curaengine::utils::SectionType::INFILL, nullptr, nullptr);
    }
    return true;
}

#ifdef TEST_PATHS_SVG_OUTPUT
void writeDebugSVG(
    const std::string& original_filename,
    const ::curaengine::types::AngleRadians& angle,
    const Point& monotonic_vec,
    const std::vector<std::vector<PathOrdering<ConstPolygonPointer>>>& sections)
{
    constexpr int buff_size = 1024;
    char buff[buff_size];
    const size_t xx = original_filename.find_first_of('_');
    std::string basename = original_filename.substr(xx, original_filename.find_last_of('.') - xx);
    std::snprintf(buff, buff_size, "/tmp/%s_%d.svg", basename.c_str(), (int)::curaengine::types::AngleDegrees(angle));
    const std::string filename(buff);

    ::curaengine::utils::AABB aabb;
    for (const auto& section : sections)
    {
        for (const auto& path : section)
        {
            aabb.include(startVertex(path.vertices));
            aabb.include(endVertex(path.vertices));
        }
    }
    aabb.include(Point{ 0, 0 });
    aabb.include(monotonic_vec);

    ::curaengine::utils::SVG svgFile(filename.c_str(), aabb);

    int color_id = -1;
    for (const auto& section : sections)
    {
        ++color_id;
        ::curaengine::utils::SVG::Color section_color{ (::curaengine::utils::SVG::Color)(((int)::curaengine::utils::SVG::Color::GRAY) + (color_id % 7)) };
        for (const auto& path : section)
        {
            svgFile.writePolyline(path.vertices, section_color);
        }
    }
    svgFile.writeArrow(Point{ 0, 0 }, monotonic_vec, ::curaengine::utils::SVG::Color::BLACK);
    // Note: SVG writes 'itself' when the object is destroyed.
}
#endif // TEST_PATHS_SVG_OUTPUT
// NOLINTBEGIN(*-magic-numbers)
TEST_P(PathOrderMonotonicTest, SectionsTest)
{
    auto layers = std::vector<SlicerLayer>(200, SlicerLayer{});
    scripta::setAll(layers);
    const auto params = GetParam();
    const double angle_radians{ std::get<1>(params) };
    const auto& filename = std::get<0>(params);
    ::curaengine::geometry::OpenLinesSet polylines;
    ASSERT_TRUE(getInfillLines(filename, angle_radians, polylines)) << "Input test-file could not be read, check setup.";

    const ::curaengine::geometry::Point2LL& pt_r = polylines.begin()->at(0);
    const ::curaengine::geometry::Point2LL& pt_s = polylines.begin()->at(1);
    const double angle_from_first_line = std::atan2(pt_s.Y - pt_r.Y, pt_s.X - pt_r.X) + 0.5 * std::numbers::pi;
    const ::curaengine::geometry::Point2LL monotonic_axis(static_cast<::curaengine::utils::coord_t>(std::cos(angle_from_first_line)) * 1000, static_cast<::curaengine::utils::coord_t>(std::sin(angle_from_first_line)) * 1000);
    const ::curaengine::geometry::Point2LL perpendicular_axis{ ::curaengine::geometry::turn90CCW(monotonic_axis) };

    constexpr ::curaengine::utils::coord_t max_adjacent_distance = line_distance + 1;
    PathOrderMonotonic<const ::curaengine::geometry::OpenPolyline*> object_under_test(angle_from_first_line, max_adjacent_distance, monotonic_axis * -1000);
    for (const auto& polyline : polylines)
    {
        object_under_test.addPolyline(&polyline);
    }
    object_under_test.optimize();

    // Collect sections:
    std::vector<std::vector<PathOrdering<const ::curaengine::geometry::OpenPolyline*>>> sections;
    sections.emplace_back();
    ::curaengine::utils::coord_t last_path_mono_projection = projectPathAlongAxis(object_under_test.paths_.front(), monotonic_axis);
    for (const auto& path : object_under_test.paths_)
    {
        const ::curaengine::utils::coord_t path_mono_projection{ projectPathAlongAxis(path, monotonic_axis) };
        if (path_mono_projection < last_path_mono_projection && ! sections.back().empty())
        {
            sections.emplace_back();
        }
        sections.back().push_back(path);
        last_path_mono_projection = path_mono_projection;
    }

#ifdef TEST_PATHS_SVG_OUTPUT
    writeDebugSVG(filename, angle_radians, monotonic_axis, sections);
#endif // TEST_PATHS_SVG_OUTPUT

    size_t section_a_id = 0;
    for (const auto& section_a : sections)
    {
        ++section_a_id;

        size_t section_b_id = 0;
        for (const auto& section_b : sections)
        {
            ++section_b_id;
            if (section_a_id >= section_b_id)
            {
                continue; // <-- So section B will always be 'later' than section A.
            }

            auto it_a = section_a.begin();
            for (auto it_b = section_b.begin(); it_b != section_b.end(); ++it_b)
            {
                const ::curaengine::utils::coord_t mono_b = projectPathAlongAxis(*it_b, monotonic_axis);
                for (; it_a != section_a.end() && projectPathAlongAxis(*it_a, monotonic_axis) < mono_b; ++it_a)
                {
                }
                const std::pair<::curaengine::utils::coord_t, ::curaengine::utils::coord_t> perp_b_range{ projectPathAlongAxis(*it_b, perpendicular_axis), projectEndAlongAxis(*it_b, perpendicular_axis) };
                if (it_a != section_a.end())
                {
                    // A and B intersect in monotonic direction, check if they overlap in the perpendicular direction:
                    const std::pair<::curaengine::utils::coord_t, ::curaengine::utils::coord_t> perp_a_range{ projectPathAlongAxis(*it_a, perpendicular_axis), projectEndAlongAxis(*it_a, perpendicular_axis) };
                    const ::curaengine::utils::coord_t mono_a = projectPathAlongAxis(*it_a, monotonic_axis);
                    const ::curaengine::utils::coord_t mono_b = projectPathAlongAxis(*it_b, monotonic_axis);
                    if (mono_a < mono_b)
                    {
                        EXPECT_FALSE(rangeOverlaps(perp_b_range, perp_a_range))
                            << "Perpendicular range overlaps for neighboring lines in different sections (next line of A / line in B).";
                    }
                }
            }
        }
    }
}

const std::vector<std::string> polygon_filenames = { std::filesystem::path(__FILE__).parent_path().append("resources/polygon_concave.txt").string(),
                                                     std::filesystem::path(__FILE__).parent_path().append("resources/polygon_concave_hole.txt").string(),
                                                     std::filesystem::path(__FILE__).parent_path().append("resources/polygon_square.txt").string(),
                                                     std::filesystem::path(__FILE__).parent_path().append("resources/polygon_square_hole.txt").string(),
                                                     std::filesystem::path(__FILE__).parent_path().append("resources/polygon_triangle.txt").string(),
                                                     std::filesystem::path(__FILE__).parent_path().append("resources/polygon_two_squares.txt").string(),
                                                     std::filesystem::path(__FILE__).parent_path().append("resources/polygon_slant_gap.txt").string(),
                                                     std::filesystem::path(__FILE__).parent_path().append("resources/polygon_sawtooth.txt").string(),
                                                     std::filesystem::path(__FILE__).parent_path().append("resources/polygon_letter_y.txt").string() };
const std::vector<::curaengine::types::AngleRadians> angle_radians = { 0,
                                                  0.1,
                                                  0.25 * std::numbers::pi,
                                                  1.0,
                                                  0.5 * std::numbers::pi,
                                                  0.75 * std::numbers::pi,
                                                  std::numbers::pi,
                                                  1.25 * std::numbers::pi,
                                                  4.0,
                                                  1.5 * std::numbers::pi,
                                                  1.75 * std::numbers::pi,
                                                  5.0,
                                                  (2.0 * std::numbers::pi) - 0.1 };

INSTANTIATE_TEST_SUITE_P(PathOrderMonotonicTestInstantiation, PathOrderMonotonicTest, testing::Combine(testing::ValuesIn(polygon_filenames), testing::ValuesIn(angle_radians)));
// NOLINTEND(*-magic-numbers)

} // namespace curaengine
