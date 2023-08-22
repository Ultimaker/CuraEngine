// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "infill.h"
#include "ReadTestPolygons.h"
#include "slicer.h"
#include "utils/Coord_t.h"
#include <gtest/gtest.h>
#include <filesystem>
#include <utility>
#include <fmt/format.h>

#include <scripta/logger.h>

// #define TEST_INFILL_SVG_OUTPUT
#ifdef TEST_INFILL_SVG_OUTPUT
#include "utils/SVG.h"
#include <cstdlib>
#endif // TEST_INFILL_SVG_OUTPUT

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{
coord_t getPatternMultiplier(const EFillMethod& pattern)
{
    switch (pattern)
    {
    case EFillMethod::GRID: // fallthrough
    case EFillMethod::TETRAHEDRAL: // fallthrough
    case EFillMethod::QUARTER_CUBIC:
        return 2;
    case EFillMethod::TRIANGLES: // fallthrough
    case EFillMethod::TRIHEXAGON: // fallthrough
    case EFillMethod::CUBIC: // fallthrough
    case EFillMethod::CUBICSUBDIV:
        return 3;
    default:
        return 1;
    }
}

// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
struct InfillParameters
{
public:
    // Actual infill parameters:
    EFillMethod pattern;
    bool zig_zagify;
    bool connect_polygons;
    coord_t line_distance;

    std::string name;

    InfillParameters(const EFillMethod& pattern, const bool& zig_zagify, const bool& connect_polygons, const coord_t& line_distance)
        : pattern(pattern)
        , zig_zagify(zig_zagify)
        , connect_polygons(connect_polygons)
        , line_distance(line_distance)
    {
        name = fmt::format("InfillParameters_{:d}_{:d}_{:d}_{:d}", static_cast<int>(pattern), zig_zagify, connect_polygons, line_distance);
    }
};

class InfillTestParameters
{
public:
    bool valid; // <-- if the file isn't read (or anything else goes wrong with the setup) we can communicate it to the tests
    std::string fail_reason;

    // Parameters used to generate the infill:
    InfillParameters params;
    Polygons outline_polygons;

    // Resulting infill:
    Polygons result_lines;
    Polygons result_polygons;

    std::string name;

    InfillTestParameters() : valid(false), fail_reason("Read of file with test polygons failed (see generateInfillTests), can't continue tests."), params(InfillParameters(EFillMethod::NONE, false, false, 0)), name("UNNAMED")
    {
    }

    InfillTestParameters(const InfillParameters& params, const size_t& test_polygon_id, Polygons outline_polygons, Polygons result_lines, Polygons result_polygons)
        : valid(true)
        , fail_reason("__")
        , params(params)
        , outline_polygons(std::move(outline_polygons))
        , result_lines(std::move(result_lines))
        , result_polygons(std::move(result_polygons))
    {
        name = fmt::format("InfillTestParameters_P{:d}_Z{:d}_C{:d}_L{:d}__{:d}", static_cast<int>(params.pattern), params.zig_zagify, params.connect_polygons, params.line_distance, test_polygon_id);
    }

    friend std::ostream& operator<<(std::ostream& os, const InfillTestParameters& params)
    {
        return os << params.name << "(" << (params.valid ? std::string("input OK") : params.fail_reason) << ")";
    }
};
// NOLINTEND(misc-non-private-member-variables-in-classes)


constexpr coord_t INFILL_LINE_WIDTH = 350;
constexpr coord_t INFILL_OVERLAP = 0;
constexpr size_t INFILL_MULTIPLIER = 1;
const AngleDegrees FILL_ANGLE = 0.;
constexpr coord_t Z = 100; // Future improvement: Also take an uneven layer, so we get the alternate.
constexpr coord_t SHIFT = 0;
constexpr coord_t MAX_RESOLUTION = 10;
constexpr coord_t MAX_DEVIATION = 5;
const std::vector<std::string> POLYGON_FILENAMES = {
    std::filesystem::path(__FILE__).parent_path().append("resources/polygon_concave.txt").string(),  std::filesystem::path(__FILE__).parent_path().append("resources/polygon_concave_hole.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("resources/polygon_square.txt").string(),   std::filesystem::path(__FILE__).parent_path().append("resources/polygon_square_hole.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("resources/polygon_triangle.txt").string(), std::filesystem::path(__FILE__).parent_path().append("resources/polygon_two_squares.txt").string()
};

#ifdef TEST_INFILL_SVG_OUTPUT
void writeTestcaseSVG(const InfillTestParameters& params)
{
    constexpr int buff_size = 1024;
    char buff[buff_size];
    std::snprintf(buff, buff_size, "/tmp/%s.svg", params.name.c_str());
    const std::string filename(buff);

    AABB aabb(params.outline_polygons);

    SVG svgFile(filename.c_str(), aabb);
    svgFile.writePolygons(params.outline_polygons, SVG::Color::BLUE);
    svgFile.nextLayer();
    svgFile.writePolylines(params.result_lines, SVG::Color::RED);
    svgFile.nextLayer();
    svgFile.writePolygons(params.result_polygons, SVG::Color::MAGENTA);
    // Note: SVG writes 'itself' when the object is destroyed.
}
#endif // TEST_INFILL_SVG_OUTPUT

InfillTestParameters generateInfillToTest(const InfillParameters& params, const size_t& test_polygon_id, const Polygons& outline_polygons)
{
    auto layers = std::vector<SlicerLayer>(200, SlicerLayer{});
    scripta::setAll(layers);

    const EFillMethod pattern = params.pattern;
    const bool zig_zagify = params.zig_zagify;
    const bool connect_polygons = params.connect_polygons;
    const coord_t line_distance = params.line_distance;

    Infill infill(pattern,
                  zig_zagify,
                  connect_polygons,
                  outline_polygons,
                  INFILL_LINE_WIDTH,
                  line_distance,
                  INFILL_OVERLAP,
                  INFILL_MULTIPLIER,
                  FILL_ANGLE,
                  Z,
                  SHIFT,
                  MAX_RESOLUTION,
                  MAX_DEVIATION); // There are some optional parameters, but these will do for now (future improvement?).

    Settings infill_settings;
    std::vector<VariableWidthLines> result_paths;
    Polygons result_polygons;
    Polygons result_lines;
    infill.generate(result_paths, result_polygons, result_lines, infill_settings, 1, SectionType::INFILL, nullptr, nullptr);

    InfillTestParameters result = InfillTestParameters(params, test_polygon_id, outline_polygons, result_lines, result_polygons);
    return result;
}

std::vector<InfillTestParameters> generateInfillTests()
{
    constexpr bool dont_zig_zaggify = false;
    constexpr bool do_connect_polygons = true;
    constexpr bool dont_connect_polygons = false;

    std::vector<Polygons> shapes;
    if (! readTestPolygons(POLYGON_FILENAMES, shapes))
    {
        return { InfillTestParameters() }; // return an invalid singleton, that'll trip up the 'file read' assertion in the TEST_P's
    }

    /* Skip methods:
     *  - that require the SierpinskyInfillProvider class, since these test classes aren't equipped to handle that yet
     *    this can be considered a TODO for these testcases here, not in the methods themselves
     *    (these are; Cross, Cross-3D and Cubic-Subdivision)
     *  - Gyroid, since it doesn't handle the 100% infill and related cases well
     *  - Concentric and ZigZag, since they now use a method that starts from an extra infill wall, which fail these tests (TODO!)
     */
    std::vector<EFillMethod> skip_methods = { EFillMethod::CONCENTRIC, EFillMethod::ZIG_ZAG, EFillMethod::CROSS, EFillMethod::CROSS_3D, EFillMethod::CUBICSUBDIV, EFillMethod::GYROID, EFillMethod::LIGHTNING };

    std::vector<EFillMethod> methods;
    for (int i_method = 0; i_method < static_cast<int>(EFillMethod::NONE); ++i_method)
    {
        const auto method = static_cast<EFillMethod>(i_method);
        if (std::find(skip_methods.begin(), skip_methods.end(), method) == skip_methods.end()) // Only use if not in skipped.
        {
            methods.push_back(method);
        }
    }

    std::vector<coord_t> line_distances = { 350, 400, 600, 800, 1200 };

    std::vector<InfillTestParameters> parameters_list;
    size_t test_polygon_id = 0;
    for (const Polygons& polygons : shapes)
    {
        for (const EFillMethod& method : methods)
        {
            for (const coord_t& line_distance : line_distances)
            {
                parameters_list.push_back(generateInfillToTest(InfillParameters(method, dont_zig_zaggify, dont_connect_polygons, line_distance), test_polygon_id, polygons));
                parameters_list.push_back(generateInfillToTest(InfillParameters(method, dont_zig_zaggify, do_connect_polygons, line_distance), test_polygon_id, polygons));
                // parameters_list.push_back(generateInfillToTest(InfillParameters(method, do_zig_zaggify, dont_connect_polygons,
                // line_distance), test_polygon_id, polygons)); parameters_list.push_back(generateInfillToTest(InfillParameters(method,
                // do_zig_zaggify, do_connect_polygons, line_distance), test_polygon_id, polygons));
                //  TODO: Re-enable when the extra infill walls are fully debugged or the discrepancy in the tests is explained.
            }
        }
        ++test_polygon_id;
    }

    return parameters_list;
}

class InfillTest : public testing::TestWithParam<InfillTestParameters>
{
};

INSTANTIATE_TEST_SUITE_P(InfillTestcases, InfillTest, testing::ValuesIn(generateInfillTests()), [](const testing::TestParamInfo<InfillTestParameters>& info) { return info.param.name; });

TEST_P(InfillTest, TestInfillSanity)
{
    InfillTestParameters params = GetParam();

#ifdef TEST_INFILL_SVG_OUTPUT
    writeTestcaseSVG(params);
#endif // TEST_INFILL_SVG_OUTPUT

    ASSERT_TRUE(params.valid) << params.fail_reason;
    ASSERT_FALSE(params.result_polygons.empty() && params.result_lines.empty()) << "Infill should have been generated.";

    long double worst_case_zig_zag_added_area = 0;
    if (params.params.zig_zagify || params.params.pattern == EFillMethod::ZIG_ZAG)
    {
        worst_case_zig_zag_added_area = params.outline_polygons.polygonLength() * INFILL_LINE_WIDTH;
    }

    const double min_available_area = std::abs(params.outline_polygons.offset(static_cast<int>(-params.params.line_distance) / 2).area());
    const long double max_available_area = std::abs(params.outline_polygons.offset(static_cast<int>(params.params.line_distance) / 2).area()) + worst_case_zig_zag_added_area;
    const long double min_expected_infill_area = (min_available_area * static_cast<long double>(INFILL_LINE_WIDTH)) / params.params.line_distance;
    const long double max_expected_infill_area = (max_available_area * INFILL_LINE_WIDTH) / params.params.line_distance + worst_case_zig_zag_added_area;

    const long double out_infill_area = ((params.result_polygons.polygonLength() + params.result_lines.polyLineLength()) * static_cast<long double>(INFILL_LINE_WIDTH)) / getPatternMultiplier(params.params.pattern);

    ASSERT_GT((coord_t)max_available_area, (coord_t)out_infill_area) << "Infill area should allways be less than the total area available.";
    ASSERT_GT((coord_t)out_infill_area, (coord_t)min_expected_infill_area) << "Infill area should be greater than the minimum area expected to be covered.";
    ASSERT_LT((coord_t)out_infill_area, (coord_t)max_expected_infill_area) << "Infill area should be less than the maximum area to be covered.";

    const coord_t maximum_error = 10_mu; // potential rounding error
    const Polygons padded_shape_outline = params.outline_polygons.offset(INFILL_LINE_WIDTH / 2);
    constexpr bool restitch = false; // No need to restitch polylines - that would introduce stitching errors.
    ASSERT_LE(std::abs(padded_shape_outline.intersectionPolyLines(params.result_lines, restitch).polyLineLength() - params.result_lines.polyLineLength()), maximum_error) << "Infill (lines) should not be outside target polygon.";
    Polygons result_polygon_lines = params.result_polygons;
    for (PolygonRef poly : result_polygon_lines)
    {
        poly.add(poly.front());
    }
    ASSERT_LE(std::abs(padded_shape_outline.intersectionPolyLines(result_polygon_lines, restitch).polyLineLength() - result_polygon_lines.polyLineLength()), maximum_error) << "Infill (lines) should not be outside target polygon.";
}

} // namespace cura
// NOLINTEND(*-magic-numbers)