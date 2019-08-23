//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include "../src/infill.h"

namespace cura
{
    bool readTestPolygons(const std::string& filename, std::vector<Polygons>& polygons_out)
    {
        FILE* handle = std::fopen(filename.c_str(), "r");
        if (!handle)
        {
            return false;
        }

        Polygon next_path;
        Polygons next_shape;

        char command = '_';
        int read = 0;
        while (command != '#')
        {
            read = std::fscanf(handle, " %c ", &command);
            if (read == EOF)
            {
                command = '#';
            }
            else if (read <= 0)
            {
                return false;
            }
            switch (command)
            {
            case 'v': // read next coordinate
                coord_t coord_x, coord_y;
                read = std::fscanf(handle, " %lld %lld ", &coord_x, &coord_y);
                if (read == EOF || read <= 0)
                {
                    return false;
                }
                next_path.emplace_back(coord_x, coord_y);
                break;
            case 'x': // close 'next' path
                // fallthrough
            case '&': // finalize 'next' polygon (which may also close a path)
                // fallthrough
            case '#': // end of file
                if (!next_path.empty())
                {
                    next_shape.add(Polygon(next_path)); // copy and add
                    next_path.clear();
                }
                if (command != 'x' && !next_shape.empty())
                {
                    polygons_out.push_back(Polygons(next_shape)); // copy and add
                    next_shape.clear();
                }
                break;
            default:
                return false;
            }
        }

        return true;
    }

    template<typename ... Ts>
    std::string makeName(const std::string& format_string, Ts ... args)
    {
        constexpr int buff_size = 1024;
        char buff[buff_size];
        std::snprintf(buff, buff_size, format_string.c_str(), args...);
        return std::string(buff);
    }

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

    struct InfillParameters
    {
    public:
        // Actual infill parameters:
        EFillMethod pattern;
        bool zig_zagify;
        bool connect_polygons;
        coord_t line_distance;

        std::string name;

        InfillParameters(const EFillMethod& pattern, const bool& zig_zagify, const bool& connect_polygons, const coord_t& line_distance) :
            pattern(pattern),
            zig_zagify(zig_zagify),
            connect_polygons(connect_polygons),
            line_distance(line_distance)
        {
            name = makeName("InfillParameters_%d_%d_%d_%lld", (int)pattern, (int)zig_zagify, (int)connect_polygons, line_distance);
        }
    };

    class InfillTestParameters
    {
    public:
        bool valid;  // <-- if the file isn't read (or anything else goes wrong with the setup) we can communicate it to the tests
        std::string fail_reason;
        size_t test_polygon_id;

        // Parameters used to generate the infill:
        InfillParameters params;
        Polygons outline_polygons;

        // Resulting infill:
        Polygons result_lines;
        Polygons result_polygons;

        std::string name;

        InfillTestParameters() :
            valid(false),
            fail_reason("Read of file with test polygons failed (see generateInfillTests), can't continue tests."),
            test_polygon_id(-1),
            params(InfillParameters(EFillMethod::NONE, false, false, 0)),
            outline_polygons(Polygons()),
            result_lines(Polygons()),
            result_polygons(Polygons()),
            name("UNNAMED")
        {
        }

        InfillTestParameters(const InfillParameters& params, const size_t& test_polygon_id, const Polygons& outline_polygons, const Polygons& result_lines, const Polygons& result_polygons) :
            valid(true),
            fail_reason("__"),
            test_polygon_id(test_polygon_id),
            params(params),
            outline_polygons(outline_polygons),
            result_lines(result_lines),
            result_polygons(result_polygons)
        {
            name = makeName("InfillTestParameters_P%d_Z%d_C%d_L%lld__%lld", (int)params.pattern, (int)params.zig_zagify, (int)params.connect_polygons, params.line_distance, test_polygon_id);
        }

        friend std::ostream& operator<<(std::ostream& os, const InfillTestParameters& params)
        {
            return os << params.name << "(" << (params.valid ? std::string("input OK") : params.fail_reason) << ")";
        }
    };

    constexpr coord_t outline_offset = 0;
    constexpr coord_t infill_line_width = 350;
    constexpr coord_t infill_overlap = 0;
    constexpr size_t infill_multiplier = 1;
    const AngleDegrees fill_angle = 0.;
    constexpr coord_t z = 100; // Future improvement: Also take an uneven layer, so we get the alternate.
    constexpr coord_t shift = 0;

    InfillTestParameters generateInfillToTest(const InfillParameters& params, const size_t& test_polygon_id, const Polygons& outline_polygons)
    {
        const EFillMethod pattern = params.pattern;
        const bool zig_zagify = params.zig_zagify;
        const bool connect_polygons = params.connect_polygons;
        const coord_t line_distance = params.line_distance;

        Infill infill
        (
            pattern,
            zig_zagify,
            connect_polygons,
            outline_polygons,
            outline_offset,
            infill_line_width,
            line_distance,
            infill_overlap,
            infill_multiplier,
            fill_angle,
            z,
            shift
        ); // There are some optional parameters, but these will do for now (future improvement?).

        Polygons result_polygons;
        Polygons result_lines;
        infill.generate(result_polygons, result_lines, nullptr, nullptr);

        InfillTestParameters result = InfillTestParameters(params, test_polygon_id, outline_polygons, result_lines, result_polygons);
        return result;
    }

    std::vector<InfillTestParameters> generateInfillTests()
    {
        std::vector<Polygons> shapes;
        if (!readTestPolygons("../tests/polygons.txt", shapes))
        {
            return { InfillTestParameters() };  // return an invalid class, that'll trip up the 'file read' assertion in the TEST_P's
        }

        std::vector<EFillMethod> methods;
        std::vector<EFillMethod> skip_methods = { EFillMethod::CROSS, EFillMethod::CROSS_3D, EFillMethod::CUBICSUBDIV }; //TODO: Support for testing infill that needs the sierpinski-provider!
        for (int i_method = 0; i_method < static_cast<int>(EFillMethod::NONE); ++i_method)
        {
            const EFillMethod method = static_cast<EFillMethod>(i_method);
            if (std::find(skip_methods.begin(), skip_methods.end(), method) == skip_methods.end()) // Only use if not in skipped.
            {
                methods.push_back(method);
            }
        }

        std::vector<coord_t> line_distances = { 400, 600, 800, 1200 }; // TODO?: Gyroid fails the 'fill less than 100% of area available' with values close to the line width, like 350.

        std::vector<InfillTestParameters> parameters_list;
        size_t test_polygon_id = 0;
        for (const Polygons& polygons : shapes)
        {
            for (const EFillMethod& method : methods)
            {
                for (const coord_t& line_distance : line_distances)
                {
                    parameters_list.push_back(generateInfillToTest(InfillParameters(method, false, false, line_distance), test_polygon_id, polygons));
                    parameters_list.push_back(generateInfillToTest(InfillParameters(method, false, true, line_distance), test_polygon_id, polygons));
                    parameters_list.push_back(generateInfillToTest(InfillParameters(method, true, false, line_distance), test_polygon_id, polygons));
                    parameters_list.push_back(generateInfillToTest(InfillParameters(method, true, true, line_distance), test_polygon_id, polygons));
                }
            }
            ++test_polygon_id;
        }

        return parameters_list;
    }

    class InfillTest : public testing::TestWithParam<InfillTestParameters> {};

    INSTANTIATE_TEST_CASE_P(InfillTestcases, InfillTest, testing::ValuesIn(generateInfillTests()), [](testing::TestParamInfo<InfillTestParameters> info) { return info.param.name; });

    TEST_P(InfillTest, TestInfillSanity)
    {
        InfillTestParameters params = GetParam();
        ASSERT_TRUE(params.valid) << params.fail_reason;
        ASSERT_FALSE(params.result_polygons.empty() && params.result_lines.empty()) << "Infill should have been generated.";

        const double available_area = std::abs(params.outline_polygons.area());
        const double expected_infill_area = (available_area * infill_line_width) / params.params.line_distance;
        const double total_infill_area = (params.result_polygons.polygonLength() + params.result_lines.polyLineLength()) * infill_line_width / getPatternMultiplier(params.params.pattern);
        ASSERT_GT((coord_t)available_area, (coord_t)total_infill_area) << "Infill area should allways be less than the total area available.";
        ASSERT_NEAR((coord_t)total_infill_area, (coord_t)expected_infill_area, (coord_t)(total_infill_area * 0.15)) << "Infill area should be within 15% of expected size."; // TODO: 10 ~ 20% is actually quite bad?
        
        const Polygons padded_shape_outline = params.outline_polygons.offset(infill_line_width / 2);
        ASSERT_EQ(padded_shape_outline.intersectionPolyLines(params.result_lines).polyLineLength(), params.result_lines.polyLineLength()) << "Infill (lines) should not be outside target polygon.";
        ASSERT_EQ(params.result_polygons.difference(padded_shape_outline).area(), 0) << "Infill (polys) should not be outside target polygon.";
    }

} //namespace cura
