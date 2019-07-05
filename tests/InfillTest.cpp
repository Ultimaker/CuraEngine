//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include "../src/infill.h"

namespace cura
{
    // TODO: first test single infill, then parameterize (see examples)

    // TODO in input: should path be closed?
    bool readTestPolygons(const std::string& filename, std::vector<Polygons>& polygons_out)
    {
        FILE* handle = std::fopen(filename.c_str(), "r");
        if (! handle)
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
                if (! next_path.empty())
                {
                    next_shape.add(Polygon(next_path)); // copy and add
                    next_path.clear();
                }
                if (command == '&' && ! next_shape.empty())
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

    TEST(Infill, TestVerticesInside)
    {
        const EFillMethod pattern = cura::EFillMethod::LINES;  // param
        const bool zig_zagify = false; // param
        const bool conect_polygons = false; // param
        //const Polygons outline = OUTLINE; // param
        const coord_t outline_offset = 0;
        const coord_t infill_line_width = 350;
        const coord_t line_distance = 1200; //350;
        const coord_t infill_overlap = 0;
        const size_t infill_multiplier = 1;
        const AngleDegrees fill_angle = 0;
        const coord_t z = 100; // param? (for some)
        const coord_t shift = 0;

        std::vector<Polygons> polygons;
        ASSERT_TRUE(readTestPolygons("../tests/polygons.txt", polygons)) << "Read of file with test polygons failed, can't continue tests.";

        for (const auto& outline : polygons)
        {
            Infill infill
            (
                pattern,
                zig_zagify,
                conect_polygons,
                outline,
                outline_offset,
                infill_line_width,
                line_distance,
                infill_overlap,
                infill_multiplier,
                fill_angle,
                z,
                shift
            ); // There are some optional parameters, but these will do for now.

            Polygons result_polygons;
            Polygons result_lines;

            // TODO? Test also with (fake) SliceDataStorage so test is close to 'actual' mesh (not just helpers)?
            infill.generate(result_polygons, result_lines, nullptr, nullptr);

            ASSERT_FALSE(result_polygons.empty() && result_lines.empty()) << "Infill should have been generated.";

            double total_area = 0;
            for (PolygonRef poly : result_lines)
            {
                Point last_line = poly[0];
                for (Point line : poly)
                {
                    // don't have to skip the 0th line, as it'll have an area of 0 anyway
                    const double vector_x = last_line.X - line.X;
                    const double vector_y = last_line.Y - line.Y;
                    total_area += std::sqrt(vector_x * vector_x + vector_y * vector_y) * infill_line_width;

                    last_line = line;
                }
            }

            ASSERT_GT(total_area, 0) << "Infill area should occupy at least some space.";
            ASSERT_GT(std::abs(outline.area()), total_area) << "There shouldn't be more infill than area to fill.";
            // TODO: compare (fuzzyly) with 'percentage'/expected, not just shape total
        }
    }

} //namespace cura
