//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include "../src/infill.h"

namespace cura
{
    // TODO: first test single infill, then parameterize (see examples)

    // "../tests/polygons.txt"  // TODO in input: should path be closed? should we have multiple paths per polygon, so we can test polygons with holes in them?
    bool readTestPolygons(const std::string& filename, std::vector<Polygons>& polygons_out)
    {
        FILE* handle = std::fopen(filename.c_str(), "r");
        if (! handle)
        {
            return false;
        }
        
        Polygon next;

        char command = '_';
        while (command != '#')
        {
            std::fscanf(handle, " %c ", &command);
            switch (command)
            {
            case 'v': // read coordinate
                coord_t coord_x, coord_y;
                std::fscanf(handle, " %ld %ld ", &coord_x, &coord_y);
                next.emplace_back(coord_x, coord_y);
                break;
            case '&': // feed next polygon
                if (! next.empty())
                {
                    Polygons polys;
                    polys.add(Polygon(next)); // copy
                    polygons_out.push_back(polys);
                    next.clear();
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
        const bool zig_zaggify = false; // param
        const bool connect_polygons = false; // param
        const Polygons outline = OUTLINE; // param
        const coord_t outline_offset = 0;
        const coord_t infill_line_width = 350;
        const coord_t line_distance = 350;
        const coord_t infill_overlap = 0;
        const size_t infill_multiplier = 1;
        const AngleDegrees fill_angle = 0;
        const coord_t z = 100; // param? (for some)
        const coord_t shift = 0;

        Infill infill
        (
            cura::EFillMethod::LINES /*param*/,
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
    }

} //namespace cura