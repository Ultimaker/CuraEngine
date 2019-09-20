//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ReadTestPolygons.h"

#include <cstdio>

// NOTE: See the documentation in the header-file for an explanation of this simple file format.

namespace cura
{
    // Read multiple files to the collection of polygons.
    // Returns boolean success/failure (read errors, not found, etc.).
    bool readTestPolygons(const std::vector<std::string>& filenames, std::vector<Polygons>& polygons_out)
    {
        for (const std::string& filename : filenames)
        {
            if (!readTestPolygons(filename, polygons_out))
            {
                return false;
            }
        }
        return true;
    }

    // Read a single file to the collection of polygons.
    // Returns boolean success/failure (read errors, not found, etc.).
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
            case 'x': // close 'next' loop
                // fallthrough
            case '&': // finalize 'next' polygon (which may also close a loop)
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
} // namespace cura
