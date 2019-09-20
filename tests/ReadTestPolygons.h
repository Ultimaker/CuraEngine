//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef READ_TEST_POLYGONS_H
#define READ_TEST_POLYGONS_H

#include <string>
#include <vector>

#include "../src/utils/polygon.h"

/* A reader for a very simple 2D polygon format, usefull for tests.
 * It's currently uesd to read all the './tests/resources/polygon_*.txt' files.
 *
 * The format of a file needs to be as follows:
 *  - specify the next vertex in the smae loop with "v <coord> <coord>\n"
 *    each subsequent vertex is assumed to be connected to the last with a line-segment
 *    unless the loop is closed by the following:
 *  - specify the end of a loop within the same polygon with "x\n"
 *    this is useful if you need to make polygons with holes, or 'polygons' that otherwise have multiple parts
 *  - specify the end of a polygon with '&\n'
 *  - specify the end of the file with '#\n'
 *
 * A small example:
v 10000 10000
v 10000 50000
v 50000 50000
#
 *
 * Note that:
 *  - loops don't have to be closed with a vertex identical to the last, a loop is assumed to be closed.
 *  - the file-end will also close both the polygon and the loop, and the polygon-end will close loops as well.
 *  - comments can be added after the 'end' of the file.
 */

namespace cura
{
    bool readTestPolygons(const std::vector<std::string>& filenames, std::vector<Polygons>& polygons_out);
    bool readTestPolygons(const std::string& filename, std::vector<Polygons>& polygons_out);
}

#endif // READ_TEST_POLYGONS_H
