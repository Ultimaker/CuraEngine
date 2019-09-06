//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef READ_TEST_POLYGONS_H
#define READ_TEST_POLYGONS_H

#include <string>
#include <vector>

#include "../src/utils/polygon.h"

namespace cura
{
	bool readTestPolygons(const std::string& filename, std::vector<Polygons>& polygons_out);
}

#endif // READ_TEST_POLYGONS_H
