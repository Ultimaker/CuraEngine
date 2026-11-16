// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <cstddef>

namespace curaengine
{
class ClosePolygonResult
{ // The result of trying to find a point on a closed polygon line. This gives back the point index, the polygon index, and the point of the connection.
  // The line on which the point lays is between pointIdx-1 and pointIdx
public:
    size_t polygonIdx = 0;
    size_t pointIdx = 0;
};

} // namespace curaengine
