// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "geometry/Point2LL.h"

namespace curaengine
{

class CombPath : public std::vector<Point2LL> //!< A single path either inside or outise the parts
{
public:
    bool cross_boundary = false; //!< Whether the path crosses a boundary.
};

} // namespace curaengine
