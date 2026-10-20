// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "geometry/Point2LL.h"

namespace cura
{

class CombPath : public std::vector<Point2LL> //!< A single path either inside or outise the parts
{
public:
    bool cross_boundary = false; //!< Whether the path crosses a boundary.
};

} // namespace cura
