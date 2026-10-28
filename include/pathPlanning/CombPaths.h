// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "CombPath.h"

namespace curaengine
{

class CombPaths : public std::vector<CombPath> //!< A list of paths alternating between inside a part and outside a part
{
public:
    bool throughAir = false; //!< Whether the path is one which moves through air.
};

} // namespace curaengine
