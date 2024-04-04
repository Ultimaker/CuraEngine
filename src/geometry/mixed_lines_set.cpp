// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/mixed_lines_set.h"


namespace cura
{

void MixedLinesSet::push_back(const MixedLinesSet& lines)
{
    push_back(lines.getOpenLines());
    push_back(lines.getClosedLines());
}

} // namespace cura
