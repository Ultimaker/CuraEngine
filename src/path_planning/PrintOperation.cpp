// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_planning/PrintOperation.h"

#include "geometry/Point3LL.h"

namespace cura
{

void PrintOperation::applyProcessors(const std::vector<const PrintOperation*>& /*parents*/)
{
    // Default behavior is do nothing
}

std::optional<Point3LL> PrintOperation::findStartPosition() const
{
    return std::nullopt; // Default behavior is having an unknown start position
}

std::optional<Point3LL> PrintOperation::findEndPosition() const
{
    return std::nullopt; // Default behavior is having an unknown end position
}

} // namespace cura
