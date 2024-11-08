// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_planning/ExtruderMove.h"

#include "path_planning/ExtruderMoveSequence.h"
#include "path_planning/FeatureExtrusion.h"
#include "path_planning/LayerPlan.h"

namespace cura
{

ExtruderMove::ExtruderMove(const Point3LL& position)
    : position_(position)
{
}

const Point3LL& ExtruderMove::getPosition() const
{
    return position_;
}

std::optional<Point3LL> ExtruderMove::findEndPosition() const
{
    return position_;
}

} // namespace cura
