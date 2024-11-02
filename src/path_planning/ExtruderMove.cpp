// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_planning/ExtruderMove.h"

#include "path_planning/ExtruderMoveSet.h"
#include "path_planning/FeatureExtrusion.h"
#include "path_planning/LayerPlan.h"

namespace cura
{

ExtruderMove::ExtruderMove(const Point3LL& position)
    : position_(position)
{
}

Point3LL ExtruderMove::getAbsolutePosition(const LayerPlan& layer_plan, const ExtruderMoveSet& extruder_move_set) const
{
    Point3LL absolute_position = position_;
    absolute_position.z_ += layer_plan.getZ() + extruder_move_set.getZOffset();
    return absolute_position;
}

const Point3LL& ExtruderMove::getPosition() const
{
    return position_;
}

} // namespace cura
