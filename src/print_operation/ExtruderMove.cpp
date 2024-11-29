// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/ExtruderMove.h"

#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/FeatureExtrusion.h"
#include "print_operation/LayerPlan.h"

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

void ExtruderMove::setPosition(const Point3LL& position)
{
    position_ = position;
}

std::optional<Point3LL> ExtruderMove::findEndPosition() const
{
    return position_;
}

} // namespace cura
