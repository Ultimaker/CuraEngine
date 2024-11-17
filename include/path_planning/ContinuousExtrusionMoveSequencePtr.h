// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_CONTINUOUSEXTRUDERMOVESEQUENCEPTR_H
#define PATHPLANNING_CONTINUOUSEXTRUDERMOVESEQUENCEPTR_H

#include <memory>

namespace cura
{

class ContinuousExtrusionMoveSequence;

using ContinuousExtrusionMoveSequencePtr = std::shared_ptr<ContinuousExtrusionMoveSequence>;
using ConstContinuousExtrusionMoveSequencePtr = std::shared_ptr<const ContinuousExtrusionMoveSequence>;

} // namespace cura

#endif // PATHPLANNING_CONTINUOUSEXTRUDERMOVESEQUENCEPTR_H
