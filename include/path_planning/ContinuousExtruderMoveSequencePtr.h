// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_CONTINUOUSEXTRUDERMOVESEQUENCEPTR_H
#define PATHPLANNING_CONTINUOUSEXTRUDERMOVESEQUENCEPTR_H

#include <memory>

namespace cura
{

class ContinuousExtruderMoveSequence;

using ContinuousExtruderMoveSequencePtr = std::shared_ptr<ContinuousExtruderMoveSequence>;
using ConstContinuousExtruderMoveSequencePtr = std::shared_ptr<const ContinuousExtruderMoveSequence>;

} // namespace cura

#endif // PATHPLANNING_CONTINUOUSEXTRUDERMOVESEQUENCEPTR_H
