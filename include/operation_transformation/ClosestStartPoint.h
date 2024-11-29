// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_CLOSESTPOINT_H
#define PATHPROCESSING_CLOSESTPOINT_H

#include "operation_transformation/StartCandidatePoint.h"

namespace cura
{

struct ClosestStartPoint
{
    coord_t distance_squared;
    StartCandidatePoint point;
};

} // namespace cura

#endif // PATHPROCESSING_CLOSESTPOINT_H
