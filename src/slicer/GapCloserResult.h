/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef SLICER_GAP_CLOSER_RESULT_H
#define SLICER_GAP_CLOSER_RESULT_H

#include "../utils/intpoint.h"

namespace cura
{

class GapCloserResult
{
public:
    int64_t len = -1;
    int polygonIdx = -1;
    unsigned int pointIdxA = -1;
    unsigned int pointIdxB = -1;
    bool AtoB = false;
};

} // namespace cura

#endif // SLICER_GAP_CLOSER_RESULT_H