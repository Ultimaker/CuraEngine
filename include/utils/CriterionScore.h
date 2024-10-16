// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_CRITERION_SCORE_H
#define UTILS_CRITERION_SCORE_H

#include <cassert>

#include "SparsePointGrid.h"

namespace cura
{

struct CriterionScore
{
    double score{ 0.0 };
    double weight{ 0.0 };
};

} // namespace cura
#endif // UTILS_CRITERION_SCORE_H
