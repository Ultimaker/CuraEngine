// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef RANDOMSCORINGCRITERION_H
#define RANDOMSCORINGCRITERION_H

#include <stddef.h>

#include "utils/scoring/ScoringCriterion.h"

namespace cura
{

/*!
 * Criterion that will give a random score whatever the element is.
 */
class RandomScoringCriterion : public ScoringCriterion
{
public:
    explicit RandomScoringCriterion();

    virtual double computeScore(const size_t candidate_index) const override;
};

} // namespace cura

#endif // RANDOMSCORINGCRITERION_H
