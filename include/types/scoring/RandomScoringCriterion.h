// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/scoring/ScoringCriterion.h"

namespace curaengine::types::scoring
{

/*!
 * Criterion that will give a random score whatever the element is.
 * The code is in the cpp file, so this class can only be used with the types of elements that are instantiated at the end of it.
 * \tparam T The type of the elements that are scored
 */
template<typename T>
class RandomScoringCriterion : public ScoringCriterion<T>
{
public:
    explicit RandomScoringCriterion() = default;

    double computeScore(const T& element) const override;
};

} // namespace curaengine::types::scoring
