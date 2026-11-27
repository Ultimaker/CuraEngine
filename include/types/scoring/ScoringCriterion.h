// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

namespace curaengine::types::scoring
{

/*!
 * Base class for implementing a selection criterion when calculating a multi-criteria score to select the best element
 * amongst a list.
 * \tparam T The type of the elements that are scored
 */
template<typename T>
class ScoringCriterion
{
public:
    ScoringCriterion() = default;

    virtual ~ScoringCriterion() = default;

    /*!
     * \brief Computes the score of an element regarding this criterion. To ensure a proper selection, this value must
     *        be contained in [0.0, 1.0] and the different given scores must be evenly distributed in this range.
     * \param element The element to be scored
     * \return The raw score of the element regarding this criterion
     */
    virtual double computeScore(const T& element) const = 0;
};

} // namespace curaengine::types::scoring
