// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <cstddef>
#include <vector>

namespace curaengine::types::scoring
{

/*!
 * Contains the index of an element in the source list, and its calculated score
 */
struct Candidate
{
    size_t candidate_index;
    double score{ 0.0 };
};

/*!
 * Base class for a selection pass, which takes a list of candidates and keeps only the best ones. Passes are chained by
 * the BestElementFinder, each one working on the candidates that were kept by the previous one.
 */
class CriteriaPass
{
public:
    CriteriaPass() = default;

    virtual ~CriteriaPass() = default;

    /*!
     * Processes the candidates and removes those that are not good enough. The remaining ones are sorted from the best
     * to the worst, so that the first one is the best candidate of this pass, and the following passes (and the final
     * selection) can use the previous order to differentiate candidates that they consider equal.
     * \param candidates The candidates to be processed, which are replaced by the ones that are kept
     */
    virtual void apply(std::vector<Candidate>& candidates) const = 0;
};

} // namespace curaengine::types::scoring
