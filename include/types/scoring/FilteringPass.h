// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <vector>

#include "types/scoring/CriteriaPass.h"

namespace curaengine::types::scoring
{

/*!
 * Base class for passes that only remove the candidates that are not acceptable, without calculating any score. The remaining
 * candidates are kept in the same order, so that the previous passes can still be used to differentiate them. How the candidates
 * are filtered is up to the sub-classes.
 * \tparam T The type of the elements that are processed
 */
template<typename T>
class FilteringPass : public CriteriaPass<T>
{
public:
    void apply(std::vector<Candidate<T>>& candidates) const final
    {
        if (! candidates.empty())
        {
            filter(candidates);
        }
    }

protected:
    /*!
     * Removes the candidates that are not acceptable
     * \param candidates The candidates to be filtered, which are never empty. They must be replaced by the ones that are kept, in the same order
     */
    virtual void filter(std::vector<Candidate<T>>& candidates) const = 0;
};

} // namespace curaengine::types::scoring
