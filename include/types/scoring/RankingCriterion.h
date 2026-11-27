// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <cstddef>
#include <cstdint>

namespace curaengine::types::scoring
{

/*!
 * Base class for implementing a selection criterion that puts the elements of a list into ranks, the highest rank being
 * the best one. It is used when elements are either acceptable or not, and not when they have to be finely compared.
 */
class RankingCriterion
{
public:
    RankingCriterion() = default;

    virtual ~RankingCriterion() = default;

    /*!
     * \brief Computes the rank of an element regarding this criterion
     * \param candidate_index The index of the candidate of the original list
     * \return The rank of the element, the higher the better
     */
    virtual uint8_t computeRank(const size_t candidate_index) const = 0;
};

} // namespace curaengine::types::scoring
