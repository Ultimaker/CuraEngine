// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <memory>
#include <optional>
#include <span>
#include <vector>

#include "types/scoring/CriteriaPass.h"

namespace curaengine::types::scoring
{
template<typename T>
class ScoringCriterion;

/*!
 * This class implements an algorithm to find an element amongst a list, regarding one or multiple selection passes. The
 * passes are implemented by subclassing the CriteriaPass class, for example with scoring criteria (see
 * WeighedScoringCriteriaPass) or by filtering the candidates (see FilteringPass). Thus, if the first pass gives a few best
 * candidates that are too close to each other, we keep only the best candidates and process a second pass with different
 * criteria, and so on until we have a single outsider, or no more passes.
 *
 * The code is in the cpp file, so this class can only be used with the types of elements that are instantiated at the end of it.
 * \tparam T The type of the elements to be selected
 */
template<typename T>
class BestElementFinder
{
private:
    std::vector<std::shared_ptr<CriteriaPass<T>>> passes_;

public:
    explicit BestElementFinder() = default;

    void appendCriteriaPass(const std::shared_ptr<CriteriaPass<T>>& pass);

    /*!
     * Tells whether there is no pass, in which case all the elements are always kept
     */
    [[nodiscard]] bool isEmpty() const;

    /*!
     * Convenience method to add a weighed scoring pass with a single criterion
     */
    void appendSingleCriterionPass(std::shared_ptr<ScoringCriterion<T>> criterion, const std::optional<double>& outsider_delta_threshold = 0.0);

    /*!
     * Runs all the passes on the elements, and returns the best one
     * \param elements The elements to select from, which have to remain alive and unchanged as long as the result is used
     * \return The best candidate, which gives both the index of the element in the given list and a pointer to it, or nullopt if no element remains
     */
    [[nodiscard]] std::optional<Candidate<T>> findBestElement(const std::span<const T> elements) const;

    /*!
     * Runs all the passes on the elements, and returns all the ones that remain
     * \param elements The elements to select from, which have to remain alive and unchanged as long as the result is used
     * \return The remaining candidates, from the best one to the worst one. The score of each one is the one that was
     * calculated by the last pass that calculates scores, which may not be significant if there is none
     */
    [[nodiscard]] std::vector<Candidate<T>> findBestElements(const std::span<const T> elements) const;
};

} // namespace curaengine::types::scoring
