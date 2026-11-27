// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <vector>

#include "types/Coord_t.h"
#include "types/geometry/Point2LL.h"
#include "types/scoring/ScoringCriterion.h"

namespace curaengine::types::scoring
{

/*!
 * Criterion that will give a score according to how well the position of the element is vertically aligned with the seams of the last layer.
 * The code is in the cpp file, so this class can only be used with the types of elements that are instantiated at the end of it.
 * \tparam T The type of the elements that are scored, which must have a position (see getElementPosition())
 */
template<typename T>
class VerticalAlignmentScoringCriterion : public ScoringCriterion<T>
{
public:
    explicit VerticalAlignmentScoringCriterion(const std::vector<types::geometry::Point2LL>& last_layer_seams, const types::coord_t max_alignment_distance);

    double computeScore(const T& element) const override;

private:
    std::vector<types::geometry::Point2LL> last_layer_seams_;
    const types::coord_t max_alignment_distance_;
};

} // namespace curaengine::types::scoring
