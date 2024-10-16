// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_SCORE_H
#define UTILS_SCORE_H

#include <fmt/format.h>

#include "CriterionScore.h"

namespace cura
{

/*!
 * This class represents a score to be calculated over different criteria, to select the best candidate among a list.
 */
class Score
{
private:
    double value_{ 0.0 };

public:
    /*!
     * Get the actual score value, should be used for debug purposes only
     */
    double getValue() const
    {
        return value_;
    }

    /*!
     * Add the calculated score of an inidividual criterion to the global score, taking care of its weight
     */
    void operator+=(const CriterionScore& criterion_score)
    {
        value_ += criterion_score.score * criterion_score.weight;
    }

    /*!
     * Comparison operators to allow selecting the best global score
     */
    auto operator<=>(const Score&) const = default;
};

} // namespace cura

namespace fmt
{

template<>
struct formatter<cura::Score> : formatter<std::string>
{
    auto format(const cura::Score& score, format_context& ctx)
    {
        return fmt::format_to(ctx.out(), "Score{{{}}}", score.getValue());
    }
};

} // namespace fmt

#endif // UTILS_SCORE_H
