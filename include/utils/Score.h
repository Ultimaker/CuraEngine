// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_SCORE_H
#define UTILS_SCORE_H

#include <fmt/format.h>

#include "CriterionScore.h"

namespace cura
{

class Score
{
private:
    double value_{ 0.0 };

public:
    double getValue() const
    {
        return value_;
    }

    void operator+=(const CriterionScore& criterion_score)
    {
        value_ += criterion_score.score * criterion_score.weight;
    }

    auto operator<=>(const Score&) const = default;

    double operator-(const Score& other) const
    {
        return value_ - other.value_;
    }
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
