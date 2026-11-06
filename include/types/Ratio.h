// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "types/NumericFacade.h"


namespace curaengine::types
{

/*
 * \brief Represents a ratio between two numbers.
 *
 * This is a facade. It behaves like a double.
 */
struct Ratio : public NumericFacade<double>
{
    using base_type = NumericFacade<double>;
    using base_type::NumericFacade;

    constexpr Ratio(const base_type& base) noexcept
        : base_type{ base } {};

    /*!
     * Create the Ratio with a numerator and a divisor from arbitrary types
     * \param numerator the numerator of the ratio
     * \param divisor the divisor of the ratio
     */
    constexpr Ratio(const numeric auto numerator, const numeric auto divisor)
        : base_type{ static_cast<value_type>(numerator) / static_cast<value_type>(divisor) } {};
};

constexpr Ratio operator""_r(const long double ratio)
{
    return { ratio };
}

} // namespace curaengine::types
