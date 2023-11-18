// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef RATIO_H
#define RATIO_H

#include "utils/types/numeric_facade.h"


namespace cura
{

/*
 * \brief Represents a ratio between two numbers.
 *
 * This is a facade. It behaves like a double.
 */
struct Ratio : public utils::NumericFacade<double>
{
    using base_type = utils::NumericFacade<double>;
    using base_type::NumericFacade;

    constexpr Ratio(const base_type& base) noexcept
        : base_type{ base } {};

    /*!
     * Create the Ratio with a numerator and a divisor from arbitrary types
     * \param numerator the numerator of the ratio
     * \param divisor the divisor of the ratio
     */
    constexpr Ratio(const utils::numeric auto numerator, const utils::numeric auto divisor)
        : base_type{ static_cast<value_type>(numerator) / static_cast<value_type>(divisor) } {};
};

constexpr Ratio operator"" _r(const long double ratio)
{
    return { ratio };
}

} // namespace cura

#endif // RATIO_H
