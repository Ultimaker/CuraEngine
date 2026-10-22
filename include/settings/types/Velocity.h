// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "utils/types/NumericFacade.h"

namespace cura
{

/*
 * Represents a velocity in millimetres per second.
 *
 * This is a facade. It behaves like a double, only it can't be negative.
 */
struct Velocity : public utils::NumericFacade<double>
{
    using base_type = utils::NumericFacade<double>;
    using base_type::NumericFacade;

    constexpr Velocity(const base_type& base) noexcept
        : base_type{ base } {};
};

} // namespace cura
