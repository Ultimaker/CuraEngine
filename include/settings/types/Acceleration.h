// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "utils/types/NumericFacade.h"

namespace curaengine
{

struct Acceleration : public utils::NumericFacade<double>
{
    using base_type = utils::NumericFacade<double>;
    using base_type::NumericFacade;

    constexpr Acceleration(const base_type& base) noexcept
        : base_type{ base } {};
};

} // namespace curaengine
