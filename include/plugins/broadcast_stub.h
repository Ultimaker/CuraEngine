// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <utility>

#include "plugins/empty.h"

namespace cura::plugins::details
{
template<class T, class C = empty>
struct broadcast_stub
{
    using derived_type = T;
    friend derived_type;

    constexpr auto operator()(auto&&... args)
    {
        return request_(std::forward<decltype(args)>(args)...);
    }

private:
    C request_{};
};
} // namespace cura::plugins::details
