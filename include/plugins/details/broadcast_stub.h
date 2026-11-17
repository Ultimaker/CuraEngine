// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <utility>

#include "plugins/empty.h"

namespace curaengine::plugins::details
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
} // namespace curaengine::plugins::details
