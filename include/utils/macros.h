// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <atomic>
#include <utility>

namespace curaengine::utils
{

template<typename T>
constexpr void unused_param(T&& param)
{
    (void)param;
}

// Pass a distinct lambda at each call site so each site has its own flag.
// No guarantees that the callable has finished when another thread returns.
template<typename Callable>
inline void run_once(Callable&& callable)
{
    static std::atomic<bool> code_ran = false;
    if (! code_ran)
    {
        bool expected = false;
        if (code_ran.compare_exchange_strong(expected, true))
        {
            std::forward<Callable>(callable)();
        }
    }
}

} // namespace curaengine::utils
