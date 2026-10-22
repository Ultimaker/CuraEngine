// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include "plugins/Registry.h"

namespace cura::plugins::details
{
template<typename TList, template<typename> class Unit>
class SingletonRegistry
{
public:
    static Registry<TList, Unit>& instance()
    {
        static Registry<TList, Unit> instance;
        return instance;
    }

private:
    constexpr SingletonRegistry() = default;
};
} // namespace cura::plugins::details
#endif // ENABLE_PLUGINS
