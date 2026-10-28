// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
namespace curaengine::plugins::details
{
template<typename T>
struct Holder
{
    using value_type = T;
    T proxy;
};
} // namespace curaengine::plugins::details
#endif // ENABLE_PLUGINS
