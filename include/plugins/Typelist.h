// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
namespace cura::plugins::details
{
template<typename... Types>
struct Typelist
{
};
} // namespace cura::plugins::details
#endif // ENABLE_PLUGINS
