// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_SPLIT_PATHS_H
#define CURAENGINE_SPLIT_PATHS_H

#include <string_view>

#include <range/v3/view/split.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/view.hpp>

namespace cura::views
{
namespace details
{
#ifdef _WIN32
constexpr auto path_sep = ';';
#else
constexpr auto path_sep = ':';
#endif
} // namespace details

inline static constexpr auto split_paths = ranges::views::split(details::path_sep)
                                         | ranges::views::transform(
                                               [](auto&& rng)
                                               {
                                                   return std::string_view(&*rng.begin(), ranges::distance(rng));
                                               });

} // namespace cura::views

#endif // CURAENGINE_SPLIT_PATHS_H
