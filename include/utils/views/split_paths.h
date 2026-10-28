// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <string_view>

#include <range/v3/view/split.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/view.hpp>

namespace curaengine::utils::views
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

} // namespace curaengine::utils::views
