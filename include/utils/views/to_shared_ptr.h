// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_UTILS_VIEWS_TO_SHARED_PTR_H
#define CURAENGINE_INCLUDE_UTILS_VIEWS_TO_SHARED_PTR_H


#include <memory>

#include <range/v3/view/transform.hpp>

namespace cura::views
{

constexpr auto to_shared_ptr = ranges::views::transform([](auto item) { return std::make_shared<decltype(item)>(item); });

} // namespace cura::views


#endif // CURAENGINE_INCLUDE_UTILS_VIEWS_TO_SHARED_PTR_H
