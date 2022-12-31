// Copyright (c) 2022 UltiMaker
// CuraEngine is release under the terms of the AGPLv3 or higher

#ifndef UTILS_CONCEPTS_STRING_H
#define UTILS_CONCEPTS_STRING_H

#include <concepts>
#include <string>
#include <type_traits>

namespace cura
{

template<class T>
concept isString = std::is_convertible_v<T, std::string>;


}

#endif // UTILS_CONCEPTS_STRING_H
