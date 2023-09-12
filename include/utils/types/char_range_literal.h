// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_TYPES_CHAR_RANGE_LITERAL_H
#define UTILS_TYPES_CHAR_RANGE_LITERAL_H

#include <algorithm>


namespace cura::utils
{
template<size_t N>
struct CharRangeLiteral
{
    constexpr CharRangeLiteral(const char (&str)[N]) noexcept
    {
        std::copy_n(str, N, value);
    }

    char value[N];
};

} // namespace cura::utils

#endif // UTILS_TYPES_CHAR_RANGE_LITERAL_H