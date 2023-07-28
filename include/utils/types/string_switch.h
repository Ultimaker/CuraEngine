// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_TYPES_STRING_SWITCH_H
#define UTILS_TYPES_STRING_SWITCH_H

#include <string_view>

namespace cura::utils
{

// Source: https://learnmoderncpp.com/2020/06/01/strings-as-switch-case-labels/
constexpr inline uint64_t hash_djb2a(const std::string_view value)
{
    uint64_t hash{ 5381 };
    for (unsigned char c : value)
    {
        hash = ((hash << 5) + hash) ^ c;
    }
    return hash;
}

constexpr inline uint64_t hash_enum(const std::string_view value)
{
    constexpr uint64_t plugin_namespace_sep_location{ 6 };
    if (value.size() > plugin_namespace_sep_location && value.at(plugin_namespace_sep_location) == ':')
    {
        return hash_djb2a("plugin");
    }
    return hash_djb2a(value);
}

constexpr inline auto operator""_sw(const char* str, size_t len)
{
    return hash_enum(std::string_view{ str, len });
}


} // namespace cura::utils

#endif // UTILS_TYPES_STRING_SWITCH_H
