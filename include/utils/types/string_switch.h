// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <string_view>

namespace curaengine::utils
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


} // namespace curaengine::utils
