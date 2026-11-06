// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <cstdlib>
#include <filesystem>

#include <fmt/core.h>

namespace fmt
{
template<>
struct formatter<std::filesystem::path> : formatter<string_view>
{
    static std::string username;
    static constexpr std::string_view obfuscated_string = "*******";

    [[nodiscard]] static std::string anonymizePath(const std::string& path)
    {
        std::string anonymized_path = path;
        size_t pos = anonymized_path.find(username);
        while (pos != std::string::npos)
        {
            anonymized_path.replace(pos, username.size(), obfuscated_string);
            pos = anonymized_path.find(username, pos + obfuscated_string.size());
        }
        return anonymized_path;
    }

    template<typename FormatContext>
    auto format(const std::filesystem::path& path, FormatContext& ctx) const
    {
        return formatter<string_view>::format(anonymizePath(path.generic_string()), ctx);
    }
};

#ifdef _WIN32
inline std::string fmt::formatter<std::filesystem::path>::username = std::getenv("USERNAME") != nullptr ? std::getenv("USERNAME") : "";
#else
inline std::string fmt::formatter<std::filesystem::path>::username = std::getenv("USER") != nullptr ? std::getenv("USER") : "";
#endif

} // namespace fmt
