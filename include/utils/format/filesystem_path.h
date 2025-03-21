// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_UTILS_FORMAT_FILESYSTEM_PATH_H
#define CURAENGINE_INCLUDE_UTILS_FORMAT_FILESYSTEM_PATH_H

#include <cstdlib>
#include <filesystem>

#include <fmt/core.h>

namespace fmt
{
template<>
struct formatter<std::filesystem::path> : formatter<string_view>
{
    static std::string USERNAME;
    static constexpr std::string_view OBFUSCATED_STRING = "*******";

    [[nodiscard]] static std::string anonymizePath(const std::string& path)
    {
        std::string anonymized_path = path;
        size_t pos = anonymized_path.find(USERNAME);
        while (pos != std::string::npos)
        {
            anonymized_path.replace(pos, USERNAME.size(), OBFUSCATED_STRING);
            pos = anonymized_path.find(USERNAME, pos + OBFUSCATED_STRING.size());
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
inline std::string fmt::formatter<std::filesystem::path>::USERNAME = std::getenv("USERNAME") != nullptr ? std::getenv("USERNAME") : "";
#else
inline std::string fmt::formatter<std::filesystem::path>::USERNAME = std::getenv("USER") != nullptr ? std::getenv("USER") : "";
#endif

} // namespace fmt

#endif // CURAENGINE_INCLUDE_UTILS_FORMAT_FILESYSTEM_PATH_H
