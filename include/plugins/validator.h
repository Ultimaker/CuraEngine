// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_PLUGINS_VALIDATOR_H
#define CURAENGINE_INCLUDE_PLUGINS_VALIDATOR_H

#include <semver.hpp>

namespace cura::plugins
{

template<size_t N>
struct VersionRangeLiteral
{
    constexpr VersionRangeLiteral(const char (&str)[N])
    {
        std::copy_n(str, N, value);
    }

    char value[N];
};


template<VersionRangeLiteral VersionRange>
struct Validator
{
    semver::version version{ "1.0.0" };
    bool include_prerelease{ false };
    semver::range::detail::range version_range{ VersionRange.value };

    constexpr operator bool() const noexcept
    {
        return version_range.satisfies(version, include_prerelease);
    }

    constexpr bool operator()(const semver::version& actual_version) const noexcept
    {
        return version_range.satisfies(actual_version, include_prerelease);
    }
};

} // namespace cura::plugins

#endif // CURAENGINE_INCLUDE_PLUGINS_VALIDATOR_H
