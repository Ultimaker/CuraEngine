// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_PLUGINS_VALIDATOR_H
#define CURAENGINE_INCLUDE_PLUGINS_VALIDATOR_H

#include "plugins/types.h"

#include <semver.hpp>

namespace cura::plugins
{


template<details::CharRangeLiteral VersionRange, details::CharRangeLiteral PluginHash>
struct Validator
{
    semver::version version{ "1.0.0" };
    std::string_view plugin_hash{};
    bool include_prerelease{ false };
    semver::range::detail::range version_range{ VersionRange.value };

    constexpr operator bool() const noexcept
    {
        // TODO: Add proper security checking
        return version_range.satisfies(version, include_prerelease) && plugin_hash == PluginHash.value;
    }
};

} // namespace cura::plugins

#endif // CURAENGINE_INCLUDE_PLUGINS_VALIDATOR_H
