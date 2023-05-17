// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_VALIDATOR_H
#define PLUGINS_VALIDATOR_H

#include "plugins/types.h"

#include <semver.hpp>

namespace cura::plugins
{

// TODO: Implement hash and other checks
template<details::CharRangeLiteral VersionRange, details::CharRangeLiteral PluginHash>
class Validator
{
public:
    constexpr Validator() noexcept = default;
    constexpr explicit Validator(std::string_view version) : version_{ version }, valid_{ version_range_.satisfies(version_, include_prerelease_) } {};

    constexpr operator bool() const noexcept
    {
        return valid_;
    }

private:
    semver::version version_{ "1.0.0" };
    std::string_view plugin_hash_{};
    bool include_prerelease_{ false };
    semver::range::detail::range version_range_{ VersionRange.value };
    bool valid_{ false };
};

} // namespace cura::plugins

#endif // PLUGINS_VALIDATOR_H
