// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_VALIDATOR_H
#define PLUGINS_VALIDATOR_H

#include <fmt/format.h>
#include <semver.hpp>

#include "plugins/types.h"

namespace cura::plugins
{

/**
 * @brief A class for validating plugin versions and hashes.
 *
 * The `Validator` class provides functionality to validate plugin versions and hashes against
 * specified version ranges and plugin hashes.
 *
 * @tparam VersionRange The version range specified as a character range literal.
 * @tparam PluginHash The plugin hash specified as a character range literal.
 */
template<details::CharRangeLiteral VersionRange>
class Validator
{
    semver::range::detail::range semver_range_{ VersionRange.value }; ///< The semver range object.
    semver::version version_{ "1.0.0" }; ///< The version to validate.
    bool include_prerelease_{ true }; ///< Flag indicating whether to include prerelease versions.
    bool valid_{ false }; ///< Flag indicating the validity of the version.
    std::string what_{};

public:
    /**
     * @brief Default constructor.
     */
    constexpr Validator() noexcept = default;

    /**
     * @brief Constructor that sets the version to validate.
     *
     * @param version The version to validate.
     */
    constexpr explicit Validator(std::string_view version) noexcept
    {
        auto semver_version = semver::from_string_noexcept(version);
        if (semver_version.has_value())
        {
            version_ = semver_version.value();
            valid_ = valid_version();
        }
        else
        {
            valid_ = false;
            what_ = fmt::format("Received invalid formatted version {} from plugin, expected version according to semver.", version);
        }
    };

    /**
     * @brief Conversion operator to bool.
     *
     * @return True if the version is valid, false otherwise.
     */
    constexpr operator bool() const noexcept
    {
        return valid_;
    }

    constexpr std::string_view what() const noexcept
    {
        return what_;
    }


    /**
     * @brief Checks if the version is valid according to the specified version range.
     *
     * @return True if the version is valid, false otherwise.
     */
    [[nodiscard]] constexpr bool valid_version() noexcept
    {
        auto valid_version{ semver_range_.satisfies(version_, include_prerelease_) };
        if (! valid_version)
        {
            what_ = fmt::format("CuraEngine requires a 'slot version' within the range of '{}', while the plugin reports to have version: '{}'", VersionRange.value, version_.to_string());
        }
        return valid_version;
    }
};

} // namespace cura::plugins

#endif // PLUGINS_VALIDATOR_H
