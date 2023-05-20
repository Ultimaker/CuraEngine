// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_VALIDATOR_H
#define PLUGINS_VALIDATOR_H

#include "plugins/types.h"

#include <semver.hpp>

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
    bool include_prerelease_{ false }; ///< Flag indicating whether to include prerelease versions.
    bool valid_{ false }; ///< Flag indicating the validity of the version.

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
    constexpr explicit Validator(std::string_view version) : version_{ version }, valid_{ valid_version() } {};

    /**
     * @brief Conversion operator to bool.
     *
     * @return True if the version is valid, false otherwise.
     */
    constexpr operator bool() const noexcept
    {
        return valid_;
    }

    /**
     * @brief Checks if the version is valid according to the specified version range.
     *
     * @return True if the version is valid, false otherwise.
     */
    [[nodiscard]] constexpr bool valid_version() const
    {
        return semver_range_.satisfies(version_, include_prerelease_);
    }

    /**
     * @brief Returns the version string.
     *
     * @return The version string.
     */
    std::string getVersion() const
    {
        return version_.to_string();
    }

    /**
     * @brief The version range specified as a string view.
     */
    static inline constexpr std::string_view version_range{ VersionRange.value };
};

} // namespace cura::plugins

#endif // PLUGINS_VALIDATOR_H
