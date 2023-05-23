// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_VALIDATOR_H
#define PLUGINS_VALIDATOR_H

#include <fmt/format.h>
#include <semver.hpp>

#include "plugins/metadata.h"
#include "plugins/types.h"

namespace cura::plugins
{

class Validator
{
    bool include_prerelease_{ true }; ///< Flag indicating whether to include prerelease versions.
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
    Validator(const slot_metadata& slot_info, const plugin_metadata& plugin_info)
    {
        auto slot_range = semver::range::detail::range(slot_info.version_range);
        auto slot_version = semver::from_string(plugin_info.slot_version);
        if (slot_range.satisfies(slot_version, include_prerelease_))
        {
            valid_ = true;
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
};

} // namespace cura::plugins

#endif // PLUGINS_VALIDATOR_H