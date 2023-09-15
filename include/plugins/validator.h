// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_VALIDATOR_H
#define PLUGINS_VALIDATOR_H

#include "plugins/metadata.h"
#include "plugins/types.h"

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <semver.hpp>

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
        auto slot_range = semver::range::detail::range(plugin_info.slot_version_range);
        auto slot_version = semver::from_string(slot_info.version);
        valid_ = slot_range.satisfies(slot_version, include_prerelease_);
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
