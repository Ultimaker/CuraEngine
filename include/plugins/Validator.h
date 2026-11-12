// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <semver.hpp>

#include "plugins/plugin_metadata.h"
#include "plugins/slot_metadata.h"

namespace curaengine::plugins
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

} // namespace curaengine::plugins
