// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <exception>
#include <string>

#include <fmt/format.h>

#include "plugins/Validator.h"
#include "plugins/plugin_metadata.h"
#include "plugins/slot_metadata.h"

namespace cura::plugins::exceptions
{
class ValidatorException : public std::exception
{
    std::string msg_;

public:
    ValidatorException(const auto& validator, const slot_metadata& slot_info) noexcept
        : msg_(fmt::format("Failed to validation plugin on Slot '{}'", slot_info.slot_id)) {};

    ValidatorException(const auto& validator, const slot_metadata& slot_info, const plugin_metadata& plugin_info) noexcept
        : msg_(
              fmt::format(
                  "Failed to validate plugin '{}-{}' running at [{}] for slot '{}', version '{}' incompatible with plugin specified slot-version-range '{}'.",
                  plugin_info.plugin_name,
                  plugin_info.plugin_version,
                  plugin_info.peer,
                  slot_info.slot_id,
                  slot_info.version,
                  plugin_info.slot_version_range))
    {
    }

    virtual const char* what() const noexcept override
    {
        return msg_.c_str();
    }
};
} // namespace cura::plugins::exceptions
