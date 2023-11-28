// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_EXCEPTION_H
#define PLUGINS_EXCEPTION_H

#include "plugins/metadata.h"
#include "plugins/types.h"
#include "plugins/validator.h"

#include <fmt/format.h>

#include <exception>
#include <string>
#include <string_view>
#include <utility>

namespace cura::plugins::exceptions
{

class ValidatorException : public std::exception
{
    std::string msg_;

public:
    ValidatorException(const auto& validator, const slot_metadata& slot_info) noexcept
        : msg_(fmt::format("Failed to validation plugin on Slot '{}'", slot_info.slot_id)){};

    ValidatorException(const auto& validator, const slot_metadata& slot_info, const plugin_metadata& plugin_info) noexcept
        : msg_(fmt::format(
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

class RemoteException : public std::exception
{
    std::string msg_;

public:
    RemoteException(const slot_metadata& slot_info, std::string_view error_msg) noexcept
        : msg_(fmt::format("Remote exception on Slot '{}': {}", slot_info.slot_id, error_msg)){};

    RemoteException(const slot_metadata& slot_info, const plugin_metadata& plugin_info, std::string_view error_msg) noexcept
        : msg_(fmt::format(
            "Remote exception for plugin '{}-{}' running at [{}] for slot '{}': {}",
            plugin_info.plugin_name,
            plugin_info.plugin_version,
            plugin_info.peer,
            slot_info.slot_id,
            error_msg))
    {
    }

    virtual const char* what() const noexcept override
    {
        return msg_.c_str();
    }
};

} // namespace cura::plugins::exceptions

#endif // PLUGINS_EXCEPTION_H