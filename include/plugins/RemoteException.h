// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <exception>
#include <string>
#include <string_view>

#include <fmt/format.h>

#include "plugins/plugin_metadata.h"
#include "plugins/slot_metadata.h"
#include "plugins/types.h"

namespace cura::plugins::exceptions
{
class RemoteException : public std::exception
{
    std::string msg_;

public:
    RemoteException(const slot_metadata& slot_info, std::string_view error_msg) noexcept
        : msg_(fmt::format("Remote exception on Slot '{}': {}", slot_info.slot_id, error_msg)) {};

    RemoteException(const slot_metadata& slot_info, const plugin_metadata& plugin_info, std::string_view error_msg) noexcept
        : msg_(
              fmt::format(
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
