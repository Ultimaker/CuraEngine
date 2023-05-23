// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_CONCEPTS_GRAPH_H
#define UTILS_CONCEPTS_GRAPH_H

#include <exception>

#include <fmt/format.h>
#include <string>
#include <utility>

#include "plugins/metadata.h"
#include "plugins/types.h"

namespace cura::plugins::exceptions
{

class ValidatorException : public std::exception
{
    std::string msg_;

public:
    ValidatorException(const auto& validator, const slot_metadata& slot_info) noexcept : msg_(fmt::format("Failed to validation plugin on Slot '{}'", slot_info.slot_id)){};

    ValidatorException(const auto& validator, const slot_metadata& slot_info, const plugin_metadata& plugin_info) noexcept
        : msg_(fmt::format("Failed to validate plugin {} '{}' at {} for slot '{}'", plugin_info.name, plugin_info.version, plugin_info.peer, slot_info.slot_id))
    {
    }

    virtual const char* what() const noexcept override
    {
        return msg_.c_str();
    }
};

} // namespace cura::plugins::exceptions

#endif // UTILS_CONCEPTS_GRAPH_H