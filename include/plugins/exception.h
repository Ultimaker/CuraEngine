// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_CONCEPTS_GRAPH_H
#define UTILS_CONCEPTS_GRAPH_H

#include <exception>

#include <fmt/format.h>
#include <string>
#include <utility>

#include "plugins/types.h"

namespace cura::plugins::exceptions
{

class ValidatorException : public std::exception
{
    std::string msg_;

public:
    ValidatorException(auto validator, std::string plugin_name, const std::string& plugin_version, const std::string& plugin_target) noexcept
        : msg_(fmt::format("Plugin {} '{}' at {} failed validation: {}", plugin_name, plugin_version, plugin_target, validator.what()))
    {
    }

    virtual const char* what() const noexcept override
    {
        return msg_.c_str();
    }
};

} // namespace cura::plugins::exceptions

#endif // UTILS_CONCEPTS_GRAPH_H