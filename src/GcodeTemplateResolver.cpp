// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "GcodeTemplateResolver.h"

#include <cura-formulae-engine/parser/parser.h>
#include <regex>

#include "Application.h"
#include "Slice.h"
#include "settings/SettingContainersEnvironmentAdapter.h"

namespace cfe = CuraFormulaeEngine;

namespace cura::GcodeTemplateResolver
{

std::string resolveGCodeTemplate(const std::string& input, const std::optional<int> context_extruder_nr)
{
    std::string output;

    const std::regex template_string_regex(R"(\{([^\}]*)\})");
    const std::regex expr_extruder_expr_regex(R"(^\s*(.+?)\s*,\s*(\d+?)\s*$)");
    const SettingContainersEnvironmentAdapter global_container_env;

    std::string::const_iterator start = input.begin();
    const std::string::const_iterator end = input.end();
    std::smatch match;

    while (std::regex_search(start, end, match, template_string_regex))
    {
        std::optional<int> extruder_nr_here = context_extruder_nr;
        output += match.prefix().str(); // portion before the match

        std::regex::string_type value_expr_str = match[1].str();

        // if the `expr_extruder_expr_regex` matches the template string is in the format
        // ```
        // { [expr: value_expr], [digit: extruder_expr] }
        // ```
        // then the first expr is the expr that result in the resulting value, the extruder_expr
        // evaluates to the extruder from which settings are resolved.
        const std::regex::string_type value_expr_str_ = value_expr_str;
        const std::string::const_iterator start_ = value_expr_str_.begin();
        const std::string::const_iterator end_ = value_expr_str_.end();
        std::smatch match_;
        if (std::regex_search(start_, end_, match_, expr_extruder_expr_regex))
        {
            value_expr_str = match_[1].str();

            const std::string extruder_expr_str = match_[2].str();
            const zeus::expected<cfe::ast::ExprPtr, error_t> extruder_expr_result = cfe::parser::parse(extruder_expr_str);
            if (! extruder_expr_result.has_value())
            {
                output += match.str();
                start = match.suffix().first;
                continue;
            }

            const cfe::ast::ExprPtr& extruder_expr = extruder_expr_result.value();
            const cfe::eval::Result extruder_expr_value_result = extruder_expr.evaluate(&global_container_env);
            if (! extruder_expr_value_result.has_value())
            {
                output += match.str();
                start = match.suffix().first;
                continue;
            }

            const cfe::eval::Value extruder_expr_value = extruder_expr_value_result.value();
            int parsed_extruder_nr;
            if (std::holds_alternative<std::int64_t>(extruder_expr_value.value))
            {
                parsed_extruder_nr = std::get<std::int64_t>(extruder_expr_value.value);
            }
            else if (std::holds_alternative<std::string>(extruder_expr_value.value))
            {
                parsed_extruder_nr = std::stoi(std::get<std::string>(extruder_expr_value.value));
            }
            else if (std::holds_alternative<double>(extruder_expr_value.value))
            {
                parsed_extruder_nr = std::floor(std::get<double>(extruder_expr_value.value));
            }
            else
            {
                output += match.str();
                start = match.suffix().first;
                continue;
            }

            if (parsed_extruder_nr >= 0 && parsed_extruder_nr < Application::getInstance().current_slice_->scene.extruders.size())
            {
                extruder_nr_here = parsed_extruder_nr;
            }
            else
            {
                spdlog::warn("Invalid replacement extruder number {}, using contextual extruder instead", parsed_extruder_nr);
            }
        }

        const zeus::expected<cfe::ast::ExprPtr, error_t> value_expr_result = cfe::parser::parse(value_expr_str);
        if (! value_expr_result.has_value())
        {
            output += match.str();
            start = match.suffix().first;
            continue;
        }

        const cfe::ast::ExprPtr& value_expr = value_expr_result.value();

        const SettingContainersEnvironmentAdapter container_env_here(extruder_nr_here);
        const cfe::eval::Result value_expr_value_result = value_expr.evaluate(&container_env_here);

        if (! value_expr_value_result.has_value())
        {
            output += match.str();
            start = match.suffix().first;
            continue;
        }

        const cfe::eval::Value& value_expr_value = value_expr_value_result.value();

        output += value_expr_value.toString();
        start = match.suffix().first;
    }

    // remaining part of the string
    output += std::string(start, end);

    return output;
}

} // namespace cura::GcodeTemplateResolver
