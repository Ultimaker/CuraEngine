// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "GcodeTemplateResolver.h"

#include <cura-formulae-engine/parser/parser.h>

#include <boost/regex.hpp>

#include "Application.h"
#include "Slice.h"
#include "settings/SettingContainersEnvironmentAdapter.h"

namespace cfe = CuraFormulaeEngine;

namespace cura::GcodeTemplateResolver
{

std::optional<cfe::eval::Value> resolveExpression(
    std::string& output,
    std::string::const_iterator& start,
    const std::string_view& expression,
    const boost::smatch& match,
    const SettingContainersEnvironmentAdapter& environment)
{
    const zeus::expected<cfe::ast::ExprPtr, error_t> parse_result = cfe::parser::parse(expression);
    if (! parse_result.has_value())
    {
        spdlog::warn("Invalid syntax in expression {}, using raw value instead", expression);
        output += match.str();
        start = match.suffix().first;
        return std::nullopt;
    }

    const cfe::eval::Result result = parse_result.value().evaluate(&environment);
    if (! result.has_value())
    {
        spdlog::warn("Invalid variable identifier in expression {}, using raw value instead", expression);
        output += match.str();
        start = match.suffix().first;
        return std::nullopt;
    }

    return result.value();
}

std::string resolveGCodeTemplate(const std::string& input, const std::optional<int> context_extruder_nr)
{
    std::string output;

    static const boost::regex expression_regex(R"({\s*(?<condition>if|else|elif|endif)?\s*(?<expression>[^{}]*?)\s*(?:,\s*(?<extruder_nr>[^{},]*))?\s*}(?<end_of_line>\n?))");
    const SettingContainersEnvironmentAdapter global_container_env;

    std::string::const_iterator start = input.cbegin();
    const std::string::const_iterator end = input.cend();
    boost::smatch match;

    while (boost::regex_search(start, end, match, expression_regex))
    {
        std::optional<int> extruder_nr_here = context_extruder_nr;
        output += match.prefix().str(); // portion before the match

        const auto& match_expression = match["expression"];
        if (! match_expression.matched || match_expression.length() == 0)
        {
            spdlog::warn("Invalid expression {}", match.str());
            output += match.str();
            start = match.suffix().first;
            continue;
        }

        const auto& match_condition = match["condition"];
        const auto& match_extruder_nr = match["extruder_nr"];
        const auto& match_end_of_line = match["end_of_line"];

        if (match_extruder_nr.matched && match_extruder_nr.length() > 0)
        {
            const std::optional<cfe::eval::Value> extruder_nr_result = resolveExpression(output, start, match_extruder_nr.str(), match, global_container_env);
            if (! extruder_nr_result.has_value())
            {
                continue;
            }

            const cfe::eval::Value extruder_expr_value = extruder_nr_result.value();
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

        const SettingContainersEnvironmentAdapter container_env_here(extruder_nr_here);
        const std::optional<cfe::eval::Value> expression_result = resolveExpression(output, start, match_expression.str(), match, container_env_here);
        if (! expression_result.has_value())
        {
            continue;
        }

        const cfe::eval::Value& expression_value = expression_result.value();

        output += expression_value.toString();
        start = match.suffix().first;
    }

    // remaining part of the string
    output += std::string(start, end);

    return output;
}

} // namespace cura::GcodeTemplateResolver
