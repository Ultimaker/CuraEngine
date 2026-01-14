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

enum class GcodeConditionState
{
    OutsideCondition,
    ConditionFalse,
    ConditionTrue,
    ConditionDone,
};

void processRawExpression(std::string& output, std::string::const_iterator& start, const boost::smatch& match)
{
    output += match.str();
    start = match.suffix().first;
}

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
        spdlog::warn("Invalid syntax in expression [{}], using raw value instead", expression);
        processRawExpression(output, start, match);
        return std::nullopt;
    }

    const cfe::eval::Result result = parse_result.value().evaluate(&environment);
    if (! result.has_value())
    {
        spdlog::warn("Invalid variable identifier in expression [{}], using raw value instead", expression);
        processRawExpression(output, start, match);
        return std::nullopt;
    }

    return result.value();
}

void processStatement(std::string& output, const GcodeConditionState condition_state, const std::string_view& statement)
{
    if (condition_state == GcodeConditionState::OutsideCondition || condition_state == GcodeConditionState::ConditionTrue)
    {
        output += statement;
    }
}

void processExpression(
    std::string& output,
    std::string::const_iterator& start,
    GcodeConditionState& condition_state,
    const boost::smatch& match,
    const std::optional<int>& context_extruder_nr,
    const SettingContainersEnvironmentAdapter& global_container_env)
{
    std::optional<int> extruder_nr_here = context_extruder_nr;

    const auto& match_expression = match["expression"];
    const auto& match_condition = match["condition"];
    const auto& match_extruder_nr = match["extruder_nr"];
    const auto& match_end_of_line = match["end_of_line"];

    enum class GcodeInstruction
    {
        Skip,
        Evaluate,
        EvaluateAndWrite,
    };

    auto instruction = GcodeInstruction::Skip;

    if (! match_condition.matched || match_condition.length() <= 0)
    {
        // This is a classic statement
        if (condition_state == GcodeConditionState::OutsideCondition || condition_state == GcodeConditionState::ConditionTrue)
        {
            // Skip and move to next
            instruction = GcodeInstruction::EvaluateAndWrite;
        }
    }
    else
    {
        const auto condition_str = match_condition.str();

        // This is a condition statement, first check validity
        if (condition_str == "if")
        {
            if (condition_state != GcodeConditionState::OutsideCondition)
            {
                spdlog::warn("Nested conditions are not supported");
                return processRawExpression(output, start, match);
            }
        }
        else
        {
            if (condition_state == GcodeConditionState::OutsideCondition)
            {
                spdlog::warn("Condition should start with an 'if' statement");
                return processRawExpression(output, start, match);
            }
        }

        if (condition_str == "if")
        {
            // First instruction, just evaluate it
            instruction = GcodeInstruction::Evaluate;
        }
        else
        {
            if (condition_state == GcodeConditionState::ConditionTrue)
            {
                // We have reached the next condition after a valid one has been found, skip the rest
                condition_state = GcodeConditionState::ConditionDone;
            }

            if (condition_str == "elif")
            {
                if (condition_state == GcodeConditionState::ConditionFalse)
                {
                    // New instruction, and valid condition has not been reached so far => evaluate it
                    instruction = GcodeInstruction::Evaluate;
                }
                else
                {
                    // New instruction, but valid condition has already been reached => skip it
                    instruction = GcodeInstruction::Skip;
                }
            }
            else if (condition_str == "else")
            {
                instruction = GcodeInstruction::Skip; // Never evaluate, expression should be empty
                if (condition_state == GcodeConditionState::ConditionFalse)
                {
                    // Fallback instruction, and valid condition has not been reached so far => active next
                    condition_state = GcodeConditionState::ConditionTrue;
                }
            }
            else if (condition_str == "endif")
            {
                instruction = GcodeInstruction::Skip; // Never evaluate, expression should be empty
                condition_state = GcodeConditionState::OutsideCondition;
            }
        }
    }

    if (instruction == GcodeInstruction::Skip)
    {
        start = match.suffix().first;
        return;
    }

    if (match_extruder_nr.matched && match_extruder_nr.length() > 0)
    {
        const std::optional<cfe::eval::Value> extruder_nr_result = resolveExpression(output, start, match_extruder_nr.str(), match, global_container_env);
        if (! extruder_nr_result.has_value())
        {
            return;
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
            return processRawExpression(output, start, match);
        }

        if (parsed_extruder_nr >= 0 && parsed_extruder_nr < Application::getInstance().current_slice_->scene.extruders.size())
        {
            extruder_nr_here = parsed_extruder_nr;
        }
        else
        {
            spdlog::warn("Invalid replacement extruder number {}, using global settings instead", parsed_extruder_nr);
            extruder_nr_here = std::nullopt;
        }
    }

    const SettingContainersEnvironmentAdapter container_env_here(extruder_nr_here);
    const std::optional<cfe::eval::Value> expression_result = resolveExpression(output, start, match_expression.str(), match, container_env_here);
    if (! expression_result.has_value())
    {
        return processRawExpression(output, start, match);
    }

    const cfe::eval::Value& expression_value = expression_result.value();
    start = match.suffix().first;

    if (instruction == GcodeInstruction::Evaluate)
    {
        if (std::holds_alternative<bool>(expression_value.value))
        {
            bool condition_value = std::get<bool>(expression_value.value);
            condition_state = condition_value ? GcodeConditionState::ConditionTrue : GcodeConditionState::ConditionFalse;
        }
        else
        {
            spdlog::warn("Condition [{}] does not evaluate to boolean, considering true", match_expression.str());
        }

        return;
    }

    output += expression_value.toString();

    if (match_end_of_line.matched && match_end_of_line.length() > 0)
    {
        output += match_end_of_line.str();
    }
}

std::string resolveGCodeTemplate(const std::string& input, const std::optional<int> context_extruder_nr)
{
    std::string output;
    GcodeConditionState condition_state = GcodeConditionState::OutsideCondition;

    static const boost::regex expression_regex(R"({\s*(?<condition>if|else|elif|endif)?\s*(?<expression>[^{}]*?)\s*(?:,\s*(?<extruder_nr>[^{},]*))?\s*}(?<end_of_line>\n?))");
    const SettingContainersEnvironmentAdapter global_container_env;

    std::string::const_iterator start = input.cbegin();
    const std::string::const_iterator end = input.cend();
    boost::smatch match;

    while (start != end)
    {
        if (boost::regex_search(start, end, match, expression_regex))
        {
            if (match.prefix().length() > 0)
            {
                processStatement(output, condition_state, match.prefix().str());
            }

            processExpression(output, start, condition_state, match, context_extruder_nr, global_container_env);
        }
        else
        {
            // remaining part of the string
            processStatement(output, condition_state, std::string_view(&*start, end - start));
            start = end;
        }
    }

    if (! output.empty() && ! output.ends_with('\n'))
    {
        output += '\n';
    }

    return output;
}

} // namespace cura::GcodeTemplateResolver
