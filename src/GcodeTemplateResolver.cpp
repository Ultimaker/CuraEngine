// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "GcodeTemplateResolver.h"

#include <cura-formulae-engine/parser/parser.h>
#include <zeus/expected.hpp>

#include <boost/regex.hpp>
#include <range/v3/algorithm/contains.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/map.hpp>

#include "Application.h"
#include "Slice.h"
#include "settings/SettingContainersEnvironmentAdapter.h"

namespace cfe = CuraFormulaeEngine;

namespace cura::GcodeTemplateResolver
{
static constexpr std::array post_slice_data_variables = { "filament_cost", "print_time", "filament_amount", "filament_weight", "jobname" };

/*! State-machine enum to track the conditional blocks states */
enum class GcodeConditionState
{
    OutsideCondition, // Normal statement, not currently inside a conditional block
    ConditionFalse, // Inside a conditional block where the current condition is false
    ConditionTrue, // Inside a conditional block where the current condition is true
    ConditionDone, // Inside a multi-conditional block where the valid condition has already been processed
};

enum class EvaluateResult
{
    Error, // An error has occured when parsing/evaluating the expression
    Skip, // The expression should not be evaluated and kept as is
};

/*!
 * Resolve the given expression
 * @param expression The raw expression text to be resolved
 * @param environment The contextual environment to be used when resolving variables
 * @return The parsed value result, or an error description
 */
zeus::expected<cfe::eval::Value, EvaluateResult> evaluateExpression(const std::string_view& expression, const cfe::env::Environment& environment)
{
    const zeus::expected<cfe::ast::ExprPtr, cfe::parser::error_t> parse_result = cfe::parser::parse(expression);
    if (! parse_result.has_value())
    {
        spdlog::error("Invalid syntax in expression [{}]", expression);
        return zeus::unexpected(EvaluateResult::Error);
    }

    const cfe::eval::Result result = parse_result.value().evaluate(&environment);
    if (! result.has_value())
    {
        if (ranges::contains(post_slice_data_variables, expression))
        {
            return zeus::unexpected(EvaluateResult::Skip);
        }

        spdlog::warn("Invalid variable identifier in expression [{}]", expression);
        return zeus::unexpected(EvaluateResult::Error);
    }

    return result.value();
}

/*!
 * Processes a statement, adding it to the output or not, depending on the current conditional state
 * @param output The output string to add the statement to
 * @param condition_state The current conditional state
 * @param statement The raw statement to be added
 */
void processStatement(std::string& output, const GcodeConditionState condition_state, const std::string_view& statement)
{
    if (condition_state == GcodeConditionState::OutsideCondition || condition_state == GcodeConditionState::ConditionTrue)
    {
        output += statement;
    }
}

/*!
 * Processes a matched expression block
 * @param[out] output The output string to add the resolved expression to
 * @param[in,out] condition_state The current conditional state, which may be updated
 * @param match The matched regular expression containing the various parts of the expression
 * @param context_extruder_nr The default contextual extruder number, which should be the current extruder number when dealing when an extruder start/end GCode, and nullopt when
 *                            dealing with the global machine start/end gcode
 * @param environments The environments adapters mapped to the settings
 * @param extra_settings Extra settings to be locally added to the environment
 * @return True if the expression processing succeeded, false if an error occurred
 */
bool processExpression(
    std::string& output,
    GcodeConditionState& condition_state,
    const boost::smatch& match,
    const std::optional<size_t>& context_extruder_nr,
    const std::map<std::optional<size_t>, cfe::env::LocalEnvironment>& environments)
{
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
                spdlog::error("Nested conditions are not supported");
                return false;
            }
        }
        else
        {
            if (condition_state == GcodeConditionState::OutsideCondition)
            {
                spdlog::error("Condition should start with an 'if' statement");
                return false;
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
        return true;
    }

    std::optional<size_t> extruder_nr_here = context_extruder_nr;

    if (match_extruder_nr.matched && match_extruder_nr.length() > 0)
    {
        const zeus::expected<cfe::eval::Value, EvaluateResult> extruder_nr_result = evaluateExpression(match_extruder_nr.str(), environments.at(std::nullopt));
        if (! extruder_nr_result.has_value())
        {
            return false;
        }

        const cfe::eval::Value extruder_expr_value = extruder_nr_result.value();
        size_t parsed_extruder_nr;
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
            return false;
        }

        extruder_nr_here = parsed_extruder_nr;
    }

    auto environment_iterator = environments.find(extruder_nr_here);
    if (environment_iterator == environments.end())
    {
        spdlog::warn("Invalid extruder number {}, using global settings instead", extruder_nr_here.value_or(-1));
        extruder_nr_here = std::nullopt;
        environment_iterator = environments.find(std::nullopt);
    }
    const zeus::expected<cfe::eval::Value, EvaluateResult> expression_result = evaluateExpression(match_expression.str(), environment_iterator->second);
    if (! expression_result.has_value())
    {
        if (expression_result.error() == EvaluateResult::Skip)
        {
            output += match.str();
            return true;
        }
        return false;
    }

    const cfe::eval::Value& expression_value = expression_result.value();
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

        return true;
    }

    output += expression_value.toString();

    if (match_end_of_line.matched && match_end_of_line.length() > 0)
    {
        output += match_end_of_line.str();
    }

    return true;
}

std::string resolveGCodeTemplate(const std::string& input, const std::optional<int> context_extruder_nr, const std::unordered_map<std::string, std::string>& extra_settings)
{
    std::string output;
    GcodeConditionState condition_state = GcodeConditionState::OutsideCondition;

    static const boost::regex expression_regex(R"({\s*(?<condition>if|else|elif|endif)?\s*(?<expression>[^{}]*?)\s*(?:,\s*(?<extruder_nr>[^{},]*))?\s*}(?<end_of_line>\n?))");
    const Scene& scene = Application::getInstance().current_slice_->scene;

    std::map<std::optional<size_t>, SettingContainersEnvironmentAdapter> environment_adapters;
    environment_adapters.emplace(std::nullopt, SettingContainersEnvironmentAdapter(scene.settings));
    for (auto [extruder_nr, extruder_train] : scene.extruders | ranges::views::enumerate)
    {
        environment_adapters.emplace(extruder_nr, SettingContainersEnvironmentAdapter(extruder_train.settings_));
    }

    std::map<std::optional<size_t>, cfe::env::LocalEnvironment> environments;
    for (const std::optional<size_t> environment_key : environment_adapters | ranges::views::keys) // Do not access by iterator because it creates a copy of the adapter
    {
        environments.emplace(environment_key, cfe::env::LocalEnvironment(&environment_adapters.at(environment_key)));
        cfe::env::LocalEnvironment& local_environment = environments.at(environment_key);
        for (auto iterator = extra_settings.begin(); iterator != extra_settings.end(); ++iterator)
        {
            local_environment.set(iterator->first, iterator->second);
        }
    }

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

            if (! processExpression(output, condition_state, match, context_extruder_nr, environments))
            {
                // Something got wrong during expression evaluation, return raw input insteaad
                output = input;
                break;
            }

            start = match.suffix().first;
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
