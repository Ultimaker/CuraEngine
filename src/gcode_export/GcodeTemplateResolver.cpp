// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "gcode_export/GcodeTemplateResolver.h"

#include <cura-formulae-engine/parser/parser.h>

#include <range/v3/algorithm/contains.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/map.hpp>

#include "Application.h"
#include "Slice.h"
#include "gcode_export/FunctionsEnvironment.h"
#include "settings/SettingContainersEnvironmentAdapter.h"

namespace cfe = CuraFormulaeEngine;

namespace cura
{


void GcodeTemplateResolver::prepareForResolving(const size_t initial_extruder_nr, const std::unordered_map<std::string, cfe::eval::Value>& extra_global_settings)
{
    initial_extruder_nr_ = initial_extruder_nr;

    // Create an environment containing the callable functions implementations
    functions_environment_ = std::make_shared<FunctionsEnvironment>(&CuraFormulaeEngine::env::std_env);

    // Create an environment containing all the extra global settings
    global_environment_ = std::make_shared<cfe::env::LocalEnvironment>(functions_environment_.get());
    global_environment_->add(extra_global_settings);
    global_environment_->set("initial_extruder_nr", static_cast<int64_t>(initial_extruder_nr_));

    // Now create the global and extruder environment, each pointing towards the extra environment
    const Scene& scene = Application::getInstance().current_slice_->scene;
    environment_adapters_[std::nullopt] = std::make_shared<SettingContainersEnvironmentAdapter>(scene.settings, global_environment_.get());
    for (auto [extruder_nr, extruder_train] : scene.extruders | ranges::views::enumerate)
    {
        auto extruder_environment_adapter = std::make_shared<SettingContainersEnvironmentAdapter>(extruder_train.settings_, global_environment_.get());
        environment_adapters_[extruder_nr] = extruder_environment_adapter;
        functions_environment_->addExtruderAdapter(extruder_nr, extruder_environment_adapter);
    }
}

std::optional<CuraFormulaeEngine::eval::Value> GcodeTemplateResolver::evaluateExpression(const std::string_view& expression, const cfe::env::Environment& environment)
{
    const zeus::expected<cfe::ast::ExprPtr, cfe::parser::error_t> parse_result = cfe::parser::parse(expression);
    if (! parse_result.has_value())
    {
        spdlog::error("Invalid syntax in expression [{}]", expression);
        return std::nullopt;
    }

    const cfe::eval::Result result = parse_result.value().evaluate(&environment);
    if (! result.has_value())
    {
        spdlog::warn("Invalid variable identifier in expression [{}]", expression);
        return std::nullopt;
    }

    return result.value();
}

void GcodeTemplateResolver::processStatement(std::string& output, const GcodeConditionState condition_state, const std::string_view& statement)
{
    if (condition_state == GcodeConditionState::OutsideCondition || condition_state == GcodeConditionState::ConditionTrue)
    {
        output += statement;
    }
}

bool GcodeTemplateResolver::processExpression(
    std::string& output,
    GcodeConditionState& condition_state,
    const boost::smatch& match,
    const std::optional<size_t>& context_extruder_nr,
    const std::map<std::optional<size_t>, std::shared_ptr<cfe::env::LocalEnvironment>>& environments)
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
        std::optional<CuraFormulaeEngine::eval::Value> extruder_nr_result = evaluateExpression(match_extruder_nr.str(), *environments.at(context_extruder_nr));
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
    std::optional<CuraFormulaeEngine::eval::Value> expression_result = evaluateExpression(match_expression.str(), *environment_iterator->second);
    if (! expression_result.has_value())
    {
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

std::string GcodeTemplateResolver::resolveGCodeTemplate(
    const std::string& input,
    const ResolvingExtruderContext& context_extruder_nr,
    const std::unordered_map<std::string, cfe::eval::Value>& extra_settings) const
{
    std::string output;
    GcodeConditionState condition_state = GcodeConditionState::OutsideCondition;

    static const boost::regex expression_regex(R"({\s*(?<condition>if|else|elif|endif)?\s*(?<expression>[^{}]*?)\s*(?:,\s*(?<extruder_nr>[^{},]*))?\s*}(?<end_of_line>\n?))");

    // Create local environment containing the context-specific extra settings, and are chained to the extruder and global environment
    std::map<std::optional<size_t>, std::shared_ptr<cfe::env::LocalEnvironment>> local_environments;
    for (auto iterator = environment_adapters_.begin(); iterator != environment_adapters_.end(); ++iterator)
    {
        auto local_environment = std::make_shared<cfe::env::LocalEnvironment>(iterator->second.get());
        local_environment->add(extra_settings);
        local_environments.emplace(iterator->first, local_environment);
    }

    std::optional<size_t> actual_extruder_nr;
    std::visit(
        [this, &actual_extruder_nr](auto&& context_extruder_nr_value)
        {
            using T = std::decay_t<decltype(context_extruder_nr_value)>;
            if constexpr (std::is_same_v<T, DynamicExtruderContext>)
            {
                if (context_extruder_nr_value == DynamicExtruderContext::Initial)
                {
                    actual_extruder_nr = initial_extruder_nr_;
                }
                // Otherwise actual_extruder_nr stays nullopt, which means use global context
            }
            else if constexpr (std::is_same_v<T, size_t>)
            {
                actual_extruder_nr = context_extruder_nr_value;
            }
        },
        context_extruder_nr);

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

            if (! processExpression(output, condition_state, match, actual_extruder_nr, local_environments))
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

} // namespace cura
