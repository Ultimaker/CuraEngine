// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GCODETEMPLATERESOLVER_H
#define GCODETEMPLATERESOLVER_H

#include <optional>
#include <string>
#include <unordered_map>
#include <zeus/expected.hpp>

#include <boost/regex.hpp>

#include "gcode_export/ResolvingExtruderContext.h"

namespace CuraFormulaeEngine
{

namespace eval
{
struct Value;
}

namespace env
{
class Environment;
class LocalEnvironment;
} // namespace env

} // namespace CuraFormulaeEngine


namespace cura
{

/*!
 *  \brief The GcodeTemplateResolver class allows setting proper variables and environments to resolve a piece of GCode
 *         that can contain dynamic formulae.
 */
class GcodeTemplateResolver
{
public:
    explicit GcodeTemplateResolver() = default;

    /*!
     * \brief Gets the calculated initial extruder number to be used when resolving the pieces of GCode
     *        having a DynamicExtruderContext::Initial context
     * \return The set initial extruder number, or nullopt if it has not been set yet
     */
    std::optional<size_t> getInitialExtruderNr() const;

    /*!
     * \brief Sets the calculated initial extruder number to be used when resolving the pieces of GCode
     *        having a DynamicExtruderContext::Initial context
     * \param initial_extruder_nr The calculated initial extruder number
     * \warning This method has to be called before calling prepareForResolving()
     */
    void setInitialExtruderNr(const size_t initial_extruder_nr);

    /*!
     * \brief Prepares for subsequent resolving by creating the proper shared environments
     * \param extra_global_settings Some extra settings to be used in common for all the resolving operation
     * \warning This method has to be called before any call to resolveGCodeTemplate()
     */
    void prepareForResolving(const std::unordered_map<std::string, CuraFormulaeEngine::eval::Value>& extra_global_settings = {});

    /*!
     * Resolve a raw GCode template that can contains conditional code and complex formulas
     * @param input The raw GCode template text
     * @param context_extruder_nr The contextual extruder number when resolving this piece of GCode
     * @param extra_settings Extra settings to be used locally for the resolving, even though they don't exist as actual settings
     * @return The fully resolved GCode template, or the raw template if an error occurred. Also, if the given input is empty,
     *         the result will also be fully empty. However, if it has some content, the function ensures that the result contains
     *         and end-of-line at the end so that it can be inserted directly in a GCode without having to insert one.
     */
    std::string resolveGCodeTemplate(
        const std::string& input,
        const ResolvingExtruderContext& context_extruder_nr = DynamicExtruderContext::Global,
        const std::unordered_map<std::string, CuraFormulaeEngine::eval::Value>& extra_settings = {}) const;

private:
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
    };

    /*!
     * Resolve the given expression
     * @param expression The raw expression text to be resolved
     * @param environment The contextual environment to be used when resolving variables
     * @return The parsed value result, or nullopt if an error occured
     */
    static std::optional<CuraFormulaeEngine::eval::Value> evaluateExpression(const std::string_view& expression, const CuraFormulaeEngine::env::Environment& environment);

    /*!
     * Processes a statement, adding it to the output or not, depending on the current conditional state
     * @param output The output string to add the statement to
     * @param condition_state The current conditional state
     * @param statement The raw statement to be added
     */
    static void processStatement(std::string& output, const GcodeConditionState condition_state, const std::string_view& statement);

    /*!
     * Processes a matched expression block
     * @param[out] output The output string to add the resolved expression to
     * @param[in,out] condition_state The current conditional state, which may be updated
     * @param match The matched regular expression containing the various parts of the expression
     * @param context_extruder_nr The default contextual extruder number, which should be the current extruder number when dealing when an extruder start/end GCode, and nullopt
     * when dealing with the global machine start/end gcode
     * @param environments The environments adapters mapped to the settings
     * @return True if the expression processing succeeded, false if an error occurred
     */
    static bool processExpression(
        std::string& output,
        GcodeConditionState& condition_state,
        const boost::smatch& match,
        const std::optional<size_t>& context_extruder_nr,
        const std::map<std::optional<size_t>, std::shared_ptr<CuraFormulaeEngine::env::LocalEnvironment>>& environments);

private:
    std::optional<size_t> initial_extruder_nr_;
    std::shared_ptr<CuraFormulaeEngine::env::LocalEnvironment> global_environment_;
    std::map<std::optional<size_t>, std::shared_ptr<CuraFormulaeEngine::env::Environment>> environment_adapters_;
};

} // namespace cura

#endif
