// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef COMMANDLINE_H
#define COMMANDLINE_H

#include <filesystem>
#include <optional>
#include <rapidjson/document.h> //Loading JSON documents to get settings from them.
#include <string> //To store the command line arguments.
#include <unordered_map>
#include <vector> //To store the command line arguments.

#include "Communication.h" //The class we're implementing.

namespace cura
{
class Settings;

using setting_map = std::unordered_map<std::string, std::string>;
using container_setting_map = std::unordered_map<std::string, setting_map>;

/*
 * \brief When slicing via the command line, interprets the command line
 * arguments to initiate a slice.
 */
class CommandLine : public Communication
{
public:
    CommandLine() = default;

    /*
     * \brief Construct a new communicator that interprets the command line to
     * start a slice.
     * \param arguments The command line arguments passed to the application.
     */
    CommandLine(const std::vector<std::string>& arguments);

    /*
     * \brief Indicate that we're beginning to send g-code.
     * This does nothing to the command line.
     */
    void beginGCode() override;

    /*
     * \brief Flush all g-code still in the stream into cout.
     */
    void flushGCode() override;

    /*
     * \brief Indicates that for command line output we need to send the g-code
     * from start to finish.
     *
     * We can't go back and erase some g-code very easily.
     */
    bool isSequential() const override;

    /*
     * \brief Test if there are any more slices to be made.
     */
    bool hasSlice() const override;

    /*
     * \brief Send the current position.
     *
     * The command line doesn't do anything with the current position so this is
     * ignored.
     */
    void sendCurrentPosition(const Point3LL&) override;

    /*
     * \brief Indicate to the command line that we finished slicing.
     *
     * The command line doesn't do anything with that information so this is
     * ignored.
     */
    void sendFinishedSlicing() const override;

    /*
     * \brief Output the g-code header.
     */
    void sendGCodePrefix(const std::string&) const override;

    /*
     * \brief Send the uuid of the generated slice so that it may be processed by
     * the front-end.
     */
    void sendSliceUUID(const std::string& slice_uuid) const override;

    /*
     * \brief Indicate that the layer has been completely sent.
     *
     * The command line doesn't do anything with that information so this is
     * ignored.
     */
    void sendLayerComplete(const LayerIndex::value_type&, const coord_t&, const coord_t&) override;

    /*
     * \brief Send a line for display.
     *
     * The command line doesn't show any layer view so this is ignored.
     */
    void sendLineTo(const PrintFeatureType&, const Point3LL&, const coord_t&, const coord_t&, const Velocity&) override;

    /*
     * \brief Complete a layer to show it in layer view.
     *
     * The command line doesn't show any layer view so this is ignored.
     */
    void sendOptimizedLayerData() override;

    /*
     * \brief Show an estimate of how long the print would take and how much
     * material it would use.
     */
    void sendPrintTimeMaterialEstimates() const override;

    /*
     * \brief Show an update of our slicing progress.
     */
    void sendProgress(double progress) const override;

    /*
     * \brief Set which extruder is being used for the following calls to
     * ``sendPolygon``, ``sendPolygons`` and ``sendLineTo``.
     *
     * This has no effect though because we don't show these three functions
     * because the command line doesn't show layer view.
     */
    void setExtruderForSend(const ExtruderTrain&) override;

    /*
     * \brief Set which layer is being used for the following calls to
     * ``sendPolygon``, ``sendPolygons`` and ``sendLineTo``.
     *
     * This has no effect though because we don't shwo these three functions
     * because the command line doesn't show layer view.
     */
    void setLayerForSend(const LayerIndex::value_type&) override;

    /*
     * \brief Slice the next scene that the command line commands us to slice.
     */
    void sliceNext() override;

protected:
    /*
     * \brief The command line arguments that the application was called with.
     */
    std::vector<std::string> arguments_;

private:
    std::vector<std::filesystem::path> search_directories_;

    /*
     * The last progress update that we output to stdcerr.
     */
    unsigned int last_shown_progress_;

    /*
     * \brief Load a JSON file and store the settings inside it.
     * \param json_filename The location of the JSON file to load settings from.
     * \param settings The settings storage to store the settings in.
     * \param force_read_parent Also read-in values of non-leaf settings. (Off by default: Only leaf-settings should be used in the engine.)
     * \param force_read_nondefault Try to parse 'value's if 'default_value's, are not available.
     * \return Error code. If it's 0, the file was successfully loaded. If it's
     * 1, the file could not be opened. If it's 2, there was a syntax error in
     * the file.
     */
    int loadJSON(const std::filesystem::path& json_filename, Settings& settings, bool force_read_parent = false, bool force_read_nondefault = false);

    /*
     * \brief Load a JSON document and store the settings inside it.
     * \param document The JSON document to load the settings from.
     * \param settings The settings storage to store the settings in.
     * \param force_read_parent Also read-in values of non-leaf settings. (Off by default: Only leaf-settings should be used in the engine.)
     * \param force_read_nondefault Try to parse 'value's if 'default_value's, are not available.
     * \return Error code. If it's 0, the document was successfully loaded. If
     * it's 1, some inheriting file could not be opened.
     */
    int loadJSON(
        const rapidjson::Document& document,
        const std::vector<std::filesystem::path>& search_directories,
        Settings& settings,
        bool force_read_parent = false,
        bool force_read_nondefault = false);

    /*
     * \brief Load an element containing a list of settings.
     * \param element The JSON element "settings" or "overrides" that contains
     * settings.
     * \param force_read_parent Also read-in values of non-leaf settings. (Off by default: Only leaf-settings should be used in the engine.)
     * \param force_read_nondefault Try to parse 'value's if 'default_value's, are not available.
     * \param settings The settings storage to store the new settings in.
     */
    void loadJSONSettings(const rapidjson::Value& element, Settings& settings, bool force_read_parent = false, bool force_read_nondefault = false);

    /*
     * \brief Find a definition file in the search directories.
     * \param definition_id The ID of the definition to look for.
     * \param search_directories The directories to search in.
     * \return The first definition file that matches the definition ID.
     */
    static std::string findDefinitionFile(const std::string& definition_id, const std::vector<std::filesystem::path>& search_directories);

    /*
     * \brief Read the resolved JSON values from a file.
     * \param element The path to the file to read the JSON values from.
     * \return The resolved JSON values.
     */
    static std::optional<container_setting_map> readResolvedJsonValues(const std::filesystem::path& json_filename);

    /*
     * \brief Read the resolved JSON values from a document.
     * \param document The document to read the JSON values from.
     * \return The resolved JSON values.
     */
    static std::optional<container_setting_map> readResolvedJsonValues(const rapidjson::Document& document);
};

} // namespace cura

#endif // COMMANDLINE_H
