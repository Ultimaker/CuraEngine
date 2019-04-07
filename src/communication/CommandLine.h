//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef COMMANDLINE_H
#define COMMANDLINE_H

#include <rapidjson/document.h> //Loading JSON documents to get settings from them.
#include <string> //To store the command line arguments.
#include <unordered_set>
#include <vector> //To store the command line arguments.

#include "Communication.h" //The class we're implementing.

namespace cura
{
class Settings;

/*
 * \brief When slicing via the command line, interprets the command line
 * arguments to initiate a slice.
 */
class CommandLine : public Communication
{
public:
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
    void sendCurrentPosition(const Point&) override;

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
     * \brief Indicate that the layer has been completely sent.
     *
     * The command line doesn't do anything with that information so this is
     * ignored.
     */
    void sendLayerComplete(const LayerIndex&, const coord_t&, const coord_t&) override;

    /*
     * \brief Send a line for display.
     *
     * The command line doesn't show any layer view so this is ignored.
     */
    void sendLineTo(const PrintFeatureType&, const Point&, const coord_t&, const coord_t&, const Velocity&) override;

    /*
     * \brief Complete a layer to show it in layer view.
     *
     * The command line doesn't show any layer view so this is ignored.
     */
    void sendOptimizedLayerData() override;

    /*
     * \brief Send a polygon to show it in layer view.
     *
     * The command line doesn't show any layer view so this is ignored.
     */
    void sendPolygon(const PrintFeatureType&, const ConstPolygonRef&, const coord_t&, const coord_t&, const Velocity&) override;

    /*
     * \brief Send a polygon to show it in layer view.
     *
     * The command line doesn't show any layer view so this is ignored.
     */
    void sendPolygons(const PrintFeatureType&, const Polygons&, const coord_t&, const coord_t&, const Velocity&) override;

    /*
     * \brief Show an estimate of how long the print would take and how much
     * material it would use.
     */
    void sendPrintTimeMaterialEstimates() const override;

    /*
     * \brief Show an update of our slicing progress.
     */
    void sendProgress(const float& progress) const override;

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
    void setLayerForSend(const LayerIndex&) override;

    /*
     * \brief Slice the next scene that the command line commands us to slice.
     */
    void sliceNext() override;

private:
    /*
     * \brief The command line arguments that the application was called with.
     */
    std::vector<std::string> arguments;

    /*
     * The last progress update that we output to stdcerr.
     */
    unsigned int last_shown_progress;

    /*
     * \brief Get the default search directories to search for definition files.
     * \return The default search directories to search for definition files.
     */
    std::unordered_set<std::string> defaultSearchDirectories();

    /*
     * \brief Load a JSON file and store the settings inside it.
     * \param json_filename The location of the JSON file to load settings from.
     * \param settings The settings storage to store the settings in.
     * \return Error code. If it's 0, the file was successfully loaded. If it's
     * 1, the file could not be opened. If it's 2, there was a syntax error in
     * the file.
     */
    int loadJSON(const std::string& json_filename, Settings& settings);

    /*
     * \brief Load a JSON document and store the settings inside it.
     * \param document The JSON document to load the settings from.
     * \param settings The settings storage to store the settings in.
     * \return Error code. If it's 0, the document was successfully loaded. If
     * it's 1, some inheriting file could not be opened.
     */
    int loadJSON(const rapidjson::Document& document, const std::unordered_set<std::string>& search_directories, Settings& settings);

    /*
     * \brief Load an element containing a list of settings.
     * \param element The JSON element "settings" or "overrides" that contains
     * settings.
     * \param settings The settings storage to store the new settings in.
     */
    void loadJSONSettings(const rapidjson::Value& element, Settings& settings);

    /*
     * \brief Find a definition file in the search directories.
     * \param definition_id The ID of the definition to look for.
     * \param search_directories The directories to search in.
     * \return The first definition file that matches the definition ID.
     */
    const std::string findDefinitionFile(const std::string& definition_id, const std::unordered_set<std::string>& search_directories);
};

} //namespace cura

#endif //COMMANDLINE_H