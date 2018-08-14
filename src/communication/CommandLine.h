//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef COMMANDLINE_H
#define COMMANDLINE_H

#include <string> //To store the command line arguments.
#include <vector> //To store the command line arguments.

#include "Communication.h" //The class we're implementing.

namespace cura
{

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
    const bool isSequential() const override;

    /*
     * \brief Test if there are any more slices to be made.
     */
    const bool hasSlice() const override;

    /*
     * \brief Send the current position.
     *
     * The command line doesn't do anything with the current position so this is
     * ignored.
     */
    void sendCurrentPosition(const Point& position) override;

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
    void sendGCodePrefix(const std::string& prefix) const override;

    /*
     * \brief Indicate that the layer has been completely sent.
     *
     * The command line doesn't do anything with that information so this is
     * ignored.
     */
    void sendLayerComplete(const LayerIndex& layer_nr, const coord_t& z, const coord_t& thickness) override;

    /*
     * \brief Send a line for display.
     *
     * The command line doesn't show any layer view so this is ignored.
     */
    void sendLineTo(const PrintFeatureType& type, const Point& to, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity) override;

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
    void sendPolygon(const PrintFeatureType& type, const ConstPolygonRef& polygon, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity) override;

    /*
     * \brief Send a polygon to show it in layer view.
     *
     * The command line doesn't show any layer view so this is ignored.
     */
    void sendPolygons(const PrintFeatureType& type, const Polygons& polygons, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity) override;

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
    void setExtruderForSend(const ExtruderTrain& extruder) override;

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
};

} //namespace cura

#endif //COMMANDLINE_H