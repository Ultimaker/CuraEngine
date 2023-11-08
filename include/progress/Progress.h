// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PROGRESS_H
#define PROGRESS_H

#include <array>
#include <optional>
#include <string>
#include <string_view>

#include "utils/gettime.h"

namespace cura
{

struct LayerIndex;

static constexpr size_t N_PROGRESS_STAGES = 7;

/*!
 * Class for handling the progress bar and the progress logging.
 *
 * The progress bar is based on a single slicing of a rather large model which needs some complex support;
 * the relative timing of each stage is currently based on that of the slicing of dragon_65_tilted_large.stl
 */
class Progress
{
public:
    /*!
     * The stage in the whole slicing process
     */
    enum class Stage : unsigned int
    {
        START = 0,
        SLICING = 1,
        PARTS = 2,
        INSET_SKIN = 3,
        SUPPORT = 4,
        EXPORT = 5,
        FINISH = 6
    };

private:
    static constexpr std::array<double, N_PROGRESS_STAGES> times{
        0.0, // START   = 0,
        5.269, // SLICING = 1,
        1.533, // PARTS   = 2,
        71.811, // INSET_SKIN = 3
        51.009, // SUPPORT = 4,
        154.62, // EXPORT  = 5,
        0.1 // FINISH  = 6
    };

    static constexpr std::array<std::string_view, N_PROGRESS_STAGES> names{ "start", "slice", "layerparts", "inset+skin", "support", "export", "process" };
    static std::array<double, N_PROGRESS_STAGES> accumulated_times; //!< Time past before each stage
    static double total_timing; //!< An estimate of the total time
    static std::optional<LayerIndex> first_skipped_layer; //!< The index of the layer for which we skipped time reporting
    /*!
     * Give an estimate between 0 and 1 of how far the process is.
     *
     * \param stage The current stage of processing
     * \param stage_process How far we currently are in the \p stage
     * \return An estimate of the overall progress.
     */
    static double calcOverallProgress(Stage stage, double stage_progress);

public:
    static void init(); //!< Initialize some values needed in a fast computation of the progress
    /*!
     * Message progress over the CommandSocket and to the terminal (if the command line arg '-p' is provided).
     *
     * \param stage The current stage of processing
     * \param progress_in_stage Any number giving the progress within the stage
     * \param progress_in_stage_max The maximal value of \p progress_in_stage
     */
    static void messageProgress(Stage stage, int progress_in_stage, int progress_in_stage_max);

    /*!
     * Message the progress stage over the command socket.
     *
     * \param stage The current stage
     * \param timeKeeper The stapwatch keeping track of the timings for each stage (optional)
     */
    static void messageProgressStage(Stage stage, TimeKeeper* timeKeeper);

    /*!
     * Message the layer progress over the command socket and into logging output.
     *
     * \param layer_nr The processed layer number
     * \param total_layers The total number of layers to be processed
     * \param total_time The total layer processing time, in seconds
     * \param stage The detailed stages time reporting for this layer
     * \param skip_threshold The time threshold under which we consider that the full layer time reporting should be skipped
     *                       because it is not relevant
     */
    static void messageProgressLayer(LayerIndex layer_nr, size_t total_layers, double total_time, const TimeKeeper::RegisteredTimes& stages, double skip_threshold = 0.1);
};


} // namespace cura
#endif // PROGRESS_H
