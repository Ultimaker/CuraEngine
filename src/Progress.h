/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef PROGRESS_H
#define PROGRESS_H

#include <string>

#include "utils/logoutput.h"
#include "utils/gettime.h"

namespace cura {

class CommandSocket;

#define N_PROGRESS_STAGES 8

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
        START   = 0, 
        SLICING = 1, 
        PARTS   = 2, 
        INSET   = 3, 
        SUPPORT = 4, 
        SKIN    = 5, 
        EXPORT  = 6, 
        FINISH  = 7
    };
private:
    static double times [N_PROGRESS_STAGES]; //!< Time estimates per stage
    static std::string names[N_PROGRESS_STAGES]; //!< name of each stage
    static double accumulated_times [N_PROGRESS_STAGES]; //!< Time past before each stage
    static const Stage stages[]; //!< an ordered array of the stages
    static double totalTiming; //!< An estimate of the total time
    /*!
     * Give an estimate between 0 and 1 of how far the process is.
     * 
     * \param stage The current stage of processing
     * \param stage_process How far we currently are in the \p stage
     * \return An estimate of the overall progress.
     */
    static float calcOverallProgress(Stage stage, float stage_progress);
public:
    static void init(); //!< Initialize some values needed in a fast computation of the progress
    /*!
     * Message progress over the \p commandSocket and to the terminal (if the command line arg '-p' is provided).
     * 
     * \param stage The current stage of processing
     * \param progress_in_stage Any number giving the progress within the stage
     * \param progress_in_stage_max The maximal value of \p progress_in_stage
     * \param commandSocket The command socket over which to communicate the progress.
     */
    static void messageProgress(Stage stage, int progress_in_stage, int progress_in_stage_max, CommandSocket* commandSocket);
    /*!
     * Message the progress stage over the command socket.
     * 
     * \param stage The current stage
     * \param timeKeeper The stapwatch keeping track of the timings for each stage (optional)
     * \param commandSocket The command socket over which to communicate (optional)
     */
    static void messageProgressStage(Stage stage, TimeKeeper* timeKeeper, CommandSocket* commandSocket);
};


} // name space cura
#endif//PROGRESS_H
