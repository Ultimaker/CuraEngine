/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef PROGRESS_H
#define PROGRESS_H

#include "commandSocket.h"
#include "utils/logoutput.h"


namespace cura {
    
#define N_PROGRESS_STAGES 8


class Progress 
{
public:
    enum class Stage 
    {
        START   = 0, 
        SLICING = 1, 
        PARTS   = 2, 
        INSET   = 3, 
        SKIN    = 4, 
        SUPPORT = 5, 
        EXPORT  = 6, 
        FINISH  = 7
    };
private:
    static double times [N_PROGRESS_STAGES];
    static double accumulated_times [N_PROGRESS_STAGES];
    static const Stage stages[];
    static double totalTiming;
    static double getTotalTiming();
    static float calcOverallProgress(Stage stage, float stage_progress);
public:
    static void init();
    static void messageProgress(Stage stage, int progress_in_stage, int progress_in_stage_max, CommandSocket* commandSocket);
};


} // name space cura
#endif//PROGRESS_H
