/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef LOGOUTPUT_H
#define LOGOUTPUT_H

#ifdef USE_G3LOG

#include "g3log/g3log.hpp"
#include "g3log/logworker.hpp"
#include "g3log/std2_make_unique.hpp"
using namespace g3;

#include "g3logcoloroutsink.h"

#endif

namespace cura {

void increaseVerboseLevel();
void enableProgressLogging();

//Report an error message (always reported, independed of verbose level)
void logError(const char* fmt, ...);
//Report a message if the verbose level is 1 or higher. (defined as _log to prevent clash with log() function from <math.h>)
void log(const char* fmt, ...);

//Report engine progress to interface if any. Only if "enableProgressLogging()" has been called.
void logProgress(const char* type, int value, int maxValue);

}//namespace cura

#endif//LOGOUTPUT_H
