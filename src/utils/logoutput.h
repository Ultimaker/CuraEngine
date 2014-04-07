/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef LOGOUTPUT_H
#define LOGOUTPUT_H

void increaseVerboseLevel();
void enableProgressLogging();

//Report an error message (always reported, independed of verbose level)
void logError(const char* fmt, ...);
//Report a message if the verbose level is 1 or higher. (defined as _log to prevent clash with log() function from <math.h>)
//void _log(const char* fmt, ...);
//#define log _log

//Report engine progress to interface if any. Only if "enableProgressLogging()" has been called.
void logProgress(const char* type, int value, int maxValue);

#endif//LOGOUTPUT_H
