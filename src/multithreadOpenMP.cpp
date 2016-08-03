/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#include "multithreadOpenMP.h"
#include <cstdlib>

namespace cura
{

bool abort_execution = false;

#ifdef _OPENMP
void handleMultithreadAbort()
{
    if (checkMultithreadAbort())
    {
        std::exit(17);
    }
}
#endif

}//namespace cura