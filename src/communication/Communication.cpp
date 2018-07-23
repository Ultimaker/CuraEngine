//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Communication.h"
#include "../utils/logoutput.h"

namespace cura
{

Communication* Communication::instance = nullptr;

Communication::Communication()
{
    instance = this;
}

Communication& Communication::getInstance()
{
    if (!instance)
    {
        //We can't create an instance for you, since you must choose the correct implementation first (e.g. command line vs. libArcus).
        logError("You must construct a communication instance with the correct implementation first.");
        exit(1);
    }
    return *instance;
}

} //namespace cura