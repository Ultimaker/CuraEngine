//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Application.h"

Application& Application::getInstance()
{
    static Application instance; //Constructs using the default constructor.
    return instance;
}