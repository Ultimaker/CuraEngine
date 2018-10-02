//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "FffProcessor.h"
#include "Application.h" //To send the layer data through the communication channel.
#include "communication/Communication.h" //To send the layer data through the communication channel.

namespace cura 
{

FffProcessor FffProcessor::instance; // definition must be in cpp

} // namespace cura 