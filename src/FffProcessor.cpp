//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "FffProcessor.h"
#include "Application.h" //To send the layer data through the communication channel.
#include "communication/Communication.h" //To send the layer data through the communication channel.
#include "SliceDataProcessor.h"
#include "utils/logoutput.h"

namespace cura 
{

FffProcessor FffProcessor::instance; // definition must be in cpp

void FffProcessor::exportSlices(const SliceDataStorage& storage)
{
    auto check = [](bool b)
    {
        if (!b)
            logError("Protobuf serialization failure...\n");
    };

    if (!target_file.empty())
    {
        check(SliceDataProcessor::exportSlices(storage, target_file, true));
    }

    if (!part_slice_file.empty())
    {
        check(SliceDataProcessor::exportSlices(storage, part_slice_file, false));
    }
}

} // namespace cura 
