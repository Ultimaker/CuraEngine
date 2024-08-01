// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "communication/EmscriptenCommunication.h"


#include "spdlog/spdlog.h"

namespace cura
{

void EmscriptenCommunication::sendSliceUUID(const std::string& slice_uuid) const
{
    spdlog::info("Slice UUID: {}", slice_uuid);
}

void EmscriptenCommunication::sendPrintTimeMaterialEstimates() const
{
    spdlog::info("Print time and material estimates");
}

void EmscriptenCommunication::sendProgress(double progress) const
{
    spdlog::info("Progress: {}", progress);
}

void EmscriptenCommunication::sliceNext()
{
    spdlog::info("Slice next");
    CommandLine::sliceNext();
};

} // namespace cura
