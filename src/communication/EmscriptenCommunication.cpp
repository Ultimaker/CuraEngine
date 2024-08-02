// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef __EMSCRIPTEN__
#include "communication/EmscriptenCommunication.h"

#include <emscripten.h>

#include <fmt/format.h>
#include <range/v3/algorithm/contains.hpp>
#include <range/v3/iterator/operations.hpp>
#include <spdlog/spdlog.h>

namespace cura
{

EmscriptenCommunication::EmscriptenCommunication(const std::vector<std::string>& arguments)
    : CommandLine(arguments)
{
    spdlog::info("Emscripten communication initialized");
    if (auto progress_flag = ranges::find(arguments_, "--progress"); progress_flag != arguments_.end())
    {
        progressHandler = *ranges::next(progress_flag);
    }
}

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
    emscripten_run_script(fmt::format("globalThis[\"{}\"]({})", progressHandler, progress).c_str());
}

void EmscriptenCommunication::sliceNext()
{
    spdlog::info("Slice next");
    CommandLine::sliceNext();
};

} // namespace cura

#endif // __EMSCRIPTEN__
