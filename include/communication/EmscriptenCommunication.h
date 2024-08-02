// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef EMSCRIPTENCOMMUNICATION_H
#define EMSCRIPTENCOMMUNICATION_H
#ifdef __EMSCRIPTEN__

#include "communication/CommandLine.h"

namespace cura
{
class Settings;

class EmscriptenCommunication : public CommandLine
{
    std::string progressHandler;

public:
    EmscriptenCommunication(const std::vector<std::string>& arguments);

    void sendSliceUUID(const std::string& slice_uuid) const override;

    void sendPrintTimeMaterialEstimates() const override;

    void sendProgress(double progress) const override;

    void sliceNext() override;
};

} // namespace cura

#endif // __EMSCRIPTEN__
#endif // EMSCRIPTENCOMMUNICATION_H
