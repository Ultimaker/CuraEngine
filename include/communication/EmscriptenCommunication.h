// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef EMSCRIPTENCOMMUNICATION_H
#define EMSCRIPTENCOMMUNICATION_H
#ifdef __EMSCRIPTEN__

#include "communication/CommandLine.h"

namespace cura
{

class EmscriptenCommunication : public CommandLine
{
    std::string progress_handler_;
    std::string slice_info_handler_;

    [[nodiscard]] static std::string createSliceInfoMessage() ;

public:
    EmscriptenCommunication(const std::vector<std::string>& arguments);

    void sendProgress(double progress) const override;

    void sliceNext() override;
};

} // namespace cura

#endif // __EMSCRIPTEN__
#endif // EMSCRIPTENCOMMUNICATION_H
