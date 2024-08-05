// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <rapidjson/document.h>

#include <fmt/format.h>
#include <range/v3/algorithm/contains.hpp>
#include <range/v3/iterator/operations.hpp>
#include <spdlog/spdlog.h>

#include "FffProcessor.h"
#include "communication/EmscriptenCommunication.h"

namespace cura
{

EmscriptenCommunication::EmscriptenCommunication(const std::vector<std::string>& arguments)
    : CommandLine(arguments)
{
    spdlog::info("Emscripten communication initialized");
    if (auto progress_flag = ranges::find(arguments_, "--progress_cb"); progress_flag != arguments_.end())
    {
        progressHandler = *ranges::next(progress_flag);
    }
    if (auto slice_info_flag = ranges::find(arguments_, "--slice_info_cb"); slice_info_flag != arguments_.end())
    {
        sliceInfoHandler = *ranges::next(slice_info_flag);
    }
}

void EmscriptenCommunication::sendProgress(double progress) const
{
    spdlog::info("Progress: {}", progress);
    emscripten_run_script(fmt::format("globalThis[\"{}\"]({})", progressHandler, progress).c_str());
}

std::string EmscriptenCommunication::createSliceInfoMessage() const
{
    // Construct a string with rapidjson containing the slice information
    rapidjson::Document doc;
    doc.SetObject();
    auto& allocator = doc.GetAllocator();
    rapidjson::Value time_estimates_json(rapidjson::kObjectType);

    auto time_estimates = FffProcessor::getInstance()->getTotalPrintTimePerFeature();
    for (const auto& [feature, duration_idx] : std::vector<std::tuple<std::string, PrintFeatureType>>{ { "infill", PrintFeatureType::Infill },
                                                                                                       { "skin", PrintFeatureType::Skin },
                                                                                                       { "support", PrintFeatureType::Support },
                                                                                                       { "inner_wall", PrintFeatureType::InnerWall },
                                                                                                       { "move_combing", PrintFeatureType::MoveCombing },
                                                                                                       { "move_retraction", PrintFeatureType::MoveRetraction },
                                                                                                       { "outer_wall", PrintFeatureType::OuterWall },
                                                                                                       { "prime_tower", PrintFeatureType::PrimeTower },
                                                                                                       { "skirt_brim", PrintFeatureType::SkirtBrim },
                                                                                                       { "support_infill", PrintFeatureType::SupportInfill },
                                                                                                       { "support_interface", PrintFeatureType::SupportInterface } })
    {
        rapidjson::Value feature_time_estimate(feature.c_str(), allocator);
        rapidjson::Value feature_duration(time_estimates[static_cast<unsigned char>(duration_idx)]);
        time_estimates_json.AddMember(feature_time_estimate, feature_duration, allocator);
    }
    doc.AddMember("time_estimates", time_estimates_json, allocator);
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer);
    auto ret = buffer.GetString();
    spdlog::info("Slice info message: {}", ret);
    return buffer.GetString();
}

void EmscriptenCommunication::sliceNext()
{
    spdlog::info("Slice next");
    CommandLine::sliceNext();
    spdlog::info("Slice next done");
    auto slice_info = createSliceInfoMessage();
    spdlog::info("Slice info handler: {}", slice_info);
    emscripten_run_script(fmt::format("globalThis[\"{}\"]({})", sliceInfoHandler, slice_info).c_str());
};

} // namespace cura

#endif // __EMSCRIPTEN__
