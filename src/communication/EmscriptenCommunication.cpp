// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef __EMSCRIPTEN__
#include "communication/EmscriptenCommunication.h"

#include <array>
#include <emscripten.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <string_view>
#include <utility>

#include <fmt/format.h>
#include <range/v3/algorithm/find.hpp>
#include <range/v3/iterator/operations.hpp>
#include <spdlog/spdlog.h>

#include "Application.h"
#include "FffProcessor.h"
#include "Slice.h"
#include "utils/string.h"

namespace cura
{

EmscriptenCommunication::EmscriptenCommunication(const std::vector<std::string>& arguments)
    : CommandLine(arguments)
{
    spdlog::info("Emscripten communication initialized");
    if (auto progress_flag = ranges::find(arguments_, "--progress_cb"); progress_flag != arguments_.end() && ranges::next(progress_flag) != arguments_.end())
    {
        progress_handler_ = *ranges::next(progress_flag);
    }
    if (auto slice_info_flag = ranges::find(arguments_, "--slice_info_cb"); slice_info_flag != arguments_.end() && ranges::next(slice_info_flag) != arguments_.end())
    {
        slice_info_handler_ = *ranges::next(slice_info_flag);
    }
    if (auto engine_info_flag = ranges::find(arguments_, "--engine_info_cb"); engine_info_flag != arguments_.end() && ranges::next(engine_info_flag) != arguments_.end())
    {
        engine_info_handler_ = *ranges::next(engine_info_flag);
    }

    if (! engine_info_handler_.empty())
    {
        auto engine_info = createEngineInfoMessage();
        emscripten_run_script(fmt::format("globalThis[\"{}\"]({})", engine_info_handler_, engine_info).c_str());
    }
}

void EmscriptenCommunication::sendProgress(double progress) const
{
    if (! progress_handler_.empty())
    {
        emscripten_run_script(fmt::format("globalThis[\"{}\"]({})", progress_handler_, progress).c_str());
    }
}

std::string EmscriptenCommunication::createSliceInfoMessage(const std::vector<cura::Duration>& time_estimates, const PrintInformation& print_information)
{
    // Construct a string with rapidjson containing the slice information
    rapidjson::Document doc;
    auto& allocator = doc.GetAllocator();
    doc.SetObject();

    // Set the slice UUID
    rapidjson::Value slice_uuid("slice_uuid", allocator);
    rapidjson::Value uuid(Application::getInstance().instance_uuid_.c_str(), allocator);
    doc.AddMember(slice_uuid, uuid, allocator);

    // Set the time estimates
    static constexpr std::array<std::pair<std::string_view, PrintFeatureType>, 14> feature_types{ {
        { "infill", PrintFeatureType::Infill },
        { "skin", PrintFeatureType::Skin },
        { "support", PrintFeatureType::Support },
        { "inner_wall", PrintFeatureType::InnerWall },
        { "move_combing", PrintFeatureType::MoveUnretracted },
        { "move_retraction", PrintFeatureType::MoveRetracted },
        { "outer_wall", PrintFeatureType::OuterWall },
        { "prime_tower", PrintFeatureType::PrimeTower },
        { "skirt_brim", PrintFeatureType::SkirtBrim },
        { "support_infill", PrintFeatureType::SupportInfill },
        { "support_interface", PrintFeatureType::SupportInterface },
        { "move_while_retracting", PrintFeatureType::MoveWhileRetracting },
        { "move_while_unretracting", PrintFeatureType::MoveWhileUnretracting },
        { "stationary_retract_unretract", PrintFeatureType::StationaryRetractUnretract },
    } };
    rapidjson::Value time_estimates_json(rapidjson::kObjectType);
    for (const auto& [feature, duration_idx] : feature_types)
    {
        rapidjson::Value feature_time(rapidjson::StringRef(feature.data(), feature.size()));
        rapidjson::Value feature_duration(time_estimates[static_cast<unsigned char>(duration_idx)]);
        time_estimates_json.AddMember(feature_time, feature_duration, allocator);
    }
    doc.AddMember("time_estimates", time_estimates_json, allocator);

    // Set the material estimates
    rapidjson::Value material_estimates_json(rapidjson::kObjectType);
    const Scene& scene = Application::getInstance().current_slice_->scene;

    for (size_t extruder_nr = 0; extruder_nr < scene.extruders.size(); extruder_nr++)
    {
        const std::optional<ExtruderPrintInformation>& extruder_info = print_information.extruders_info[extruder_nr];
        const double value = extruder_info.has_value() ? extruder_info->filament_length : 0.0;
        spdlog::info("Extruder {} used {} [mm] of filament", extruder_nr, value);
        const std::string extruder_id_str = fmt::format("{}", extruder_nr);
        rapidjson::Value extruder_id(extruder_id_str.c_str(), static_cast<rapidjson::SizeType>(extruder_id_str.size()), allocator);
        rapidjson::Value extruder_material_estimate(value);
        material_estimates_json.AddMember(extruder_id, extruder_material_estimate, allocator);
    }
    doc.AddMember("material_estimates", material_estimates_json, allocator);

    // Set CuraEngine information
    rapidjson::Value slicer_info_json(rapidjson::kObjectType);
    bool has_painted_support = false;
    bool has_painted_extruder = false;
    bool has_painted_seam = false;
    uint32_t painted_mesh_count = 0;
    for (const MeshGroup& mesh_group : scene.mesh_groups)
    {
        has_painted_support |= mesh_group.has_painted_support;
        has_painted_extruder |= mesh_group.has_painted_extruder;
        has_painted_seam |= mesh_group.has_painted_seam;
        painted_mesh_count += mesh_group.painted_mesh_count;
    }
    rapidjson::Value paint_features_json(rapidjson::kObjectType);
    paint_features_json.AddMember("has_painted_support", has_painted_support, allocator);
    paint_features_json.AddMember("has_painted_extruder", has_painted_extruder, allocator);
    paint_features_json.AddMember("has_painted_seam", has_painted_seam, allocator);
    paint_features_json.AddMember("painted_mesh_count", painted_mesh_count, allocator);
    slicer_info_json.AddMember("paint_features", paint_features_json, allocator);
    doc.AddMember("slicer_info", slicer_info_json, allocator);

    // Serialize the JSON document to a string
    rapidjson::StringBuffer buffer;
    rapidjson::Writer writer(buffer);
    doc.Accept(writer);
    return buffer.GetString();
}

std::string EmscriptenCommunication::createEngineInfoMessage()
{
    // Construct a string with rapidjson containing the engine information
    rapidjson::Document doc;
    auto& allocator = doc.GetAllocator();
    doc.SetObject();

    // Set the slicer version
    rapidjson::Value version("version", allocator);
    rapidjson::Value version_value(CURA_ENGINE_VERSION, allocator);
    doc.AddMember(version, version_value, allocator);

    // Set the hash
    rapidjson::Value hash("hash", allocator);
    rapidjson::Value hash_value(CURA_ENGINE_HASH, allocator);
    doc.AddMember(hash, hash_value, allocator);

    // Serialize the JSON document to a string
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer);
    return buffer.GetString();
}

void EmscriptenCommunication::sendPrintInformation(const std::vector<cura::Duration>& time_estimates, const PrintInformation& print_information) const
{
    if (! slice_info_handler_.empty())
    {
        auto slice_info = createSliceInfoMessage(time_estimates, print_information);
        emscripten_run_script(fmt::format("globalThis[\"{}\"]({})", slice_info_handler_, slice_info).c_str());
    }
}

} // namespace cura

#endif // __EMSCRIPTEN__
