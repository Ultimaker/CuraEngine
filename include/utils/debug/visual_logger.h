// Copyright (c) 2023 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_DEBUG_VISUAL_LOGGER_H
#define UTILS_DEBUG_VISUAL_LOGGER_H

#include <experimental/source_location>
#include <memory>
#include <mutex>
#include <utility>

#include <range/v3/all.hpp>
#include <spdlog/spdlog.h>
#include <vtu11/vtu11.hpp>

#include "Application.h"
#include "settings/Settings.h"
#include "utils/concepts/geometry.h"

namespace cura::debug
{
class VisualLogger
{
public:
    VisualLogger() : vtu_dir_{}, dataset_info_{} {};

    explicit VisualLogger(const std::string& id) : id_{ id }, vtu_dir_{}, dataset_info_{}
    {
        spdlog::info("Visual Debugger: Initializing vtu <{}> file(s) in {}", id_, vtu_dir_.string());
        vtu11::writePVtu(vtu_dir_.string(), id_, dataset_info_, 1);
    }

    VisualLogger(const std::string& id, std::filesystem::path vtu_dir, std::vector<vtu11::DataSetInfo>& dataset_info) : id_{ id }, vtu_dir_{ std::move(vtu_dir) }, dataset_info_{ dataset_info }
    {
        spdlog::info("Visual Debugger: Initializing vtu <{}> file(s) in {}", id_, vtu_dir_.string());
        vtu11::writePVtu(vtu_dir_.string(), id_, dataset_info_, 1);
    }

    ~VisualLogger()
    {
        const auto idx = getIdx();
        spdlog::info("Visual Debugger: Finalizing vtu <{}> with a total of {} parallel vtu(s) files", id_, idx);
        vtu11::writePVtu(vtu_dir_.string(), id_, dataset_info_, idx); // Need to write this again since we now know the exact number of vtu files
    }

    VisualLogger(const VisualLogger& other) : id_{ other.id_ }, vtu_dir_{ other.vtu_dir_ }, idx_{ other.idx_ }, dataset_info_{ other.dataset_info_ } {};

    VisualLogger(VisualLogger&& other) noexcept : id_{ std::exchange(other.id_, {}) }, vtu_dir_{ std::exchange(other.vtu_dir_, {}) }, idx_{ std::exchange(other.idx_, 0) }, dataset_info_{ std::exchange(other.dataset_info_, {}) } {};

    VisualLogger& operator=(const VisualLogger& other)
    {
        return *this = VisualLogger(other);
    }

    VisualLogger& operator=(VisualLogger&& other) noexcept
    {
        std::swap(vtu_dir_, other.vtu_dir_);
        std::swap(idx_, other.idx_);
        std::swap(id_, other.id_);
        std::swap(dataset_info_, other.dataset_info_);
        return *this;
    }

    [[nodiscard]] const std::string& getId() const
    {
        return id_;
    }

    void setSettings(std::shared_ptr<Settings> settings)
    {
        settings_ = settings;
    }

    void setLayers(std::shared_ptr<std::unordered_map<int, coord_t>> layer_heights)
    {
        layer_height_ = layer_heights;
    }

#ifndef VISUAL_DEBUG
    template<typename... Args>
    constexpr void log(Args... args){};
#else
    void log(const isMesh auto& mesh, const std::experimental::source_location location = std::experimental::source_location::current())
    {
        spdlog::info("Visual Debugger: logging <{}> {} - {} - L{}: {}", id_, location.file_name(), location.function_name(), location.line(), mesh.mesh_name);
        using float_type = double;
        std::vector<float_type> points{};
        std::vector<double> pointData{};
        std::vector<double> cellData{};
        for (const auto& [face_idx, face] : mesh.faces | ranges::views::enumerate)
        {
            for (const auto& vertex_idx : face.vertex_index)
            {
                const auto& vertex = mesh.vertices[vertex_idx];
                points.emplace_back(static_cast<float_type>(vertex.p.x));
                points.emplace_back(static_cast<float_type>(vertex.p.y));
                points.emplace_back(static_cast<float_type>(vertex.p.z));
                pointData.push_back(vertex_idx);
            }
            cellData.push_back(face_idx);
        }
        auto connectivity = ranges::views::iota(0) | ranges::views::take(mesh.faces.size() * 3) | ranges::to<std::vector<vtu11::VtkIndexType>>;
        auto offsets = ranges::views::iota(0) | ranges::views::take(connectivity.size()) | ranges::views::stride(3) | ranges::to<std::vector<vtu11::VtkIndexType>>;
        auto types = ranges::views::repeat(5) | ranges::views::take(offsets.size()) | ranges::to<std::vector<vtu11::VtkCellType>>;

        vtu11::Vtu11UnstructuredMesh meshPartition{ points, connectivity, offsets, types };
        std::vector<vtu11::DataSetData> dataSetData{ pointData, cellData };

        writePartition(meshPartition, dataSetData);
    }

    void log(const isLayers auto& layers, const std::experimental::source_location location = std::experimental::source_location::current())
    {
        spdlog::info("Visual Debugger: logging <{}> {} - {} - L{}", id_, location.file_name(), location.function_name(), location.line());
        const auto x = settings_->get<coord_t>("machine_width");
        const auto y = settings_->get<coord_t>("machine_depth");
        using float_type = double;
        std::vector<float_type> points{};
        std::vector<double> cellData{};
        for (const auto& [layer_idx, layer] : layers | ranges::views::enumerate)
        {
            points.emplace_back(0.);
            points.emplace_back(0.);
            points.emplace_back(layer.z);

            points.emplace_back(0.);
            points.emplace_back(static_cast<float_type>(y));
            points.emplace_back(layer.z);

            points.emplace_back(static_cast<float_type>(x));
            points.emplace_back(static_cast<float_type>(y));
            points.emplace_back(layer.z);

            points.emplace_back(static_cast<float_type>(x));
            points.emplace_back(0.);
            points.emplace_back(layer.z);
            cellData.push_back(layer_idx);
        }
        auto connectivity = ranges::views::iota(0) | ranges::views::take(layers.size() * 4) | ranges::to<std::vector<vtu11::VtkIndexType>>;
        auto offsets = ranges::views::iota(0) | ranges::views::take(connectivity.size()) | ranges::views::stride(4) | ranges::to<std::vector<vtu11::VtkIndexType>>;
        auto types = ranges::views::repeat(7) | ranges::views::take(offsets.size()) | ranges::to<std::vector<vtu11::VtkCellType>>;

        vtu11::Vtu11UnstructuredMesh meshPartition{ points, connectivity, offsets, types };
        std::vector<vtu11::DataSetData> dataSetData{ cellData };

        writePartition(meshPartition, dataSetData);
    }

    void log(const isPolygons auto& polys, const auto& z, const std::experimental::source_location location = std::experimental::source_location::current())
    {
        spdlog::info("Visual Debugger: logging <{}> {} - {} - L{}", id_, location.file_name(), location.function_name(), location.line());

        using float_type = double;
        std::vector<float_type> points{};
        std::vector<double> cellData{};
        for (const auto& poly : polys)
        {
            for (const auto& point : poly)
            {
                points.emplace_back(static_cast<float_type>(point.X));
                points.emplace_back(static_cast<float_type>(point.Y));
                points.emplace_back(static_cast<float_type>(z));
            }
        }
        auto connectivity = ranges::views::iota(0) | ranges::views::take(points.size() / 3) | ranges::to<std::vector<vtu11::VtkIndexType>>;
        auto offsets = ranges::views::iota(0) | ranges::views::take(connectivity.size()) | ranges::views::stride(3) | ranges::to<std::vector<vtu11::VtkIndexType>>;
        auto types = ranges::views::repeat(7) | ranges::views::take(offsets.size()) | ranges::to<std::vector<vtu11::VtkCellType>>;
        vtu11::Vtu11UnstructuredMesh meshPartition{ points, connectivity, offsets, types };
        std::vector<vtu11::DataSetData> dataSetData{ cellData };

        writePartition(meshPartition, dataSetData);
    }
#endif

private:
    std::mutex mutex_;
    std::filesystem::path vtu_dir_;
    size_t idx_{ 0 };
    std::vector<vtu11::DataSetInfo> dataset_info_;
    std::string id_{};
    std::shared_ptr<Settings> settings_;
    std::shared_ptr<std::unordered_map<int, coord_t>> layer_height_;

    void writePartition(vtu11::Vtu11UnstructuredMesh& mesh_partition, const std::vector<vtu11::DataSetData>& dataset_data)
    {
        const auto idx = getIdx();
        spdlog::info("Visual Debugger: writing <{}> parition {}", id_, idx);
        vtu11::writePartition(vtu_dir_.string(), id_, mesh_partition, dataset_info_, dataset_data, idx, "RawBinary");
        setIdx(idx + 1);
    }

    [[nodiscard]] size_t getIdx()
    {
        std::lock_guard<std::mutex> guard(mutex_);
        return idx_;
    }

    void setIdx(size_t idx)
    {
        std::lock_guard<std::mutex> guard(mutex_);
        idx_ = idx;
    }
};

using shared_visual_logger = std::shared_ptr<VisualLogger>;
} // namespace cura::debug


#endif // UTILS_DEBUG_VISUAL_LOGGER_H
