// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INCLUDE_UTILS_VISUAL_DEBUG_VISUAL_LOGGER_H
#define INCLUDE_UTILS_VISUAL_DEBUG_VISUAL_LOGGER_H

#include <any>
#include <functional>
#include <memory>
#include <mutex>
#include <tuple>
#include <utility>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <range/v3/to_container.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/repeat.hpp>
#include <range/v3/view/stride.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/transform.hpp>
#include <spdlog/spdlog.h>
#include <vtu11/vtu11.hpp>

#include "utils/concepts/arachne.h"
#include "utils/concepts/geometry.h"
#include "utils/views/get.h"
#include "utils/visual_debug/visual_data_info.h"

namespace cura::debug
{
using layer_map_t = std::unordered_map<int, unsigned long long>;
using shared_layer_map_t = std::shared_ptr<layer_map_t>;

#ifdef VISUAL_DEBUG
inline namespace enabled
{
class VisualLogger
{
public:

    constexpr VisualLogger() noexcept = default;

    template<typename... Args>
    constexpr VisualLogger(const std::string& id, const std::filesystem::path& vtu_path, Args&& ... args) : id_ { id }, vtu_path_ { vtu_path }
    {
        spdlog::info( "Visual Debugger: Initializing vtu <{}> file(s) in {}", id_, vtu_path_.string());
        visual_data_.reserve( sizeof...( Args ));
        (visual_data_.emplace_back( args ), ...);
        if ( !visual_data_.empty())
        {
            spdlog::debug( "Visual Debugger: logging: {}", visual_data_ | views::get( & VisualDataInfo::name ));
        }

        vtu11::writePVtu( vtu_path_.string(), id_, getDatasetInfos(), idx_ );
    };

    VisualLogger(const VisualLogger& other) noexcept = default;

    VisualLogger(VisualLogger&& other) noexcept = default;

    VisualLogger& operator=(const VisualLogger& other) = default;

    VisualLogger& operator=(VisualLogger&& other) noexcept = default;

    ~VisualLogger()
    {
        const auto idx = idx_++;
        spdlog::info( "Visual Debugger: Finalizing vtu <{}> with a total of {} parallel vtu(s) files", id_, idx );
        vtu11::writePVtu( vtu_path_.string(), id_, getDatasetInfos(), idx ); // Need to write this again since we now know the exact number of vtu files
    };

    constexpr void log(const polygon auto& poly, const int layer_idx) { };

    constexpr void log(const polygons auto& polys, const int layer_idx) { };

    constexpr void log(const vertices auto& vertices)
    {
    };

    constexpr void log(const st_edges_viewable auto& polys, const int layer_idx) { };

private:
    std::string id_ { };
    size_t idx_ { 1UL };
    std::filesystem::path vtu_path_ { };
    shared_layer_map_t layer_map_ { };
    std::vector<VisualDataInfo> visual_data_ { };

    [[nodiscard]] std::vector<vtu11::VtkIndexType> getConnectivity(size_t no_points)
    {
        return ranges::views::iota( 0 ) | ranges::views::take( no_points * 3 ) | ranges::to<std::vector<vtu11::VtkIndexType>>;
    }

    [[nodiscard]] std::vector<vtu11::VtkIndexType> getOffsets(size_t no_cells)
    {
        return ranges::views::iota( 0 ) | ranges::views::take( no_cells ) | ranges::views::stride( 3 ) | ranges::to<std::vector<vtu11::VtkIndexType>>;
    }

    [[nodiscard]] std::vector<vtu11::VtkCellType> getCellTypes(size_t no_cells, vtu11::VtkIndexType cell_type)
    {
        return ranges::views::repeat( cell_type ) | ranges::views::take( no_cells ) | ranges::to<std::vector<vtu11::VtkCellType>>;
    }

    [[nodiscard]] constexpr std::vector<vtu11::DataSetInfo> getDatasetInfos()
    {
        return visual_data_ | ranges::views::transform( [](auto& val) { return val.getDataSetInfo(); } ) | ranges::to<std::vector<vtu11::DataSetInfo>>;
    }

    void writePartition(vtu11::Vtu11UnstructuredMesh& mesh_partition, const std::vector<vtu11::DataSetData>& dataset_data)
    {
        const auto idx = idx_++;
        spdlog::info( "Visual Debugger: writing <{}> partition {}", id_, idx );
        vtu11::writePartition( vtu_path_.string(), id_, mesh_partition, getDatasetInfos(), dataset_data, idx, "RawBinary" );
    }
};
} // namespace enabled

using shared_visual_logger_t = std::shared_ptr<VisualLogger>;
#endif

#ifndef VISUAL_DEBUG
inline namespace disabled
{
class VisualLogger
{
public:
    template<typename... Args>
    constexpr VisualLogger(Args... args) { };

    template<typename... Args>
    constexpr void log(Args... args) { };

    template<typename... Args>
    constexpr void setValue(Args... args) { };
};
} // namespace disabled

using shared_visual_logger_t = std::shared_ptr<VisualLogger>;
#endif
}// namespace cura::debug


#endif// INCLUDE_UTILS_VISUAL_DEBUG_VISUAL_LOGGER_H