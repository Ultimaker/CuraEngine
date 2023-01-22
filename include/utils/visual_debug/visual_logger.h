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
#include <range/v3/view/join.hpp>
#include <range/v3/view/repeat.hpp>
#include <range/v3/view/stride.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/transform.hpp>
#include <spdlog/spdlog.h>
#include <vtu11/vtu11.hpp>

#include "utils/concepts/arachne.h"
#include "utils/concepts/geometry.h"
#include "utils/views/coord.h"
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
    using value_type = double;

    constexpr VisualLogger() noexcept = default;

    template<typename... Args>
    VisualLogger(const std::string& id, const std::filesystem::path& vtu_path, Args&& ... args) : id_ { id }, vtu_path_ { vtu_path }
    {
        const std::scoped_lock lock { mutex_ };
        spdlog::info( "Visual Debugger: Initializing vtu <{}> file(s) in {}", id_, vtu_path_.string());
        visual_data_.reserve( sizeof...( Args ));
        (visual_data_.emplace_back( args ), ...);
        if ( !visual_data_.empty())
        {
            spdlog::debug( "Visual Debugger: <{}> logging: {}", id_, visual_data_ | views::get( & VisualDataInfo::name ));
        }
        vtu11::writePVtu( vtu_path_.string(), id_, getDatasetInfos(), 1 );
    };

    VisualLogger(const VisualLogger& other) noexcept : id_ { other.id_ }
                                                       , idx_ { other.idx_ }
                                                       , vtu_path_ { other.vtu_path_ }
                                                       , layer_map_ { other.layer_map_ }
                                                       , visual_data_ { other.visual_data_ } { };

    VisualLogger(VisualLogger&& other) noexcept : id_ { std::move( other.id_ ) }
                                                  , idx_ { std::exchange( other.idx_, 0 ) }
                                                  , vtu_path_ { std::move( other.vtu_path_ ) }
                                                  , layer_map_ { std::move( other.layer_map_ ) }
                                                  , visual_data_ { std::move( other.visual_data_ ) } { };

    VisualLogger& operator=(const VisualLogger& other)
    {
        const std::scoped_lock lock { mutex_ };
        id_ = other.id_;
        idx_ = other.idx_, 0;
        vtu_path_ = other.vtu_path_;
        layer_map_ = other.layer_map_;
        visual_data_ = other.visual_data_;
        return * this;
    };

    VisualLogger& operator=(VisualLogger&& other) noexcept
    {
        const std::scoped_lock lock { mutex_ };
        id_ = std::move( other.id_ );
        idx_ = std::exchange( other.idx_, 0 );
        vtu_path_ = std::move( other.vtu_path_ );
        layer_map_ = std::move( other.layer_map_ );
        visual_data_ = std::move( other.visual_data_ );
        return * this;
    };

    ~VisualLogger()
    {
        const std::scoped_lock lock { mutex_ };
        vtu11::writePVtu( vtu_path_.string(), id_, getDatasetInfos(), idx_ ); // Need to write this again since we now know the exact number of vtu files
    };

    void setValue(std::shared_ptr<layer_map_t> layer_map)
    {
        layer_map_ = layer_map;
    }

    constexpr void log(const polygon auto& poly, const int layer_idx) { };

    constexpr void log(const polygons auto& polys, const int layer_idx) { };

    constexpr void log(const mesh auto& mesh)
    {
        // FIXME: add the last face as well
        std::vector<value_type> points { };
        for ( const auto& face : mesh.faces )
        {
            for ( const auto& vertex_idx : face.vertex_index )
            {
                const auto& vertex = mesh.vertices[vertex_idx];
                points.emplace_back( static_cast<value_type>(vertex.p.x));
                points.emplace_back( static_cast<value_type>(vertex.p.y));
                points.emplace_back( static_cast<value_type>(vertex.p.z));
            }
        }
        auto connectivity = getConnectivity( mesh.faces.size() * 3 );
        auto offsets = getOffsets( connectivity.size(), 3 );
        auto types = getCellTypes( offsets.size(), 5 );
        vtu11::Vtu11UnstructuredMesh mesh_partition { points, connectivity, offsets, types };
        writePartition( mesh_partition );
    };

    constexpr void log(const st_edges_viewable auto& polys, const int layer_idx) { };

private:
    std::mutex mutex_;
    std::string id_ { };
    size_t idx_ { };
    std::filesystem::path vtu_path_ { };
    shared_layer_map_t layer_map_ { };
    std::vector<VisualDataInfo> visual_data_ { };

    [[nodiscard]] std::vector<vtu11::VtkIndexType> getConnectivity(size_t no_points)
    {
        return ranges::views::iota( 0 ) | ranges::views::take( no_points ) | ranges::to<std::vector<vtu11::VtkIndexType>>;
    }

    [[nodiscard]] std::vector<vtu11::VtkIndexType> getOffsets(size_t no_cells, size_t step)
    {
        return ranges::views::iota( 0 ) | ranges::views::take( no_cells ) | ranges::views::stride( step ) | ranges::to<std::vector<vtu11::VtkIndexType>>;
    }

    [[nodiscard]] std::vector<vtu11::VtkCellType> getCellTypes(size_t no_cells, vtu11::VtkIndexType cell_type)
    {
        return ranges::views::repeat( cell_type ) | ranges::views::take( no_cells ) | ranges::to<std::vector<vtu11::VtkCellType>>;
    }

    [[nodiscard]] constexpr std::vector<vtu11::DataSetInfo> getDatasetInfos()
    {
        return visual_data_ | ranges::views::transform( [](auto& val) { return val.getDataSetInfo(); } ) | ranges::to<std::vector<vtu11::DataSetInfo>>;
    }

    void writePartition(vtu11::Vtu11UnstructuredMesh& mesh_partition, const std::vector<vtu11::DataSetData>& dataset_data = { })
    {
        const std::scoped_lock lock { mutex_ };
        const auto idx = idx_++;
        spdlog::info( "Visual Debugger: writing <{}> partition {}", id_, idx );
        vtu11::writePartition( vtu_path_.string(), id_, mesh_partition, getDatasetInfos(), dataset_data, idx, "Ascii" );
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