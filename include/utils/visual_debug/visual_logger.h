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
#include <range/v3/action/sort.hpp>
#include <range/v3/algorithm/contains.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/drop.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/repeat.hpp>
#include <range/v3/view/stride.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>
#include <spdlog/spdlog.h>
#include <vtu11/vtu11.hpp>

#include "utils/concepts/arachne.h"
#include "utils/concepts/geometry.h"
#include "utils/views/coord.h"
#include "utils/views/get.h"
#include "utils/visual_debug/visual_data_info.h"

namespace cura::debug
{
using layer_map_t = std::unordered_map<int, double>;
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
        vtu11::writePVtu( vtu_path_.string(), id_, { }, 1 );
    };

    VisualLogger(const VisualLogger& other) noexcept : id_ { other.id_ }
                                                       , idx_ { other.idx_ }
                                                       , vtu_path_ { other.vtu_path_ }
                                                       , layer_map_ { other.layer_map_ }
                                                       , cell_dataset_info_ { other.cell_dataset_info_ }
                                                       , point_dataset_info_ { other.point_dataset_info_ } { };

    VisualLogger(VisualLogger&& other) noexcept : id_ { std::move( other.id_ ) }
                                                  , idx_ { std::exchange( other.idx_, 0 ) }
                                                  , vtu_path_ { std::move( other.vtu_path_ ) }
                                                  , layer_map_ { std::move( other.layer_map_ ) }
                                                  , cell_dataset_info_ { std::move( other.cell_dataset_info_ ) }
                                                  , point_dataset_info_ { std::move( other.point_dataset_info_ ) } { };

    VisualLogger& operator=(const VisualLogger& other)
    {
        const std::scoped_lock lock { mutex_ };
        id_ = other.id_;
        idx_ = other.idx_, 0;
        vtu_path_ = other.vtu_path_;
        layer_map_ = other.layer_map_;
        cell_dataset_info_ = other.cell_dataset_info_;
        point_dataset_info_ = other.point_dataset_info_;
        return * this;
    };

    VisualLogger& operator=(VisualLogger&& other) noexcept
    {
        const std::scoped_lock lock { mutex_ };
        id_ = std::move( other.id_ );
        idx_ = std::exchange( other.idx_, 0 );
        vtu_path_ = std::move( other.vtu_path_ );
        layer_map_ = std::move( other.layer_map_ );
        cell_dataset_info_ = std::move( other.cell_dataset_info_ );
        point_dataset_info_ = std::move( other.point_dataset_info_ );
        return * this;
    };

    ~VisualLogger()
    {
        const std::scoped_lock lock { mutex_ };
        vtu11::writePVtu( vtu_path_.string(), id_, ranges::views::concat( point_dataset_info_, cell_dataset_info_ ) | ranges::to_vector, idx_ ); // Need to write this again since we now know the exact number of vtu files
    };

    void setValue(shared_layer_map_t& layer_map)
    {
        layer_map_ = layer_map;
    }

    constexpr void log(const polygon auto& poly, const int layer_idx) { };

    template<typename... VDI>
    constexpr void log(const polygons auto& polys, const int layer_idx, VDI... visual_data_infos)
    {
        ( updateDataInfos( visual_data_infos ), ...);

        std::vector<value_type> points { };
        std::vector<vtu11::VtkIndexType> offsets { 0 };
        auto cell_datas = ranges::views::repeat( vtu11::DataSetData { } ) | ranges::views::take( cell_dataset_info_.size()) | ranges::to_vector;
        auto point_datas = ranges::views::repeat( vtu11::DataSetData { } ) | ranges::views::take( point_dataset_info_.size()) | ranges::to_vector;

        for ( const auto& poly : polys)
        {
            offsets.push_back(offsets.back() + poly.size() );
            // log cell data
            size_t cell_data_idx { };
            for ( const auto& data : cell_dataset_info_ )
            {
                ([ & ] {
                  if constexpr ( visual_data_infos.dataset_type == vtu11::DataSetType::CellData )
                  {
                      if ( visual_data_infos == data )
                      {
                          cell_datas[cell_data_idx++].emplace_back( static_cast<double>( std::invoke( visual_data_infos.projection, poly )));
                      }
                  }
                }(), ...);
            }

            for ( const auto& point : poly )
            {
                // log node data
                size_t pont_data_idx { };
                for ( const auto& data : point_dataset_info_ )
                {
                    ([ & ] {
                      if constexpr ( visual_data_infos.dataset_type == vtu11::DataSetType::PointData )
                      {
                          if ( visual_data_infos == data )
                          {
                              point_datas[pont_data_idx++].emplace_back( static_cast<double>( std::invoke( visual_data_infos.projection, point )));
                          }
                      }
                    }(), ...);
                }
                points.emplace_back( static_cast<value_type>(point.X));
                points.emplace_back( static_cast<value_type>(point.Y));
                points.emplace_back( layer_map_->at( layer_idx ));
            }
        }

        auto connectivity = getConnectivity( points.size() / 3 );
        auto types = getCellTypes( offsets.size() - 1, 7 );
        vtu11::Vtu11UnstructuredMesh mesh_partition { points, connectivity, offsets | ranges::views::drop(1) | ranges::to_vector, types };

        writePartition( mesh_partition, ranges::views::concat( point_datas, cell_datas ) | ranges::to_vector );
    };

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

    template<typename... VDI>
    constexpr void log(const st_edges_viewable auto& st_edges, const int layer_idx, VDI... visual_data_infos)
    {
        ( updateDataInfos( visual_data_infos ), ...);

        std::vector<value_type> points { };
        auto cell_datas = ranges::views::repeat( vtu11::DataSetData { } ) | ranges::views::take( cell_dataset_info_.size()) | ranges::to_vector;
        auto point_datas = ranges::views::repeat( vtu11::DataSetData { } ) | ranges::views::take( point_dataset_info_.size()) | ranges::to_vector;
        for ( const auto& st_edge : st_edges )
        {
            // log cell data
            size_t cell_data_idx { };
            for ( const auto& data : cell_dataset_info_ )
            {
                ([ & ] {
                  if constexpr ( visual_data_infos.dataset_type == vtu11::DataSetType::CellData )
                  {
                      if ( visual_data_infos == data )
                      {
                          cell_datas[cell_data_idx++].emplace_back( static_cast<double>( std::invoke( visual_data_infos.projection, st_edge )));
                      }
                  }
                }(), ...);
            }
            for ( const auto& node : { st_edge.from, st_edge.to } )
            {
                // log node data
                size_t pont_data_idx { };
                for ( const auto& data : point_dataset_info_ )
                {
                    ([ & ] {
                      if constexpr ( visual_data_infos.dataset_type == vtu11::DataSetType::PointData )
                      {
                          if ( visual_data_infos == data )
                          {
                              point_datas[pont_data_idx++].emplace_back( static_cast<double>( std::invoke( visual_data_infos.projection, * node )));
                          }
                      }
                    }(), ...);
                }
                points.emplace_back( static_cast<value_type>(node->p.X));
                points.emplace_back( static_cast<value_type>(node->p.Y));
                points.emplace_back( layer_map_->at( layer_idx ));
            }
        }
        auto connectivity = getConnectivity( points.size() / 3 );
        auto offsets = getOffsets( connectivity.size(), 2 );
        auto types = getCellTypes( offsets.size(), 3 );
        vtu11::Vtu11UnstructuredMesh mesh_partition { points, connectivity, offsets, types };

        writePartition( mesh_partition, ranges::views::concat( point_datas, cell_datas ) | ranges::to_vector );
    };

private:
    std::mutex mutex_;
    std::string id_ { };
    size_t idx_ { };
    std::filesystem::path vtu_path_ { };
    shared_layer_map_t layer_map_ { };
    std::vector<vtu11::DataSetInfo> cell_dataset_info_ { };
    std::vector<vtu11::DataSetInfo> point_dataset_info_ { };

    [[nodiscard]] std::vector<vtu11::VtkIndexType> getConnectivity(size_t no_points)
    {
        return ranges::views::iota( 0 ) | ranges::views::take( no_points ) | ranges::to<std::vector<vtu11::VtkIndexType>>;
    }

    [[nodiscard]] std::vector<vtu11::VtkIndexType> getOffsets(size_t no_cells, size_t step)
    {
        return ranges::views::iota( 0 ) | ranges::views::take( no_cells ) | ranges::views::stride( step ) | ranges::to<std::vector<vtu11::VtkIndexType>>;
    }

    [[nodiscard]] constexpr std::vector<vtu11::VtkCellType> getCellTypes(size_t no_cells, vtu11::VtkIndexType cell_type)
    {
        return ranges::views::repeat( cell_type ) | ranges::views::take( no_cells ) | ranges::to<std::vector<vtu11::VtkCellType>>;
    }

    void updateDataInfos(const auto& visual_data_info) noexcept
    {
        const std::scoped_lock lock { mutex_ };
        if ( visual_data_info.dataset_type == vtu11::DataSetType::CellData )
        {
            if ( std::find( cell_dataset_info_.begin(), cell_dataset_info_.end(), visual_data_info ) == cell_dataset_info_.end())
            {
                cell_dataset_info_.emplace_back( static_cast<vtu11::DataSetInfo>(visual_data_info));
            }
        }
        else if ( visual_data_info.dataset_type == vtu11::DataSetType::PointData )
        {
            if ( std::find( point_dataset_info_.begin(), point_dataset_info_.end(), visual_data_info ) == point_dataset_info_.end())
            {
                point_dataset_info_.emplace_back( static_cast<vtu11::DataSetInfo>(visual_data_info));
            }
        }
    }

    void writePartition(vtu11::Vtu11UnstructuredMesh& mesh_partition, const std::vector<vtu11::DataSetData>& dataset_data = { })
    {
        const std::scoped_lock lock { mutex_ };
        const auto idx = idx_++;
        spdlog::info( "Visual Debugger: <{}> writing partition {}", id_, idx );
        auto data_set_info_view = ranges::views::concat( point_dataset_info_, cell_dataset_info_ );
        if ( !data_set_info_view.empty())
        {
            spdlog::debug( "Visual Debugger: <{}> logging: {}", id_, data_set_info_view | ranges::views::transform( [](const auto& dsi) { return std::get<0>( dsi ); } ));
        }
        vtu11::writePartition( vtu_path_.string(), id_, mesh_partition, data_set_info_view | ranges::to_vector, dataset_data, idx, "ascii" );
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