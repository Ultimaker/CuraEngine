// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INCLUDE_UTILS_VISUAL_DEBUG_VISUAL_LOGGER_H
#define INCLUDE_UTILS_VISUAL_DEBUG_VISUAL_LOGGER_H

#include <any>
#include <functional>
#include <memory>
#include <mutex>
#include <tuple>
#include <type_traits>
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
#include <range/v3/view/map.hpp>
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
#include "utils/visual_debug/cell_type.h"
#include "utils/visual_debug/section_type.h"
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
    VisualLogger(const std::string& id, const size_t logger_idx, const std::filesystem::path& vtu_path, SectionType section_type = SectionType::NA, Args&& ... args) : id_ { id }, logger_idx_ { logger_idx }, vtu_path_ { vtu_path }, section_type_ { section_type }
    {
        const std::scoped_lock lock { mutex_ };
        spdlog::info( "Visual Debugger: Initializing Logger <{}>-<{}> file(s) in {}", id_, logger_idx_, vtu_path_.string());
        vtu11::writePVtu( vtu_path_.string(), id_, { }, 1 );
    };

    void setValue(shared_layer_map_t& layer_map)
    {
        layer_map_ = layer_map;
    }

    constexpr void log(const layer_viewable auto& layers)
    {
        for (const auto& [layer_idx, layer] : layers | ranges::views::enumerate)
        {
            log(layer.polygons, layer_idx);
        }
    };

    template<typename... VDI>
    constexpr void log(const polygons auto& polys, const int layer_idx, VDI... visual_data_infos)
    {
        updateDataInfos( CellVisualDataInfo { "log_idx" } );
        updateDataInfos( CellVisualDataInfo { "layer_idx" } );
        updateDataInfos( CellVisualDataInfo { "logger_idx" } );
        updateDataInfos( CellVisualDataInfo { "cell_idx" } );
        updateDataInfos( PointVisualDataInfo { "log_idx" } );
        updateDataInfos( PointVisualDataInfo { "layer_idx" } );
        updateDataInfos( PointVisualDataInfo { "logger_idx" } );
        updateDataInfos( PointVisualDataInfo { "point_idx" } );
        ( updateDataInfos( visual_data_infos ), ...);

        std::vector<value_type> points { };
        std::vector<vtu11::VtkIndexType> offsets { 0 };
        auto cell_datas = cell_dataset_info_ | ranges::views::transform( [](const auto& dsi) { return std::make_pair( std::get<0>( dsi ), vtu11::DataSetData { } ); } ) | ranges::to<std::unordered_map<std::string, vtu11::DataSetData>>;
        auto point_datas = point_dataset_info_ | ranges::views::transform( [](const auto& dsi) { return std::make_pair( std::get<0>( dsi ), vtu11::DataSetData { } ); } ) | ranges::to<std::unordered_map<std::string, vtu11::DataSetData>>;

        size_t cell_idx { };
        for ( const auto& poly : polys )
        {
            offsets.push_back( offsets.back() + poly.size());
            // log cell data
            ([ & ] {
              if constexpr ( visual_data_infos.dataset_type == vtu11::DataSetType::CellData )
              {
                  cell_datas[visual_data_infos.name].emplace_back( static_cast<value_type>( std::invoke( visual_data_infos.projection, poly )));
              }
            }(), ...); // variadic arguments to the log call
            cell_datas["log_idx"].emplace_back( static_cast<value_type>( idx_ ));
            cell_datas["layer_idx"].emplace_back( static_cast<value_type>( layer_idx ));
            cell_datas["logger_idx"].emplace_back( static_cast<value_type>( logger_idx_ ));
            cell_datas["cell_idx"].emplace_back( static_cast<value_type>( cell_idx++ ));

            size_t point_idx { };
            for ( const auto& point : poly )
            {
                // log node data
                ([ & ] {
                  if constexpr ( visual_data_infos.dataset_type == vtu11::DataSetType::PointData )
                  {
                      point_datas[visual_data_infos.name].emplace_back( static_cast<value_type>( std::invoke( visual_data_infos.projection, point )));
                  }
                }(), ...); // variadic arguments to the log call
                point_datas["log_idx"].emplace_back( static_cast<value_type>( idx_ ));
                point_datas["layer_idx"].emplace_back( static_cast<value_type>( layer_idx ));
                point_datas["logger_idx"].emplace_back( static_cast<value_type>( logger_idx_ ));
                point_datas["point_idx"].emplace_back( static_cast<value_type>( point_idx++ ));

                points.emplace_back( static_cast<value_type>(point.X));
                points.emplace_back( static_cast<value_type>(point.Y));
                points.emplace_back( layer_map_->at( layer_idx ));
            }
        }

        auto connectivity = getConnectivity( points.size() / 3 );
        auto types = getCellTypes( offsets.size() - 1, static_cast<long>(CellType::POLYGON));
        vtu11::Vtu11UnstructuredMesh mesh_partition { points, connectivity, offsets | ranges::views::drop( 1 ) | ranges::to_vector, types };

        writePartition( mesh_partition, point_datas, cell_datas );
    };

    constexpr void log(const mesh auto& mesh)
    {
        // FIXME: add the last face as well
        updateDataInfos( CellVisualDataInfo { "log_idx" } );
        updateDataInfos( CellVisualDataInfo { "logger_idx" } );
        updateDataInfos( CellVisualDataInfo { "cell_idx" } );
        updateDataInfos( PointVisualDataInfo { "log_idx" } );
        updateDataInfos( PointVisualDataInfo { "logger_idx" } );
        updateDataInfos( PointVisualDataInfo { "point_idx" } );

        auto cell_datas = cell_dataset_info_ | ranges::views::transform( [](const auto& dsi) { return std::make_pair( std::get<0>( dsi ), vtu11::DataSetData { } ); } ) | ranges::to<std::unordered_map<std::string, vtu11::DataSetData>>;
        auto point_datas = point_dataset_info_ | ranges::views::transform( [](const auto& dsi) { return std::make_pair( std::get<0>( dsi ), vtu11::DataSetData { } ); } ) | ranges::to<std::unordered_map<std::string, vtu11::DataSetData>>;

        std::vector<value_type> points { };
        size_t cell_idx { };
        for ( const auto& face : mesh.faces )
        {
            cell_datas["log_idx"].emplace_back( static_cast<value_type>( idx_ ));
            cell_datas["logger_idx"].emplace_back( static_cast<value_type>( logger_idx_ ));
            cell_datas["cell_idx"].emplace_back( static_cast<value_type>( cell_idx++ ));

            size_t point_idx { };
            for ( const auto& vertex_idx : face.vertex_index )
            {
                const auto& vertex = mesh.vertices[vertex_idx];
                points.emplace_back( static_cast<value_type>(vertex.p.x));
                points.emplace_back( static_cast<value_type>(vertex.p.y));
                points.emplace_back( static_cast<value_type>(vertex.p.z));

                point_datas["log_idx"].emplace_back( static_cast<value_type>( idx_ ));
                point_datas["logger_idx"].emplace_back( static_cast<value_type>( logger_idx_ ));
                point_datas["point_idx"].emplace_back( static_cast<value_type>( point_idx++ ));
            }
        }
        auto connectivity = getConnectivity( mesh.faces.size() * 3 );
        auto offsets = getOffsets( connectivity.size(), 3 );
        auto types = getCellTypes( offsets.size(), static_cast<long>(CellType::TRIANGLE));
        vtu11::Vtu11UnstructuredMesh mesh_partition { points, connectivity, offsets, types };
        writePartition( mesh_partition, point_datas, cell_datas );
    };

    template<typename... VDI>
    constexpr void log(const st_edges_viewable auto& st_edges, const int layer_idx, VDI... visual_data_infos)
    {
        updateDataInfos( CellVisualDataInfo { "log_idx" } );
        updateDataInfos( CellVisualDataInfo { "layer_idx" } );
        updateDataInfos( CellVisualDataInfo { "logger_idx" } );
        updateDataInfos( CellVisualDataInfo { "cell_idx" } );
        updateDataInfos( PointVisualDataInfo { "log_idx" } );
        updateDataInfos( PointVisualDataInfo { "layer_idx" } );
        updateDataInfos( PointVisualDataInfo { "logger_idx" } );
        updateDataInfos( PointVisualDataInfo { "point_idx" } );
        ( updateDataInfos( visual_data_infos ), ...);

        std::vector<value_type> points { };
        auto cell_datas = cell_dataset_info_ | ranges::views::transform( [](const auto& dsi) { return std::make_pair( std::get<0>( dsi ), vtu11::DataSetData { } ); } ) | ranges::to<std::unordered_map<std::string, vtu11::DataSetData>>;
        auto point_datas = point_dataset_info_ | ranges::views::transform( [](const auto& dsi) { return std::make_pair( std::get<0>( dsi ), vtu11::DataSetData { } ); } ) | ranges::to<std::unordered_map<std::string, vtu11::DataSetData>>;
        size_t cell_idx { };
        for ( const auto& st_edge : st_edges )
        {
            // log cell data
            ([ & ] {
              if constexpr ( visual_data_infos.dataset_type == vtu11::DataSetType::CellData )
              {
                  cell_datas[visual_data_infos.name].emplace_back( static_cast<value_type>( std::invoke( visual_data_infos.projection, st_edge )));
              }
            }(), ...); // variadic arguments to the log call
            cell_datas["log_idx"].emplace_back( static_cast<value_type>( idx_ ));
            cell_datas["layer_idx"].emplace_back( static_cast<value_type>( layer_idx ));
            cell_datas["logger_idx"].emplace_back( static_cast<value_type>( logger_idx_ ));
            cell_datas["cell_idx"].emplace_back( static_cast<value_type>( cell_idx++ ));

            size_t point_idx { };
            for ( const auto& node : { st_edge.from, st_edge.to } )
            {
                // log node data
                ([ & ] {
                  if constexpr ( visual_data_infos.dataset_type == vtu11::DataSetType::PointData )
                  {
                      point_datas[visual_data_infos.name].emplace_back( static_cast<value_type>( std::invoke( visual_data_infos.projection, * node )));
                  }
                }(), ...); // variadic arguments to the log call
                point_datas["log_idx"].emplace_back( static_cast<value_type>( idx_ ));
                point_datas["layer_idx"].emplace_back( static_cast<value_type>( layer_idx ));
                point_datas["logger_idx"].emplace_back( static_cast<value_type>( logger_idx_ ));
                point_datas["point_idx"].emplace_back( static_cast<value_type>( point_idx++ ));

                points.emplace_back( static_cast<value_type>(node->p.X));
                points.emplace_back( static_cast<value_type>(node->p.Y));
                points.emplace_back( layer_map_->at( layer_idx ));
            }
        }
        auto connectivity = getConnectivity( points.size() / 3 );
        auto offsets = getOffsets( connectivity.size(), 2 );
        auto types = getCellTypes( offsets.size(), static_cast<long>(CellType::LINE));
        vtu11::Vtu11UnstructuredMesh mesh_partition { points, connectivity, offsets, types };

        writePartition( mesh_partition, point_datas, cell_datas );
    };

private:
    std::mutex mutex_;
    std::string id_ { };
    size_t logger_idx_ { };
    size_t idx_ { };
    SectionType section_type_ { };
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

    void writePartition(vtu11::Vtu11UnstructuredMesh& mesh_partition, const auto& point_data, const auto& cell_data)
    {
        const std::scoped_lock lock { mutex_ };
        const auto idx = idx_++;
        spdlog::info( "Visual Debugger: <{}>-<{}> writing partition {}", id_, logger_idx_, idx );
        auto dataset_infos = ranges::actions::sort( ranges::views::concat( point_dataset_info_, cell_dataset_info_ ) | ranges::to_vector );
        std::vector<vtu11::DataSetData> data { };
        auto dataset_data = ranges::views::concat( point_data, cell_data ) | ranges::to<std::unordered_map<std::string, vtu11::DataSetData>>;
        for ( const auto& [ name, _, __ ] : dataset_infos )
        {
            data.push_back( dataset_data[name] );
        }
        if ( !dataset_infos.empty())
        {
            spdlog::debug( "Visual Debugger: <{}>-<{}> logging: {}", id_, logger_idx_, dataset_infos | ranges::views::transform( [](const auto& dsi) { return std::get<0>( dsi ); } ));
        }
        vtu11::writePartition( vtu_path_.string(), id_, mesh_partition,dataset_infos, data, idx, "rawbinarycompressed" );
//        vtu11::writePartition( vtu_path_.string(), id_, mesh_partition, dataset_infos, data, idx, "ascii" );
        vtu11::writePVtu( vtu_path_.string(), id_, dataset_infos, idx_ );  // Make sure it is up to data
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