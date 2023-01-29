// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INCLUDE_UTILS_VISUAL_DEBUG_VISUAL_LOGGER_H
#define INCLUDE_UTILS_VISUAL_DEBUG_VISUAL_LOGGER_H

#include <memory>
#include <unordered_map>
#ifdef VISUAL_DEBUG
#include <any>
#include <filesystem>
#include <functional>
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
#endif

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
    using mapped_data_t = std::unordered_map<std::string_view, vtu11::DataSetData>;

    constexpr VisualLogger() noexcept = default;

    VisualLogger(const std::string& id, const size_t logger_idx, const std::filesystem::path& vtu_path) : enabled_ { true }, id_ { id }, logger_idx_ { logger_idx }, vtu_path_ { vtu_path }
    {
        const std::scoped_lock lock { mutex_ };
        spdlog::info( "Visual Debugger: Initializing Logger <{}>-<{}> stored in {}", id_, logger_idx_, vtu_path_.string());
        vtu11::writePVtu( vtu_path_.string(), id_, { }, 1 );
    };

    void setValue(shared_layer_map_t& layer_map)
    {
        layer_map_ = layer_map;
    }

    template<typename... Args>
    constexpr void log(Args&& ... args)
    {
        if ( enabled_ )
        {
            for ( auto general_vdi : general_vdis_ )
            {
                updateDataInfos( CellVisualDataInfo { general_vdi } );
                updateDataInfos( PointVisualDataInfo { general_vdi } );
            }
            log_( std::forward<Args>( args )... );
        }
    };

private:
    static constexpr std::array<std::string_view, 4> general_vdis_ { "log_idx", "logger_idx", "idx", "section_type" };
    bool enabled_ { false };
    std::mutex mutex_;
    std::string id_ { };
    size_t logger_idx_ { };
    size_t idx_ { };
    std::filesystem::path vtu_path_ { };
    shared_layer_map_t layer_map_ { };
    std::vector<vtu11::DataSetInfo> cell_dataset_info_ { };
    std::vector<vtu11::DataSetInfo> point_dataset_info_ { };

    constexpr void log_(const layer_viewable auto& layers, SectionType section_type)
    {
        for ( const auto& [ layer_idx, layer ] : layers | ranges::views::enumerate )
        {
            log( layer.polygons, layer_idx, section_type );
        }
    };

    template<typename... VDI>
    constexpr void log_(const polygons auto& polys, const int layer_idx, SectionType section_type, VDI... visual_data_infos)
    {
        updateDataInfos( CellVisualDataInfo { "layer_idx" } );
        updateDataInfos( PointVisualDataInfo { "layer_idx" } );
        ( updateDataInfos( visual_data_infos ), ...);

        std::vector<value_type> points { };
        std::vector<vtu11::VtkIndexType> offsets { 0 };
        auto cell_datas = cell_dataset_info_ | ranges::views::transform( [](const auto& dsi) { return std::make_pair( std::get<0>( dsi ), vtu11::DataSetData { } ); } ) | ranges::to<mapped_data_t>;
        auto point_datas = point_dataset_info_ | ranges::views::transform( [](const auto& dsi) { return std::make_pair( std::get<0>( dsi ), vtu11::DataSetData { } ); } ) | ranges::to<mapped_data_t>;

        size_t cell_idx { };
        for ( const auto& cell : polys )
        {
            offsets.push_back( offsets.back() + cell.size());
            // log cell data
            ([ & ] {
              if constexpr ( visual_data_infos.dataset_type == vtu11::DataSetType::CellData )
              {
                  updateData( cell_datas, visual_data_infos.name, std::invoke( visual_data_infos.projection, cell ));
              }
            }(), ...); // variadic arguments to the log call
            updateData( cell_datas, "idx", cell_idx++, "log_idx", idx_, "layer_idx", layer_idx, "logger_idx", logger_idx_, "section_type", static_cast<int>(section_type));

            size_t point_idx { };
            for ( const auto& point : cell )
            {
                // log node data
                ([ & ] {
                  if constexpr ( visual_data_infos.dataset_type == vtu11::DataSetType::PointData )
                  {
                      updateData( point_datas, visual_data_infos.name, std::invoke( visual_data_infos.projection, point ));
                  }
                }(), ...); // variadic arguments to the log call
                updateData( point_datas, "idx", cell_idx++, "log_idx", idx_, "layer_idx", layer_idx, "logger_idx", logger_idx_, "section_type", static_cast<int>(section_type));
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

    constexpr void log_(const mesh auto& mesh, SectionType section_type)
    {
        // FIXME: add the last face as well
        auto cell_datas = cell_dataset_info_ | ranges::views::transform( [](const auto& dsi) { return std::make_pair( std::get<0>( dsi ), vtu11::DataSetData { } ); } ) | ranges::to<mapped_data_t>;
        auto point_datas = point_dataset_info_ | ranges::views::transform( [](const auto& dsi) { return std::make_pair( std::get<0>( dsi ), vtu11::DataSetData { } ); } ) | ranges::to<mapped_data_t>;

        std::vector<value_type> points { };
        size_t cell_idx { };
        for ( const auto& cell : mesh.faces )
        {
            updateData( cell_datas, "idx", cell_idx++, "log_idx", idx_, "logger_idx", logger_idx_, "section_type", static_cast<int>(section_type));
            size_t point_idx { };
            for ( const auto& vertex_idx : cell.vertex_index )
            {
                updateData( point_datas, "idx", point_idx++, "log_idx", idx_, "layer_idx", logger_idx_, "section_type", static_cast<int>(section_type));
                const auto& vertex = mesh.vertices[vertex_idx];
                points.emplace_back( static_cast<value_type>(vertex.p.x));
                points.emplace_back( static_cast<value_type>(vertex.p.y));
                points.emplace_back( static_cast<value_type>(vertex.p.z));
            }
        }
        auto connectivity = getConnectivity( mesh.faces.size() * 3 );
        auto offsets = getOffsets( connectivity.size(), 3 );
        auto types = getCellTypes( offsets.size(), static_cast<long>(CellType::TRIANGLE));
        vtu11::Vtu11UnstructuredMesh mesh_partition { points, connectivity, offsets, types };
        writePartition( mesh_partition, point_datas, cell_datas );
    };

    template<typename... VDI>
    constexpr void log_(const st_edges_viewable auto& st_edges, const int layer_idx, SectionType section_type, VDI... visual_data_infos)
    {
        updateDataInfos( CellVisualDataInfo { "layer_idx" } );
        updateDataInfos( PointVisualDataInfo { "layer_idx" } );
        ( updateDataInfos( visual_data_infos ), ...);

        std::vector<value_type> points { };
        auto cell_datas = cell_dataset_info_ | ranges::views::transform( [](const auto& dsi) { return std::make_pair( std::get<0>( dsi ), vtu11::DataSetData { } ); } ) | ranges::to<mapped_data_t>;
        auto point_datas = point_dataset_info_ | ranges::views::transform( [](const auto& dsi) { return std::make_pair( std::get<0>( dsi ), vtu11::DataSetData { } ); } ) | ranges::to<mapped_data_t>;
        size_t cell_idx { };
        for ( const auto& cell : st_edges )
        {
            // log cell data
            ([ & ] {
              if constexpr ( visual_data_infos.dataset_type == vtu11::DataSetType::CellData )
              {
                  updateData( cell_datas, visual_data_infos.name, std::invoke( visual_data_infos.projection, cell ));
              }
            }(), ...); // variadic arguments to the log call
            updateData( cell_datas, "idx", cell_idx++, "log_idx", idx_, "layer_idx", layer_idx, "logger_idx", logger_idx_, "section_type", static_cast<int>(section_type));

            size_t point_idx { };
            for ( const auto& point : { cell.from, cell.to } )
            {
                // log node data
                ([ & ] {
                  if constexpr ( visual_data_infos.dataset_type == vtu11::DataSetType::PointData )
                  {
                      updateData( point_datas, visual_data_infos.name, std::invoke( visual_data_infos.projection, * point ));
                  }
                }(), ...); // variadic arguments to the log call
                updateData( point_datas, "idx", point_idx++, "log_idx", idx_, "layer_idx", layer_idx, "logger_idx", logger_idx_, "section_type", static_cast<int>(section_type));

                points.emplace_back( static_cast<value_type>(point->p.X));
                points.emplace_back( static_cast<value_type>(point->p.Y));
                points.emplace_back( layer_map_->at( layer_idx ));
            }
        }
        auto connectivity = getConnectivity( points.size() / 3 );
        auto offsets = getOffsets( connectivity.size(), 2 );
        auto types = getCellTypes( offsets.size(), static_cast<long>(CellType::LINE));
        vtu11::Vtu11UnstructuredMesh mesh_partition { points, connectivity, offsets, types };

        writePartition( mesh_partition, point_datas, cell_datas );
    };

    template<typename... VDI>
    constexpr void log_(const toolpaths auto& toolpaths, const int layer_idx, SectionType section_type, VDI... visual_data_infos)
    {
        for ( const auto& toolpath : toolpaths )
        {
            log_( toolpath, layer_idx, section_type, visual_data_infos... );
        }
    };

    template<typename... VDI>
    constexpr void log_(const toolpath auto& toolpath, const int layer_idx, SectionType section_type, VDI... visual_data_infos)
    {
        updateDataInfos( CellVisualDataInfo { "layer_idx" } );
        updateDataInfos( PointVisualDataInfo { "layer_idx" } );
        ( updateDataInfos( visual_data_infos ), ...);

        std::vector<value_type> points { };
        std::vector<vtu11::VtkIndexType> offsets { 0 };
        auto cell_datas = cell_dataset_info_ | ranges::views::transform( [](const auto& dsi) { return std::make_pair( std::get<0>( dsi ), vtu11::DataSetData { } ); } ) | ranges::to<mapped_data_t>;
        auto point_datas = point_dataset_info_ | ranges::views::transform( [](const auto& dsi) { return std::make_pair( std::get<0>( dsi ), vtu11::DataSetData { } ); } ) | ranges::to<mapped_data_t>;
        size_t cell_idx { };
        for ( const auto& cell : toolpath )
        {
            offsets.push_back( offsets.back() + cell.size());
            // log cell data
            ([ & ] {
              if constexpr ( visual_data_infos.dataset_type == vtu11::DataSetType::CellData )
              {
                  updateData( cell_datas, visual_data_infos.name, std::invoke( visual_data_infos.projection, cell ));
              }
            }(), ...); // variadic arguments to the log call
            updateData( cell_datas, "idx", cell_idx++, "log_idx", idx_, "layer_idx", layer_idx, "logger_idx", logger_idx_, "section_type", static_cast<int>(section_type));

            size_t point_idx { };
            for ( const auto& point : cell.junctions )
            {
                // log node data
                ([ & ] {
                  if constexpr ( visual_data_infos.dataset_type == vtu11::DataSetType::PointData )
                  {
                      updateData( point_datas, visual_data_infos.name, std::invoke( visual_data_infos.projection, point ));
                  }
                }(), ...); // variadic arguments to the log call
                updateData( point_datas, "idx", point_idx++, "log_idx", idx_, "layer_idx", layer_idx, "logger_idx", logger_idx_, "section_type", static_cast<int>(section_type));

                points.emplace_back( static_cast<value_type>(point.p.X));
                points.emplace_back( static_cast<value_type>(point.p.Y));
                points.emplace_back( layer_map_->at( layer_idx ));
            }
        }
        auto connectivity = getConnectivity( points.size() / 3 );
        auto types = getCellTypes( offsets.size() - 1, static_cast<long>(CellType::POLY_LINE));
        vtu11::Vtu11UnstructuredMesh mesh_partition { points, connectivity, offsets | ranges::views::drop( 1 ) | ranges::to_vector, types };

        writePartition( mesh_partition, point_datas, cell_datas );
    };

    template<class Property, class Data, class... Datas>
    inline constexpr void updateData(auto& container, const Property& property, const Data& data, Datas... datas)
    {
        container[property].emplace_back( static_cast<value_type>( data ));
        if constexpr ( sizeof...( datas ) > 1 )
        {
            updateData( container, datas... );
        }
    }

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
        auto dataset_data = ranges::views::concat( point_data, cell_data ) | ranges::to<mapped_data_t>;
        for ( const auto& [ name, _, __ ] : dataset_infos )
        {
            data.push_back( dataset_data[name] );
        }
        if ( !dataset_infos.empty())
        {
            spdlog::debug( "Visual Debugger: <{}>-<{}> logging: {}", id_, logger_idx_, dataset_infos | ranges::views::transform( [](const auto& dsi) { return std::get<0>( dsi ); } ));
        }
        vtu11::writePartition( vtu_path_.string(), id_, mesh_partition, dataset_infos, data, idx, "rawbinarycompressed" );
        vtu11::writePVtu( vtu_path_.string(), id_, dataset_infos, idx_ );  // Make sure it is up to date
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