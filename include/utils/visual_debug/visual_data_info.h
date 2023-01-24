// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INCLUDE_UTILS_VISUAL_DEBUG_VISUAL_DATA_INFO_H
#define INCLUDE_UTILS_VISUAL_DEBUG_VISUAL_DATA_INFO_H

#include <functional>

#include <vtu11/vtu11.hpp>

namespace cura::debug
{

template<class T, class Proj = std::identity, vtu11::DataSetType DST = vtu11::DataSetType::CellData, size_t Nm = 1>
struct VisualDataInfo
{
    std::string name { };
    static constexpr vtu11::DataSetType dataset_type { DST };
    Proj projection { };

    [[nodiscard]] explicit operator vtu11::DataSetInfo() const
    {
        return { name, DST, Nm };
    }

    [[nodiscard]] constexpr bool operator==(const vtu11::DataSetInfo& other) const noexcept
    {
        return name == std::get<0>( other ) && std::get<1>( other ) == DST;
    };

    [[nodiscard]] constexpr bool operator!=(const vtu11::DataSetInfo& other) const noexcept
    {
        return !* this == other;
    }

    [[nodiscard]] constexpr auto operator<=>(const VisualDataInfo& other) const noexcept = default;
};

template<class Proj = std::identity, size_t Nm = 1>
class CellVisualDataInfo : public VisualDataInfo<CellVisualDataInfo<Proj, Nm>, Proj, vtu11::DataSetType::CellData, Nm>
{
};

template<class Proj = std::identity, size_t Nm = 1>
class PointVisualDataInfo : public VisualDataInfo<PointVisualDataInfo<Proj, Nm>, Proj, vtu11::DataSetType::PointData, Nm>
{
};

// CTAD

template<class Proj>
CellVisualDataInfo(const std::string& name, Proj&& projection) -> CellVisualDataInfo<Proj, 1UL>;

template<class Proj>
PointVisualDataInfo(const std::string& name, Proj&& projection) -> PointVisualDataInfo<Proj, 1UL>;
} // namespace cura::debug

#endif //INCLUDE_UTILS_VISUAL_DEBUG_VISUAL_DATA_INFO_H
