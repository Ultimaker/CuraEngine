// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INCLUDE_UTILS_VISUAL_DEBUG_VISUAL_DATA_INFO_H
#define INCLUDE_UTILS_VISUAL_DEBUG_VISUAL_DATA_INFO_H

#include <vtu11/vtu11.hpp>

namespace cura::debug
{
struct VisualDataInfo
{
    std::string name;
    vtu11::DataSetType dataset_type;
    std::size_t components;
    std::any projection;

    [[nodiscard]] vtu11::DataSetInfo getDataSetInfo()
    {
        return { name, dataset_type, components };
    };
};
} // namespace cura::debug

#endif //INCLUDE_UTILS_VISUAL_DEBUG_VISUAL_DATA_INFO_H
