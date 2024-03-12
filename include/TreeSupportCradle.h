//
// Created by Thomas on 04/03/2024.
//

#ifndef CURAENGINE_TREESUPPORTCRADLE_H
#define CURAENGINE_TREESUPPORTCRADLE_H
#include "TreeSupportElement.h"
#include "TreeSupportEnums.h"
#include "utils/Coord_t.h"
#include "utils/polygon.h"
#include <spdlog/spdlog.h>

namespace cura
{

struct TreeSupportCradleLine
{
    //required to shrink a vector using resize
    TreeSupportCradleLine()
    {
        spdlog::error("Dummy TreeSupportCradleLine constructor called");
    }

    TreeSupportCradleLine(Polygon line, LayerIndex layer_idx)
        : line(line)
        , layer_idx(layer_idx)
    {
    }
    Polygons area;
    Polygon line;
    Polygon removed_line;
    std::vector<TreeSupportElement*> tips;
    LayerIndex layer_idx;
    bool is_base = false;

    void addLineToRemoved(Polygon& line_to_add)
    {
        if (removed_line.empty())
        {
            removed_line.add(line_to_add.front());
            removed_line.add(line_to_add.back());
        }
        else
        {
            if (vSize2(removed_line.front() - removed_line.back()) < vSize2(line_to_add.front() - removed_line.back()))
            {
                removed_line.front() = line_to_add.front();
            }

            if (vSize2(removed_line.front() - removed_line.back()) < vSize2(removed_line.front() - line_to_add.back()))
            {
                removed_line.back() = line_to_add.back();
            }
        }
    }
};

struct TreeSupportCradleOverhangInformation
{
    TreeSupportCradleOverhangInformation()
        : TreeSupportCradleOverhangInformation(Polygons(), false)
    {
    }
    TreeSupportCradleOverhangInformation(Polygons area, bool is_line, bool is_roof = false, LayerIndex line_layer_idx = -1, int32_t line_idx = -1)
        : area(area)
        , is_line(is_line)
        , is_roof(is_roof)
        , line_information(std::pair<LayerIndex, int32_t>(line_layer_idx, line_idx))
    {

    }
    Polygons area;
    bool is_line;
    std::pair<LayerIndex, int32_t> line_information = std::pair<LayerIndex, int32_t>(-1, -1);
    bool is_roof;
};

struct TreeSupportCradle
{
    std::vector<std::deque<TreeSupportCradleLine>> lines;
    bool is_roof;
    LayerIndex layer_idx;
    std::vector<Polygons> base_below;
    Point center;
    size_t shadow_idx;
    std::unordered_map<LayerIndex, std::vector<TreeSupportCradleOverhangInformation>> overhang;

    size_t config_cradle_layers_min;
    coord_t config_cradle_length_min;

    TreeSupportCradle(LayerIndex layer_idx, Point center, size_t shadow_idx, bool roof, size_t cradle_layers_min, coord_t cradle_length_min)
        : layer_idx(layer_idx)
        , center(center)
        , shadow_idx(shadow_idx)
        , is_roof(roof)
        , config_cradle_layers_min(cradle_layers_min)
        , config_cradle_length_min(cradle_length_min)
    {
    }


    std::optional<TreeSupportCradleLine*> getCradleLineOfIndex(LayerIndex requested_layer_idx, size_t cradle_line_idx)
    {
        if (cradle_line_idx < lines.size())
        {
            for (size_t height_idx = 0; height_idx < lines[cradle_line_idx].size(); height_idx++)
            {
                if (lines[cradle_line_idx][height_idx].layer_idx == requested_layer_idx)
                {
                    return &lines[cradle_line_idx][height_idx];
                }
            }
        }
        return {};
    }

    void verifyLines()
    {
        for (size_t line_idx = 0; line_idx < lines.size(); line_idx++)
        {
            if (lines[line_idx].size() < config_cradle_layers_min)
            {
                lines[line_idx].clear();
                continue;
            }
            LayerIndex previous_layer_idx;

            for (size_t up_idx = 0; up_idx < lines[line_idx].size(); up_idx++)
            {
                if(!lines[line_idx][up_idx].is_base)
                {
                    previous_layer_idx = lines[line_idx][up_idx].layer_idx;
                    break;
                }
            }
            for (size_t up_idx = 1; up_idx < lines[line_idx].size(); up_idx++)
            {
                if(!lines[line_idx][up_idx].is_base)
                {
                    if (lines[line_idx][up_idx].layer_idx > previous_layer_idx + up_idx ||
                        lines[line_idx][up_idx].line.size()<2 ||
                        lines[line_idx][up_idx].line.polylineLength() < config_cradle_length_min)
                    {
                        if (up_idx <= config_cradle_layers_min)
                        {

                            spdlog::info("Removing cradle line of cradle on layer {} line at {}. Invalid line was on layer {}",layer_idx,line_idx,lines[line_idx][up_idx].layer_idx);
                            lines[line_idx].clear();
                            break;
                        }
                        else
                        {
                            spdlog::info("Partially removing cradle line of cradle on layer {} line at {} at height {}",layer_idx,line_idx,up_idx);
                            lines[line_idx].resize(up_idx-1);
                            break;
                        }
                    }
                }
            }
        }
    }
};

struct CradlePresenceInformation
{
    CradlePresenceInformation(TreeSupportCradle* cradle,LayerIndex layer_idx,size_t line_idx):
    cradle(cradle),
    layer_idx(layer_idx),
    line_idx(line_idx)
    {}
    TreeSupportCradle* cradle;
    LayerIndex layer_idx;
    size_t line_idx;

    TreeSupportCradleLine* getCradleLine()
    {
        return cradle->getCradleLineOfIndex(layer_idx,line_idx).value();
    }

    bool cradleLineExists()
    {
        return cradle->getCradleLineOfIndex(layer_idx,line_idx).has_value();
    }

};


}

#endif // CURAENGINE_TREESUPPORTCRADLE_H
