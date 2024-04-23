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
//todo rename file as now general TreeSupportTipDataStructures
struct TreeSupportCradle;

struct OverhangInformation
{

    OverhangInformation(Polygons overhang, bool roof):
        overhang_(overhang),
        is_roof_(roof),
        is_cradle_(false),
        cradle_layer_idx_(-1),
        cradle_line_idx_(-1),
        cradle_(nullptr)
    {

    }

    OverhangInformation(Polygons overhang, bool roof, TreeSupportCradle* cradle, int32_t cradle_layer_idx = -1,int32_t cradle_line_idx = -1):
        overhang_(overhang),
        is_roof_(roof),
        is_cradle_(true),
        cradle_layer_idx_(cradle_layer_idx),
        cradle_line_idx_(cradle_line_idx),
        cradle_(cradle)
    {

    }

    Polygons overhang_;
    bool is_roof_;
    bool is_cradle_;
    int32_t cradle_layer_idx_;
    int32_t cradle_line_idx_;
    TreeSupportCradle* cradle_;

    bool isCradleLine()
    {
        return is_cradle_ && cradle_line_idx_ >= 0 && cradle_layer_idx_>=0;
    }

};
struct TreeSupportCradleLine
{
    //required to shrink a vector using resize
    TreeSupportCradleLine()
    {
        spdlog::error("Dummy TreeSupportCradleLine constructor called");
    }

    TreeSupportCradleLine(Polygon line, LayerIndex layer_idx, bool is_roof)
        : line_(line)
        , layer_idx_(layer_idx)
        , is_roof_(is_roof)
    {
    }
    Polygons area_;
    Polygon line_;
    Polygon removed_line_;
    LayerIndex layer_idx_;
    bool is_base_ = false;
    bool is_roof_;

    void addLineToRemoved(Polygon& line_to_add)
    {
        if (removed_line_.empty())
        {
            removed_line_.add(line_to_add.front());
            removed_line_.add(line_to_add.back());
        }
        else
        {
            if (vSize2(removed_line_.front() - removed_line_.back()) < vSize2(line_to_add.front() - removed_line_.back()))
            {
                removed_line_.front() = line_to_add.front();
            }

            if (vSize2(removed_line_.front() - removed_line_.back()) < vSize2(removed_line_.front() - line_to_add.back()))
            {
                removed_line_.back() = line_to_add.back();
            }
        }
    }
};

struct TreeSupportCradle
{
    std::vector<std::deque<TreeSupportCradleLine>> lines_;
    bool is_roof_;
    LayerIndex layer_idx_;
    std::vector<Polygons> base_below_;
    std::vector<Point2LL> centers_;
    size_t shadow_idx_;
    std::unordered_map<LayerIndex, std::vector<OverhangInformation>> overhang_;

    size_t config_cradle_layers_min_;
    coord_t config_cradle_length_min_;
    size_t cradle_line_count_;


    TreeSupportCradle(LayerIndex layer_idx, Point2LL center, size_t shadow_idx, bool roof, size_t cradle_layers_min, coord_t cradle_length_min, size_t cradle_line_count)
        : layer_idx_(layer_idx)
        , centers_({center})
        , shadow_idx_(shadow_idx)
        , is_roof_(roof)
        , config_cradle_layers_min_(cradle_layers_min)
        , config_cradle_length_min_(cradle_length_min)
        ,cradle_line_count_(cradle_line_count)
    {
    }


    std::optional<TreeSupportCradleLine*> getCradleLineOfIndex(LayerIndex requested_layer_idx, size_t cradle_line_idx)
    {
        if (cradle_line_idx < lines_.size())
        {
            for (size_t height_idx = 0; height_idx < lines_[cradle_line_idx].size(); height_idx++)
            {
                if (lines_[cradle_line_idx][height_idx].layer_idx_ == requested_layer_idx)
                {
                    return &lines_[cradle_line_idx][height_idx];
                }
            }
        }
        return {};
    }

    Point2LL getCenter(LayerIndex layer_idx_req)
    {
        if(layer_idx_req<layer_idx_)
        {
            return centers_.front();
        }
        size_t index = layer_idx_req - layer_idx_;
        if(centers_.size()<=index)
        {
            return centers_.back();
        }
        return centers_[index];
    }

    size_t getIndexForLineEnd(Point2LL line_end, LayerIndex layer_idx_req)
    {
        Point2LL current_direction = line_end - getCenter(layer_idx_req);
        double angle = std::atan2(current_direction.Y, current_direction.X);
        size_t angle_idx = size_t(std::round(((angle+std::numbers::pi)/(2.0*std::numbers::pi)) * double(cradle_line_count_))) % cradle_line_count_;
        return angle_idx;
    }

    void verifyLines()
    {
        for (size_t line_idx = 0; line_idx < lines_.size(); line_idx++)
        {
            if (lines_[line_idx].size() < config_cradle_layers_min_)
            {
                lines_[line_idx].clear();
                continue;
            }
            LayerIndex previous_layer_idx;

            for (size_t up_idx = 0; up_idx < lines_[line_idx].size(); up_idx++)
            {
                if(!lines_[line_idx][up_idx].is_base_)
                {
                    previous_layer_idx = lines_[line_idx][up_idx].layer_idx_;
                    if (lines_[line_idx][up_idx].layer_idx_ > previous_layer_idx + up_idx ||
                        lines_[line_idx][up_idx].line_.size() < 2 ||
                        lines_[line_idx][up_idx].line_.polylineLength() < config_cradle_length_min_)
                    {
                        lines_[line_idx].clear();
                    }
                    break;
                }
            }
            for (size_t up_idx = 1; up_idx < lines_[line_idx].size(); up_idx++)
            {
                if(!lines_[line_idx][up_idx].is_base_)
                {
                    if (lines_[line_idx][up_idx].layer_idx_ > previous_layer_idx + up_idx ||
                        lines_[line_idx][up_idx].line_.size()<2 ||
                        lines_[line_idx][up_idx].line_.polylineLength() < config_cradle_length_min_)
                    {
                        if (up_idx <= config_cradle_layers_min_)
                        {
                            spdlog::debug("Removing cradle line of cradle on layer {} line at {}. Invalid line was on layer {}",layer_idx_,line_idx,lines_[line_idx][up_idx].layer_idx_);
                            lines_[line_idx].clear();
                            break;
                        }
                        else
                        {
                            spdlog::debug("Partially removing cradle line of cradle on layer {} line at {} at height {}",layer_idx_,line_idx,up_idx);
                            lines_[line_idx].resize(up_idx-1);
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
    cradle_(cradle),
    layer_idx_(layer_idx),
    line_idx_(line_idx)
    {}
    TreeSupportCradle* cradle_;
    LayerIndex layer_idx_;
    size_t line_idx_;

    TreeSupportCradleLine* getCradleLine()
    {
        return cradle_->getCradleLineOfIndex(layer_idx_,line_idx_).value();
    }

    bool cradleLineExists()
    {
        return cradle_->getCradleLineOfIndex(layer_idx_,line_idx_).has_value();
    }

};


}

#endif // CURAENGINE_TREESUPPORTCRADLE_H
