/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef FUZZY_WALLS_H
#define FUZZY_WALLS_H

#include "sliceDataStorage.h"
#include "PolygonFlowAdjuster.h"

namespace cura {

class FuzzyWalls : public PolygonFlowAdjuster
{
public:
    struct Settings
    {
        coord_t max_amplitude;
        coord_t avg_dist_between_points;
        ColourUsage color_usage;
        coord_t min_dist_between_points;
        coord_t range_random_point_dist;
        Settings(const SettingsBaseVirtual* settings_base)
        : max_amplitude(settings_base->getSettingInMicrons("magic_fuzzy_skin_thickness"))
        , avg_dist_between_points(settings_base->getSettingInMicrons("magic_fuzzy_skin_point_dist"))
        , color_usage(settings_base->getSettingAsColourUsage("fuzz_map_texture_color"))
        , min_dist_between_points(avg_dist_between_points * 3 / 4) // hardcoded: the point distance may vary between 3/4 and 5/4 the supplied value
        , range_random_point_dist(avg_dist_between_points / 2)
        {
        }
    };
    FuzzyWalls(const SliceMeshStorage& mesh);
    Polygons makeFuzzy(const SliceMeshStorage& mesh, const unsigned int layer_nr, const Polygons& in);
    float getFlow(const Polygons& from, unsigned int poly_idx, unsigned int from_point_idx, unsigned int to_point_idx);
protected:
    struct CarryOver
    {
        coord_t dist_left_over;
        float offset_random; // [-1,1]
        float next_offset_random; // [-1,1]
        coord_t step_size;
        Point p0p1_perp;
    };
    Settings settings;
    std::function<coord_t (const unsigned int, const Point)> getAmplitude;

    std::vector<std::vector<float>> flows; //!< The flow per segment per polygon in the input

    void makeCornerFuzzy(const unsigned int layer_nr, const Point p0, const Point p1, const Point p2, const CarryOver carry_over, PolygonRef result);
    void makeSegmentFuzzy(const unsigned int layer_nr, const Point p0, const Point p1, PolygonRef result, CarryOver& carry_over);
};

}//namespace cura

#endif//FUZZY_WALLS_H
