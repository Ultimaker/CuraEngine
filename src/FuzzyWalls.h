/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef FUZZY_WALLS_H
#define FUZZY_WALLS_H

#include "sliceDataStorage.h"

namespace cura {

class FuzzyWalls
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
    Polygons makeFuzzy(const SliceMeshStorage& mesh, const unsigned int layer_nr, const Polygons& in) const;
protected:
    Settings settings;
    std::function<coord_t (const unsigned int, const Point)> getAmplitude;
};

}//namespace cura

#endif//FUZZY_WALLS_H
