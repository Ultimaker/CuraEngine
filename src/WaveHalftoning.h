/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef WAVE_HALFTONING_H
#define WAVE_HALFTONING_H

#include "sliceDataStorage.h"

namespace cura {

/*!
 * Class for making walls halftoned using an amplitude modulated triangle wave.
 * 
 * An amplitude modulated wave-form offset is applied to the outlines of the input polygon (generally the outer wall).
 * The wave-form is a triangle wave: \/\/\/\/\/ for which the amplitudes for each location depend on the texture colors.
 * 
 * Points are introduced along each segment of the input polygon with an offset orthogonal to the original segment.
 * 
 * This offset is based on the texture color close to the offsetted point
 * 
 * 
 * TODO
 * This file is adopted from FuzzyWalls.
 * These files may need to be merged into one in order to reduce the amount of code duplication.
 */
class WaveHalftoning
{
public:
    /*!
     * The algorithm parameters
     */
    struct Settings
    {
        coord_t max_amplitude; //!< The maximum inward or outward offset of an introduced point from the original segment in the inpit polygon.
        coord_t min_amplitude; //!< The minmum inward or outward offset of an introduced point from the original segment in the inpit polygon.
        coord_t dist_between_points; //!< The distance between two sample points on the input polygon which are to be offsetted.
        ColourUsage color_usage; //!< How colors of the textured model are translated into offsets.
        bool inverse_color_usage; //!< Whether to align higher colors with inward offsets vs outward
        Settings(const SettingsBaseVirtual* settings_base)
        : max_amplitude(settings_base->getSettingInMicrons("wave_halftoning_amplitude_max"))
        , min_amplitude(settings_base->getSettingInMicrons("wave_halftoning_amplitude_min"))
        , dist_between_points(settings_base->getSettingInMicrons("wave_halftoning_wave_length") / 2)
        , color_usage(settings_base->getSettingAsColourUsage("wave_halftoning_texture_color"))
        , inverse_color_usage(!settings_base->getSettingBoolean("wave_halftoning_is_white"))
        {
        }
    };
    /*!
     * Algorithm constructor which initialized with the algorithm settings obtained from the user settings specified in the \p mesh
     * 
     * \param mesh Where to get the settigns from and the texture colors.
     */
    WaveHalftoning(const SliceMeshStorage& mesh);

    /*!
     * Apply the halftoning to the \p in polygon and return the resulting polygon.
     * 
     * \param layer_nr The layer at which to obtain texture colors
     * \param in The input polygon
     * \return a halftoned polygon
     */
    Polygons makeHalftoned(const unsigned int layer_nr, const Polygons& in);

protected:
    /*!
     * Algorithm parameters to carry over from fuzzifying one sample point to fuzzifying the next.
     * 
     * For some we do this because recomputing is more expensive.
     * For others they cannot be recomputed.
     */
    struct CarryOver
    {
        coord_t dist_left_over; //!< The distance on the next segment to skip before getting the next sample point.
        float last_amplitude; //!< [-1,1] The offset within the amplitude of the previous sample point.
        Point p0p1_perp; //!< The input polygon segment normal of the previous input point to the next.
        int direction; //!< Whether to offset outward or inward on the next point.
    };
    /*!
     * The algorithm parameters with which to run \ref FuzzyWalls::makeFuzzy
     */
    Settings settings;

    /*!
     * Function to get the offsetting amplitude at a given location on the walls.
     * 
     * \ref layer_nr The layer number at which we are processing fuzzy walls.
     * \ref location The location on the input polygon.
     * \ref return The amplitude within which to offset sample points on the input polygon.
     */
    std::function<coord_t (const unsigned int layer_nr, const Point location)> getAmplitude;

    void makeCornerHalftoned(const unsigned int layer_nr, const Point p0, const Point p1, const Point p2, const CarryOver carry_over, PolygonRef result);
    void makeSegmentHalftoned(const unsigned int layer_nr, const Point p0, const Point p1, PolygonRef result, CarryOver& carry_over);
};

}//namespace cura

#endif//WAVE_HALFTONING_H
