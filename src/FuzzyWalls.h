/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef FUZZY_WALLS_H
#define FUZZY_WALLS_H

#include "sliceDataStorage.h"
#include "PolygonFlowAdjuster.h"

namespace cura {

/*!
 * Class for making walls fuzzy.
 * 
 * Points are introduced along each segment of the input polygon with a randomized offset orthogonal to the original segment.
 * 
 * The flows are adjusted such that the output polygon uses the same amount of filament.
 * 
 * The randomized offset occurs within a static amplitude or
 * within an amplitude defined by the texture color of the textured model if fuzz_map_enabled.
 * 
 */
class FuzzyWalls : public PolygonFlowAdjuster
{
public:
    /*!
     * The algorithm parameters
     */
    struct Settings
    {
        coord_t max_amplitude; //!< The maximum inward or outward offset of an introduced point from the original segment in the inpit polygon.
        coord_t avg_dist_between_points; //!< The average distance between two sample points on the input polygon which are to be offsetted.
        ColourUsage color_usage; //!< How colors of the textured model are translated into offsets.
        coord_t min_dist_between_points; //!< The minimum distance between two sample points on the input polygon which are to be offsetted.
        coord_t range_random_point_dist; //!< The difference between the minimum and maximum distance between two sample points on the input polygon which are to be offsetted.
        Settings(const SettingsBaseVirtual* settings_base)
        : max_amplitude(settings_base->getSettingInMicrons("magic_fuzzy_skin_thickness"))
        , avg_dist_between_points(settings_base->getSettingInMicrons("magic_fuzzy_skin_point_dist"))
        , color_usage(settings_base->getSettingAsColourUsage("fuzz_map_texture_color"))
        , min_dist_between_points(avg_dist_between_points * 3 / 4) // hardcoded: the point distance may vary between 3/4 and 5/4 the supplied value
        , range_random_point_dist(avg_dist_between_points / 2)
        {
        }
    };
    /*!
     * Algorithm constructor which initialized with the algorithm settings obtained from the user settings specified in the \p mesh
     * 
     * \param mesh Where to get the settigns from and the texture colors.
     */
    FuzzyWalls(const SliceMeshStorage& mesh);

    /*!
     * Turn the \p in polygon fuzzy and return it.
     * 
     * \param layer_nr The layer at which to obtain texture colors
     * \param in The input polygon
     * \return a fuzzified polygon
     */
    Polygons makeFuzzy(const unsigned int layer_nr, const Polygons& in);

    /*!
     * Get the flow associated with a given polygon segment in the fuzzified walls.
     * 
     * \param from The fuzzy walls
     * \param poly_idx The index of the polygon where to get the flow for
     * \param from_point_idx The index into the above polygon of the start of the segment for which to get the flow.
     * \param to_point_idx The index into the above polygon of the end of the segment for which to get the flow.
     * \return the flow
     */
    float getFlow(const Polygons& from, unsigned int poly_idx, unsigned int from_point_idx, unsigned int to_point_idx);
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
        float offset_random; //!< [-1,1] The random number used to determine the offset within the amplitude of the previous sample point.
        float next_offset_random; // [-1,1] The random number used to determine the offset within the amplitude of the next sample point.
        coord_t step_size; //!< The step size of the current step from the previous sample point to the next.
        Point p0p1_perp; //!< The input polygon segment normal of the previous input point to the next.
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

    std::vector<std::vector<float>> flows; //!< The flow per segment per polygon in the input

    void makeCornerFuzzy(const unsigned int layer_nr, const Point p0, const Point p1, const Point p2, const CarryOver carry_over, PolygonRef result);
    void makeSegmentFuzzy(const unsigned int layer_nr, const Point p0, const Point p1, PolygonRef result, CarryOver& carry_over);
};

}//namespace cura

#endif//FUZZY_WALLS_H
