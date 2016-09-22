/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SKIRT_BRIM_H
#define SKIRT_BRIM_H

#include "sliceDataStorage.h"

namespace cura 
{

class SkirtBrim
{
public:
    /*!
     * Generate skirt or brim (depending on parameters).
     * 
     * When \p distance > 0 and \p count == 1 a skirt is generated, which has
     * slightly different configuration. Otherwise, a brim is generated.
     * 
     * \param storage Storage containing the parts at the first layer.
     * \param distance The distance of the first outset from the parts at the first
     * layer.
     * \param primary_line_count Number of outsets / brim lines of the primary extruder.
     * \param outside_only Whether to only generate a brim on the outside, rather than also in holes
     */
    static void generate(SliceDataStorage& storage, int distance, unsigned int primary_line_count, bool outside_only);

private:
    /*!
     * Get the reference outline of the first layer around which to generate the first brim/skirt line.
     * 
     * This function may change the support polygons in the first layer
     * in order to meet criteria for putting brim around the model as well as around the support.
     * 
     * \param storage Storage containing the parts at the first layer.
     * \param primary_line_count Number of outsets / brim lines of the primary extruder.
     * \param primary_extruder_skirt_brim_line_width Line widths of the initial skirt/brim lines
     * \param is_skirt Whether a skirt is being generated vs a brim
     * \param outside_only Whether to only generate a brim on the outside, rather than also in holes
     * \param[out] first_layer_outline The resulting reference polygons
     */
    static void getFirstLayerOutline(SliceDataStorage& storage, const unsigned int primary_line_count, const int primary_extruder_skirt_brim_line_width, const bool is_skirt, const bool outside_only, Polygons& first_layer_outline);

    /*!
     * Generate the skirt/brim lines around the model
     * 
     * \param storage Storage containing the parts at the first layer.
     * \param start_distance The distance of the first outset from the parts at the first
     * \param primary_line_count Number of outsets / brim lines of the primary extruder.
     * \param primary_extruder_skirt_brim_line_width Line widths of the initial skirt/brim lines
     * \param primary_extruder_minimal_length The minimal total length of the skirt/brim lines of the primary extruder
     * \param first_layer_outline The reference polygons from which to offset outward to generate skirt/brim lines
     * \param[out] skirt_brim_primary_extruder Where to store the resulting brim/skirt lines in
     * \return The offset of the last brim/skirt line from the reference polygon \p first_layer_outline
     */
    static int generatePrimarySkirtBrimLines(SliceDataStorage& storage, int start_distance, unsigned int primary_line_count, const int primary_extruder_skirt_brim_line_width, const int64_t primary_extruder_minimal_length, const Polygons& first_layer_outline, Polygons& skirt_brim_primary_extruder);
};
}//namespace cura

#endif //SKIRT_BRIM_H
