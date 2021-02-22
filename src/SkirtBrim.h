//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SKIRT_BRIM_H
#define SKIRT_BRIM_H

#include "utils/Coord_t.h"

namespace cura 
{

class Polygons;
class SliceDataStorage;

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
     * \param first_layer_outline The outline to generate skirt or brim around.
     * \param distance The distance of the first outset from the parts at the first
     * layer.
     * \param primary_line_count Number of offsets / brim lines of the primary extruder.
     * \param set to false to force not doing brim generation for helper-structures (support and ooze/draft shields)
     */
//     static void generate(SliceDataStorage& storage, Polygons first_layer_outline, const coord_t distance, size_t primary_line_count, const bool allow_helpers = true);
    static void generate(SliceDataStorage& storage);
    
    /*!
     * \brief Get the reference outline of the first layer around which to
     * generate the first brim/skirt line.
     *
     * This function may change the support polygons in the first layer
     * in order to meet criteria for putting brim around the model as well as
     * around the support.
     *
     * \param storage Storage containing the parts at the first layer.
     * \param primary_line_count Number of offsets / brim lines of the primary
     * extruder.
     * \param is_skirt Whether a skirt is being generated vs a brim
     * \param[out] first_layer_outline The resulting reference polygons
     */
//     static void getFirstLayerOutline(SliceDataStorage& storage, const size_t primary_line_count, const bool is_skirt, Polygons& first_layer_outline);

    static void generateSupportBrim(SliceDataStorage& storage, const bool merge_with_model_skirtbrim);

private:
    /*!
     * \brief Generate the skirt/brim lines around the model.
     * 
     * \param start_distance The distance of the first outset from the parts at
     * the first line.
     * \param primary_line_count Number of offsets / brim lines of the primary
     * extruder.
     * \param primary_extruder_minimal_length The minimal total length of the
     * skirt/brim lines of the primary extruder.
     * \param first_layer_outline The reference polygons from which to offset
     * outward to generate skirt/brim lines.
     * \param[out] skirt_brim_primary_extruder Where to store the resulting
     * brim/skirt lines.
     * \return The offset of the last brim/skirt line from the reference polygon
     * \p first_layer_outline.
     */
    static coord_t generatePrimarySkirtBrimLines(const coord_t start_distance, size_t& primary_line_count, const coord_t primary_extruder_minimal_length, const Polygons& first_layer_outline, Polygons& skirt_brim_primary_extruder);
};
}//namespace cura

#endif //SKIRT_BRIM_H
