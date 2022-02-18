//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SKIRT_BRIM_H
#define SKIRT_BRIM_H

#include "utils/Coord_t.h"
#include "ExtruderTrain.h"
#include "sliceDataStorage.h"

namespace cura 
{

class Polygons;
class SliceDataStorage;

class SkirtBrim
{
private:
    struct Offset
    {
        Offset(const Polygons* reference_outline, const size_t reference_idx, bool external_only, const coord_t offset_value, const coord_t total_offset, size_t inset_idx, const int extruder_nr, bool is_last)
        : reference_outline(reference_outline)
        , reference_idx(reference_idx)
        , external_only(external_only)
        , offset_value(offset_value)
        , total_offset(total_offset)
        , inset_idx(inset_idx)
        , extruder_nr(extruder_nr)
        , is_last(is_last)
        {}
        const Polygons* reference_outline; //!< Optional reference polygons from which to offset
        int reference_idx; //!< Optional reference index into storage.skirt_brim from which to offset
        bool external_only; //!< Wether to only offset outward from the reference polygons
        coord_t offset_value; //!< Distance by which to offset from the reference
        coord_t total_offset; //!< Total distance from the model
        size_t inset_idx; //!< The outset index of this brimline
        int extruder_nr; //!< The extruder by which to print this brim line
        mutable bool is_last; //!< Whether this is the last planned offset for this extruder.
    };
    
    struct OffsetSorter
    {
        bool operator()(const Offset& a, const Offset& b)
        {
            return a.total_offset + a.extruder_nr
            < b.total_offset + b.extruder_nr; // add extruder_nr so that it's more stable when both extruders have the same offset settings
        }
    };
    
    SliceDataStorage& storage;
    const bool is_brim;
    const bool is_skirt;
    const bool has_ooze_shield;
    const bool has_draft_shield;
    const std::vector<ExtruderTrain>& extruders;
    const int extruder_count;
    const std::vector<bool> extruder_is_used;
    int first_used_extruder_nr;
    int skirt_brim_extruder_nr;
    std::vector<bool> external_polys_only;
    std::vector<coord_t> line_widths;
    std::vector<coord_t> skirt_brim_minimal_length;
    std::vector<int> line_count;
    std::vector<coord_t> gap;
public:
    SkirtBrim(SliceDataStorage& storage);

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
    void generate();

    /*!
     * Generate the brim inside the ooze shield and draft shield
     * 
     * \warning Adjusts brim_covered_area
     * 
     * \param storage Storage containing the parts at the first layer.
     * \param[in,out] brim_covered_area The area that was covered with brim before (in) and after (out) adding the shield brims
     */
    void generateShieldBrim(Polygons& brim_covered_area);

    /*!
     * \brief Get the reference outline of the first layer around which to
     * generate the first brim/skirt line.
     *
     * This function may change the support polygons in the first layer
     * in order to meet criteria for putting brim around the model as well as
     * around the support.
     *
     * \param extruder_nr The extruder for which to get the outlines. -1 to include outliens for all extruders
     * \return The resulting reference polygons
     */
    Polygons getFirstLayerOutline(const int extruder_nr);

    /*!
     * Generate a brim line with offset parameters given by \p offset from the \p starting_outlines and store it in the \ref storage.
     * 
     * \warning Has side effects on \p covered_area, \p allowed_areas_per_extruder and \p total_length
     * 
     * \param offset The parameters with which to perform the offset
     * \param[in,out] covered_area The total area covered by the brims (and models) on the first layer.
     * \param[in,out] allowed_areas_per_extruder The difference between the machine areas and the \p covered_area
     * \param[out] result Where to store the resulting brim line
     * \return The length of the added lines
     */
    coord_t generateOffset(const Offset& offset, Polygons& covered_area, std::vector<Polygons>& allowed_areas_per_extruder, SkirtBrimLine& result);

    /*!
     * Generate a skirt of extruders which don't yet comply with the minimum length requirement.
     * 
     * This skirt goes directly adjacent to all primary brims.
     * 
     * The skirt is stored in storage.skirt_brim.
     * 
     * \param[in,out] covered_area The total area covered by the brims (and models) on the first layer.
     * \param[in,out] allowed_areas_per_extruder The difference between the machine areas and the \p covered_area
     * \param[in,out] total_length The total length of the brim lines for each extruder.
     */
    void generateSecondarySkirtBrim(Polygons& covered_area, std::vector<Polygons>& allowed_areas_per_extruder, std::vector<coord_t>& total_length);
public:
    void generateSupportBrim(const bool merge_with_model_skirtbrim);

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
