// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SKIRT_BRIM_H
#define SKIRT_BRIM_H

#include <variant>

#include "ExtruderTrain.h"
#include "settings/EnumSettings.h"
#include "sliceDataStorage.h"
#include "utils/Coord_t.h"

namespace cura
{

class Polygons;
class SliceDataStorage;

constexpr coord_t min_brim_line_length = 3000u; //!< open polyline brim lines smaller than this will be removed

class SkirtBrim
{
private:
    /*!
     * A helper class to store an offset yet to be performed on either an outline polygon, or based on an earlier generated brim line.
     */
    struct Offset
    {
        Offset(
            const std::variant<Polygons*, int>& reference_outline_or_index,
            const bool external_only,
            const coord_t offset_value,
            const coord_t total_offset,
            const size_t inset_idx,
            const int extruder_nr,
            const bool is_last)
            : reference_outline_or_index(reference_outline_or_index)
            , external_only(external_only)
            , offset_value(offset_value)
            , total_offset(total_offset)
            , inset_idx(inset_idx)
            , extruder_nr(extruder_nr)
            , is_last(is_last)
        {
        }

        std::variant<Polygons*, int> reference_outline_or_index;
        bool external_only; //!< Wether to only offset outward from the reference polygons
        coord_t offset_value; //!< Distance by which to offset from the reference
        coord_t total_offset; //!< Total distance from the model
        int inset_idx; //!< The outset index of this brimline
        int extruder_nr; //!< The extruder by which to print this brim line
        bool is_last; //!< Whether this is the last planned offset for this extruder.
    };

    /*!
     * Defines an order on offsets (potentially from different extruders) based on how far the offset is from the original outline.
     */
    static inline const auto OffsetSorter{ [](const Offset& a, const Offset& b)
                                           {
                                               // Use extruder_nr in case both extruders have the same offset settings.
                                               return a.total_offset != b.total_offset ? a.total_offset < b.total_offset : a.extruder_nr < b.extruder_nr;
                                           } };

    SliceDataStorage& storage; //!< Where to retrieve settings and store brim lines.
    const EPlatformAdhesion adhesion_type; //!< Whether we are generating brim, skirt, or raft
    const bool has_ooze_shield; //!< Whether the meshgroup has an ooze shield
    const bool has_draft_shield; //!< Whether the meshgroup has a draft shield
    const std::vector<ExtruderTrain>& extruders; //!< The extruders of the current slice
    const int extruder_count; //!< The total number of extruders
    const std::vector<bool> extruder_is_used; //!< For each extruder whether it is actually used in this print
    int first_used_extruder_nr; //!< The first extruder which is used
    int skirt_brim_extruder_nr; //!< The extruder with which the skirt/brim is printed or -1 if printed with both
    std::vector<bool> external_polys_only; //!< For each extruder whether to only generate brim on the outside
    std::vector<coord_t> line_widths; //!< For each extruder the skirt/brim line width
    std::vector<coord_t> skirt_brim_minimal_length; //!< For each extruder the minimal brim length
    std::vector<int> line_count; //!< For each extruder the (minimal) number of brim lines to generate
    std::vector<coord_t> gap; //!< For each extruder the gap between the part and the first brim/skirt line

public:
    /*!
     * Precomputes some values used in several functions when calling \ref generate
     *
     * \param storage Storage containing the parts at the first layer.
     */
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

private:
    /*!
     * Plan the offsets which we will be going to perform and put them in the right order.
     *
     * In order for brims of different materials to grow toward the middle,
     * we need to perform the offsets alternatingly.
     * We therefore first create all planned Offset objects,
     * and then order them according to distance from the boundary.
     * \param[out] starting_outlines The first layer outlines from which to compute the offsets. Returned as output parameter because pointers need to stay valid.
     * \return An ordered list of offsets to perform in the order in which they are to be performed.
     */
    std::vector<Offset> generateBrimOffsetPlan(std::vector<Polygons>& starting_outlines);

    /*!
     * Generate the primary skirt/brim of the one skirt_brim_extruder or of all extruders simultaneously.
     *
     * \param[in,out] all_brim_offsets The offsets to perform. Adjusted when the minimal length constraint isn't met yet.
     * \param[in,out] covered_area The area of the first layer covered by model or generated brim lines.
     * \param[in,out] allowed_areas_per_extruder The difference between the machine bed area (offsetted by the nozzle offset) and the covered_area.
     * \return The total length of the brim lines added by this method per extruder.
     */
    std::vector<coord_t> generatePrimaryBrim(std::vector<Offset>& all_brim_offsets, Polygons& covered_area, std::vector<Polygons>& allowed_areas_per_extruder);

    /*!
     * Generate the brim inside the ooze shield and draft shield
     *
     * \warning Adjusts brim_covered_area
     *
     * \param storage Storage containing the parts at the first layer.
     * \param[in,out] brim_covered_area The area that was covered with brim before (in) and after (out) adding the shield brims
     * \param[in,out] allowed_areas_per_extruder The difference between the machine areas and the \p covered_area
     */
    void generateShieldBrim(Polygons& brim_covered_area, std::vector<Polygons>& allowed_areas_per_extruder);

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
    Polygons getFirstLayerOutline(const int extruder_nr = -1);

    /*!
     * The disallowed area around the internal holes of parts with other parts inside which would get an external brim.
     *
     * In order to prevent the external_only brim of a part inside another part to overlap with the internal holes of the outer part,
     * we generate a disallowed area around those internal hole polygons.
     *
     * \param outline The full layer outlines
     * \param extruder_nr The extruder for which to compute disallowed areas
     * \return The disallowed areas
     */
    Polygons getInternalHoleExclusionArea(const Polygons& outline, const int extruder_nr);

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
    /*!
     * Generate the brim which is printed from the outlines of the support inward.
     */
    void generateSupportBrim();
};
} // namespace cura

#endif // SKIRT_BRIM_H
