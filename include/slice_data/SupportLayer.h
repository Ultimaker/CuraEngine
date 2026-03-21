// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SLICEDATA_SUPPORTLAYER_H
#define SLICEDATA_SUPPORTLAYER_H

#include "SupportInfillPart.h"

namespace cura
{

class AABB;
class MeshSliceData;
struct LayerIndex;

class SupportLayer
{
public:
    std::vector<SupportInfillPart> support_infill_parts; //!< a list of support infill parts
    Shape support_bottom; //!< Piece of support below the support and above the model. This must not overlap with any of the support_infill_parts or support_roof.
    Shape support_roof; //!< Piece of support above the support and below the model. This must not overlap with any of the support_infill_parts or support_bottom.
                        //   NOTE: This is _all_ of the support_roof, and as such, overlaps with support_fractional_roof!
    Shape support_fractional_roof; //!< If the support distance is not exactly a multiple of the layer height,
                                   //   the first part of support just underneath the model needs to be printed at a fracional layer height.
    Shape support_mesh_drop_down; //!< Areas from support meshes which should be supported by more support
    Shape support_mesh; //!< Areas from support meshes which should NOT be supported by more support
    Shape anti_overhang; //!< Areas where no overhang should be detected.

    /*!
     * Exclude the given polygons from the support infill areas and update the SupportInfillParts.
     *
     * \param exclude_polygons The polygons to exclude
     * \param exclude_polygons_boundary_box The boundary box for the polygons to exclude
     */
    void excludeAreasFromSupportInfillAreas(const Shape& exclude_polygons, const AABB& exclude_polygons_boundary_box);

    /* Fill up the infill parts for the support with the given support polygons. The support polygons will be split into parts.
     *
     * \param area The support polygon to fill up with infill parts.
     * \param support_fill_per_layer The support polygons to fill up with infill parts.
     * \param support_line_width Line width of the support extrusions.
     * \param wall_line_count Wall-line count around the fill.
     * \param use_fractional_config (optional, default to false) If the area should be added as fractional support.
     * \param unionAll (optional, default to false) Wether to 'union all' for the split into parts bit.
     * \param custom_line_distance (optional, default to 0) Distance between lines of the infill pattern. custom_line_distance of 0 means use the default instead.
     */
    void fillInfillParts(
        const Shape& area,
        const coord_t support_line_width,
        const coord_t wall_line_count,
        const bool use_fractional_config = false,
        const bool unionAll = false,
        const coord_t custom_line_distance = 0)
    {
        for (const SingleShape& island_outline : area.splitIntoParts(unionAll))
        {
            support_infill_parts.emplace_back(island_outline, support_line_width, use_fractional_config, wall_line_count, custom_line_distance);
        }
    }

    /* Fill up the infill parts for the support with the given support polygons. The support polygons will be split into parts. This also takes into account fractional-height
     * support layers.
     *
     * \param layer_nr The layer-index of the support layer to be filled.
     * \param support_fill_per_layer The support polygons to fill up with infill parts.
     * \param infill_layer_height The layer height of the support-fill.
     * \param meshes The model meshes to be supported, needed here to handle fractional support layer height.
     * \param support_line_width Line width of the support extrusions.
     * \param wall_line_count Wall-line count around the fill.
     * \param grow_layer_above (optional, default to 0) In cases where support shrinks per layer up, an appropriate offset may be nescesary.
     * \param unionAll (optional, default to false) Wether to 'union all' for the split into parts bit.
     * \param custom_line_distance (optional, default to 0) Distance between lines of the infill pattern. custom_line_distance of 0 means use the default instead.
     */
    void fillInfillParts(
        const LayerIndex layer_nr,
        const std::vector<Shape>& support_fill_per_layer,
        const coord_t infill_layer_height,
        const std::vector<std::shared_ptr<MeshSliceData>>& meshes,
        const coord_t support_line_width,
        const coord_t wall_line_count,
        const coord_t grow_layer_above = 0,
        const bool unionAll = false,
        const coord_t custom_line_distance = 0);
};

} // namespace cura

#endif
