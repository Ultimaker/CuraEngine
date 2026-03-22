// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SLICEDATA_SLICELAYERPART_H
#define SLICEDATA_SLICELAYERPART_H

#include <optional>

#include "geometry/SingleShape.h"
#include "slice_data/SkinPart.h"
#include "utils/AABB.h"
#include "utils/ExtrusionLine.h"

namespace cura
{

/*!
    The SliceLayerPart is a single enclosed printable area for a single layer. (Also known as islands)
    It's filled during the FffProcessor.processSliceData(.), where each step uses data from the previous steps.
    Finally it's used in the FffProcessor.writeGCode(.) to generate the final gcode.
 */
class SliceLayerPart
{
public:
    AABB boundaryBox; //!< The boundaryBox is an axis-aligned boundary box which is used to quickly check for possible
                      //!< collision between different parts on different layers. It's an optimization used during
                      //!< skin calculations.
    SingleShape outline; //!< The outline is the first member that is filled, and it's filled with polygons that match
                         //!< a cross-section of the 3D model.
    Shape print_outline; //!< An approximation to the outline of what's actually printed, based on the outer wall.
                         //!< Too small parts will be omitted compared to the outline.
    Shape spiral_wall; //!< The centerline of the wall used by spiralize mode. Only computed if spiralize mode is enabled.
    Shape inner_area; //!< The area of the outline, minus the walls. This will be filled with either skin or infill.
    std::vector<SkinPart> skin_parts; //!< The skin parts which are filled for 100% with lines and/or insets.
    std::vector<VariableWidthLines> wall_toolpaths; //!< toolpaths for walls, will replace(?) the insets. Binned by inset_idx.
    std::vector<VariableWidthLines> infill_wall_toolpaths; //!< toolpaths for the walls of the infill areas. Binned by inset_idx.
    Shape top_most_surface; //!< Sub-part of the outline containing the area that is not covered by something above
    Shape bottom_most_surface; //!< Sub-part of the outline containing the area that has nothing below

    /*!
     * The areas inside of the mesh.
     * Like SliceLayerPart::outline, this class member is not used to actually determine the feature area,
     * but is used to compute the inside comb boundary.
     */
    Shape infill_area;

    /*!
     * The areas which need to be filled with sparse (0-99%) infill.
     * Like SliceLayerPart::outline, this class member is not used to actually determine the feature area,
     * but is used to compute the infill_area_per_combine_per_density.
     *
     * These polygons may be cleared once they have been used to generate gradual infill and/or infill combine.
     *
     * If these polygons are not initialized, simply use the normal infill area.
     */
    std::optional<Shape> infill_area_own;

    /*!
     * The areas which need to be filled with sparse (0-99%) infill for different thicknesses.
     * The infill_area is an array to support thicker layers of sparse infill and areas of different infill density.
     *
     * Take an example of infill_area[x][n], the meanings of the indexes are as follows:
     *   x  -  The sparsity of the infill area (0 - 99 in percentage). So, the areas in infill_area[x] are the most dense ones.
     *   n  -  The thickness (in number of layers) of the infill area. See example below:
     *          / ------ -- /         <- layer 3
     *        /-- ------ /            <- layer 2
     *          1   2     3
     * LEGEND: "-" means infill areas.
     * Numbers 1, 2 and 3 identifies 3 different infill areas. Infill areas 1 and 3 have a tickness of 1 layer, while area 2 has a thickness of 2.
     *
     * After the areas have been categoried into different densities, overlapping parts with the same density on multiple layers
     * will be combined into a single layer. Here is an illustration:
     *
     *                        *a group of 3 layers*
     *       NOT COMBINED                                   COMBINED
     *
     *       /22222222 2 2/                                  /22222222 2 2/     <--  the 2 layers next to the middle part are combined into a single layer
     *     /0 22222222 2/              ====>               /0 ........ 2/
     *   /0 1 22222222/                                  /0 1 ......../         <--  the 3 layers in the middle part are combined into a single layer
     *                                                      ^          ^
     *                                                      |          |
     *                                                      |          |-- those two density level 2 layers cannot not be combined in the current implementation.
     *                                                      |              this is because we separate every N layers into groups, and try to combine the layers
     *                                                      |              in each group starting from the bottom one. In this case, those two layers don't have a
     *                                                      |              bottom layer in this group, so they cannot be combined.
     *                                                      |              (TODO) this can be a good future work.
     *                                                      |
     *                                                      |-- ideally, those two layer can be combined as well, but it is not the case now.
     *                                                          (TODO) this can be a good future work.
     *
     * NOTES:
     *   - numbers represent the density levels of each infill
     *
     * This maximum number of layers we can combine is a user setting. This number, say "n", means the maximum number of layers we can combine into one.
     * On the combined layers, the extrusion amount will be higher than the normal extrusion amount because it needs to extrude for multiple layers instead of one.
     *
     * infill_area[x][n] is infill_area of (n+1) layers thick.
     *
     * infill_area[0] corresponds to the most dense infill area.
     * infill_area[x] will lie fully inside infill_area[x+1].
     * infill_area_per_combine_per_density.back()[0] == part.infill area initially
     */
    std::vector<std::vector<Shape>> infill_area_per_combine_per_density;

    /*!
     * Get the infill_area_own (or when it's not instantiated: the normal infill_area)
     * \see SliceLayerPart::infill_area_own
     * \return the own infill area
     */
    Shape& getOwnInfillArea();

    /*!
     * Get the infill_area_own (or when it's not instantiated: the normal infill_area)
     * \see SliceLayerPart::infill_area_own
     * \return the own infill area
     */
    const Shape& getOwnInfillArea() const;

    /*!
     * Searches whether the part has any walls in the specified inset index
     * \param inset_idx The index of the wall
     * \return true if there is at least one ExtrusionLine at the specified wall index, false otherwise
     */
    bool hasWallAtInsetIndex(size_t inset_idx) const;
};

} // namespace cura

#endif // SLICEDATA_SLICELAYERPART_H
