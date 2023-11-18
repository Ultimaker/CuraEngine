// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SLICE_DATA_STORAGE_H
#define SLICE_DATA_STORAGE_H

#include <map>
#include <memory>
#include <optional>

#include "PrimeTower.h"
#include "RetractionConfig.h"
#include "SupportInfillPart.h"
#include "TopSurface.h"
#include "WipeScriptConfig.h"
#include "settings/Settings.h" //For MAX_EXTRUDERS.
#include "settings/types/Angle.h" //Infill angles.
#include "settings/types/LayerIndex.h"
#include "utils/AABB.h"
#include "utils/AABB3D.h"
#include "utils/IntPoint.h"
#include "utils/NoCopy.h"
#include "utils/polygon.h"

// libArachne
#include "utils/ExtrusionLine.h"

namespace cura
{

class Mesh;
class SierpinskiFillProvider;
class LightningGenerator;

/*!
 * A SkinPart is a connected area designated as top and/or bottom skin.
 * Surrounding each non-bridged skin area with an outline may result in better top skins.
 * It's filled during FffProcessor.processSliceData(.) and used in FffProcessor.writeGCode(.) to generate the final gcode.
 */
class SkinPart
{
public:
    PolygonsPart outline; //!< The skinOutline is the area which needs to be 100% filled to generate a proper top&bottom filling. It's filled by the "skin" module. Includes both
                          //!< roofing and non-roofing.
    Polygons skin_fill; //!< The part of the skin which is not roofing.
    Polygons roofing_fill; //!< The inner infill which has air directly above
    Polygons top_most_surface_fill; //!< The inner infill of the uppermost top layer which has air directly above.
    Polygons bottom_most_surface_fill; //!< The inner infill of the bottommost bottom layer which has air directly below.
};

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
    PolygonsPart outline; //!< The outline is the first member that is filled, and it's filled with polygons that match
                          //!< a cross-section of the 3D model. The first polygon is the outer boundary polygon and the
                          //!< rest are holes.
    Polygons print_outline; //!< An approximation to the outline of what's actually printed, based on the outer wall.
                            //!< Too small parts will be omitted compared to the outline.
    Polygons spiral_wall; //!< The centerline of the wall used by spiralize mode. Only computed if spiralize mode is enabled.
    Polygons inner_area; //!< The area of the outline, minus the walls. This will be filled with either skin or infill.
    std::vector<SkinPart> skin_parts; //!< The skin parts which are filled for 100% with lines and/or insets.
    std::vector<VariableWidthLines> wall_toolpaths; //!< toolpaths for walls, will replace(?) the insets. Binned by inset_idx.
    std::vector<VariableWidthLines> infill_wall_toolpaths; //!< toolpaths for the walls of the infill areas. Binned by inset_idx.

    /*!
     * The areas inside of the mesh.
     * Like SliceLayerPart::outline, this class member is not used to actually determine the feature area,
     * but is used to compute the inside comb boundary.
     */
    Polygons infill_area;

    /*!
     * The areas which need to be filled with sparse (0-99%) infill.
     * Like SliceLayerPart::outline, this class member is not used to actually determine the feature area,
     * but is used to compute the infill_area_per_combine_per_density.
     *
     * These polygons may be cleared once they have been used to generate gradual infill and/or infill combine.
     *
     * If these polygons are not initialized, simply use the normal infill area.
     */
    std::optional<Polygons> infill_area_own;

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
    std::vector<std::vector<Polygons>> infill_area_per_combine_per_density;

    /*!
     * Get the infill_area_own (or when it's not instantiated: the normal infill_area)
     * \see SliceLayerPart::infill_area_own
     * \return the own infill area
     */
    Polygons& getOwnInfillArea();

    /*!
     * Get the infill_area_own (or when it's not instantiated: the normal infill_area)
     * \see SliceLayerPart::infill_area_own
     * \return the own infill area
     */
    const Polygons& getOwnInfillArea() const;

    /*!
     * Searches whether the part has any walls in the specified inset index
     * \param inset_idx The index of the wall
     * \return true if there is at least one ExtrusionLine at the specified wall index, false otherwise
     */
    bool hasWallAtInsetIndex(size_t inset_idx) const;
};

/*!
    The SlicerLayer contains all the data for a single cross section of the 3D model.
 */
class SliceLayer
{
public:
    coord_t printZ; //!< The height at which this layer needs to be printed. Can differ from sliceZ due to the raft.
    coord_t thickness; //!< The thickness of this layer. Can be different when using variable layer heights.
    std::vector<SliceLayerPart> parts; //!< An array of LayerParts which contain the actual data. The parts are printed one at a time to minimize travel outside of the 3D model.
    Polygons openPolyLines; //!< A list of lines which were never hooked up into a 2D polygon. (Currently unused in normal operation)

    /*!
     * \brief The parts of the model that are exposed at the very top of the
     * model.
     *
     * This is filled only when the top surface is needed.
     */
    TopSurface top_surface;

    /*!
     * \brief The parts of the model that are exposed at the bottom(s) of the model.
     *
     * Note: Filled only when needed.
     */
    Polygons bottom_surface;

    /*!
     * Get the all outlines of all layer parts in this layer.
     *
     * \param external_polys_only Whether to only include the outermost outline of each layer part
     * \return A collection of all the outline polygons
     */
    Polygons getOutlines(bool external_polys_only = false) const;

    /*!
     * Get the all outlines of all layer parts in this layer.
     * Add those polygons to @p result.
     *
     * \param external_polys_only Whether to only include the outermost outline of each layer part
     * \param result The result: a collection of all the outline polygons
     */
    void getOutlines(Polygons& result, bool external_polys_only = false) const;

    ~SliceLayer();
};

/******************/


class SupportLayer
{
public:
    std::vector<SupportInfillPart> support_infill_parts; //!< a list of support infill parts
    Polygons support_bottom; //!< Piece of support below the support and above the model. This must not overlap with any of the support_infill_parts or support_roof.
    Polygons support_roof; //!< Piece of support above the support and below the model. This must not overlap with any of the support_infill_parts or support_bottom.
                           //   NOTE: This is _all_ of the support_roof, and as such, overlaps with support_fractional_roof!
    Polygons support_fractional_roof; //!< If the support distance is not exactly a multiple of the layer height,
                                      //   the first part of support just underneath the model needs to be printed at a fracional layer height.
    Polygons support_mesh_drop_down; //!< Areas from support meshes which should be supported by more support
    Polygons support_mesh; //!< Areas from support meshes which should NOT be supported by more support
    Polygons anti_overhang; //!< Areas where no overhang should be detected.

    /*!
     * Exclude the given polygons from the support infill areas and update the SupportInfillParts.
     *
     * \param exclude_polygons The polygons to exclude
     * \param exclude_polygons_boundary_box The boundary box for the polygons to exclude
     */
    void excludeAreasFromSupportInfillAreas(const Polygons& exclude_polygons, const AABB& exclude_polygons_boundary_box);

    /* Fill up the infill parts for the support with the given support polygons. The support polygons will be split into parts. This also takes into account fractional-height
     * support layers.
     *
     * \param layer_nr Current layer index.
     * \param support_fill_per_layer All of the (infill) support (since the layer above might be needed).
     * \param support_line_width Line width of the support extrusions.
     * \param wall_line_count Wall-line count around the fill.
     * \param grow_layer_above (optional, default to 0) In cases where support shrinks per layer up, an appropriate offset may be nescesary.
     * \param unionAll (optional, default to false) Wether to 'union all' for the split into parts bit.
     */
    void fillInfillParts(
        const LayerIndex layer_nr,
        const std::vector<Polygons>& support_fill_per_layer,
        const coord_t support_line_width,
        const coord_t wall_line_count,
        const coord_t grow_layer_above = 0,
        const bool unionAll = false);
};

class SupportStorage
{
public:
    bool generated; //!< whether generateSupportGrid(.) has completed (successfully)

    int layer_nr_max_filled_layer; //!< the layer number of the uppermost layer with content

    std::vector<AngleDegrees> support_infill_angles; //!< a list of angle values which is cycled through to determine the infill angle of each layer
    std::vector<AngleDegrees> support_infill_angles_layer_0; //!< a list of angle values which is cycled through to determine the infill angle of each layer
    std::vector<AngleDegrees> support_roof_angles; //!< a list of angle values which is cycled through to determine the infill angle of each layer
    std::vector<AngleDegrees> support_bottom_angles; //!< a list of angle values which is cycled through to determine the infill angle of each layer

    std::vector<SupportLayer> supportLayers;
    std::shared_ptr<SierpinskiFillProvider> cross_fill_provider; //!< the fractal pattern for the cross (3d) filling pattern

    SupportStorage();
    ~SupportStorage();
};
/******************/

class SubDivCube; // forward declaration to prevent dependency loop

class SliceMeshStorage
{
public:
    Settings& settings;
    std::vector<SliceLayer> layers;
    std::string mesh_name;

    LayerIndex layer_nr_max_filled_layer; //!< the layer number of the uppermost layer with content (modified while infill meshes are processed)

    std::vector<AngleDegrees> infill_angles; //!< a list of angle values which is cycled through to determine the infill angle of each layer
    std::vector<AngleDegrees> roofing_angles; //!< a list of angle values which is cycled through to determine the roofing angle of each layer
    std::vector<AngleDegrees> skin_angles; //!< a list of angle values which is cycled through to determine the skin angle of each layer
    std::vector<Polygons> overhang_areas; //!< For each layer the areas that are classified as overhang on this mesh.
    std::vector<Polygons> full_overhang_areas; //!< For each layer the full overhang without the tangent of the overhang angle removed, such that the overhang area adjoins the
                                               //!< areas of the next layers.
    std::vector<std::vector<Polygons>> overhang_points; //!< For each layer a list of points where point-overhang is detected. This is overhang that hasn't got any surface area,
                                                        //!< such as a corner pointing downwards.
    AABB3D bounding_box; //!< the mesh's bounding box

    std::shared_ptr<SubDivCube> base_subdiv_cube;
    std::shared_ptr<SierpinskiFillProvider> cross_fill_provider; //!< the fractal pattern for the cross (3d) filling pattern

    std::shared_ptr<LightningGenerator> lightning_generator; //!< Pre-computed structure for Lightning type infill

    RetractionAndWipeConfig retraction_wipe_config; //!< Per-Object retraction and wipe settings.

    /*!
     * \brief Creates a storage space for slice results of a mesh.
     * \param mesh The mesh that the storage space belongs to.
     * \param slice_layer_count How many layers are needed to store the slice
     * results of the mesh. This needs to be at least as high as the highest
     * layer that contains a part of the mesh.
     */
    SliceMeshStorage(Mesh* mesh, const size_t slice_layer_count);

    /*!
     * \param extruder_nr The extruder for which to check
     * \return whether a particular extruder is used by this mesh
     */
    bool getExtruderIsUsed(const size_t extruder_nr) const;

    /*!
     * \param extruder_nr The extruder for which to check
     * \param layer_nr the layer for which to check
     * \return whether a particular extruder is used by this mesh on a particular layer
     */
    bool getExtruderIsUsed(const size_t extruder_nr, const LayerIndex& layer_nr) const;

    /*!
     * Gets whether this is a printable mesh (not an infill mesh, slicing mesh,
     * etc.)
     * \return True if it's a mesh that gets printed.
     */
    bool isPrinted() const;

    /*!
     * \return the mesh's user specified z seam hint
     */
    Point getZSeamHint() const;
};

/*!
 * Class to store all open polylines or closed polygons related to one outset index of brim/skirt.
 */
struct SkirtBrimLine
{
    Polygons open_polylines;
    Polygons closed_polygons;
};

class SliceDataStorage : public NoCopy
{
public:
    size_t print_layer_count; //!< The total number of layers (except the raft and filler layers)

    Point3 model_size, model_min, model_max;
    AABB3D machine_size; //!< The bounding box with the width, height and depth of the printer.
    std::vector<std::shared_ptr<SliceMeshStorage>> meshes;

    std::vector<RetractionAndWipeConfig> retraction_wipe_config_per_extruder; //!< Config for retractions, extruder switch retractions, and wipes, per extruder.

    SupportStorage support;

    std::vector<SkirtBrimLine> skirt_brim[MAX_EXTRUDERS]; //!< Skirt/brim polygons per extruder, ordered from inner to outer polygons.
    Polygons support_brim; //!< brim lines for support, going from the edge of the support inward. \note Not ordered by inset.
    Polygons raftOutline; // Storage for the outline of the raft. Will be filled with lines when the GCode is generated.

    int max_print_height_second_to_last_extruder; //!< Used in multi-extrusion: the layer number beyond which all models are printed with the same extruder
    std::vector<int> max_print_height_per_extruder; //!< For each extruder the highest layer number at which it is used.
    std::vector<size_t> max_print_height_order; //!< Ordered indices into max_print_height_per_extruder: back() will return the extruder number with the highest print height.

    std::vector<int> spiralize_seam_vertex_indices; //!< the index of the seam vertex for each layer
    std::vector<Polygons*> spiralize_wall_outlines; //!< the wall outline polygons for each layer

    PrimeTower primeTower;

    std::vector<Polygons> oozeShield; // oozeShield per layer
    Polygons draft_protection_shield; //!< The polygons for a heightened skirt which protects from warping by gusts of wind and acts as a heated chamber.

    /*!
     * \brief Creates a new slice data storage that stores the slice data of the
     * current mesh group.
     */
    SliceDataStorage();

    ~SliceDataStorage()
    {
    }

    /*!
     * Get all outlines within a given layer.
     *
     * \param layer_nr The index of the layer for which to get the outlines
     * (negative layer numbers indicate the raft).
     * \param include_support Whether to include support in the outline.
     * \param include_prime_tower Whether to include the prime tower in the
     * outline.
     * \param external_polys_only Whether to disregard all hole polygons.
     * \param extruder_nr (optional) only give back outlines for this extruder (where the walls are printed with this extruder)
     */
    Polygons
        getLayerOutlines(const LayerIndex layer_nr, const bool include_support, const bool include_prime_tower, const bool external_polys_only = false, const int extruder_nr = -1)
            const;

    /*!
     * Get the extruders used.
     *
     * \return A vector of booleans indicating whether the extruder with the
     * corresponding index is used in the mesh group.
     */
    std::vector<bool> getExtrudersUsed() const;

    /*!
     * Get the extruders used on a particular layer.
     *
     * \param layer_nr the layer for which to check
     * \return a vector of bools indicating whether the extruder with corresponding index is used in this layer.
     */
    std::vector<bool> getExtrudersUsed(LayerIndex layer_nr) const;

    /*!
     * Gets whether prime blob is enabled for the given extruder number.
     *
     * \param extruder_nr the extruder number to check.
     * \return a bool indicating whether prime blob is enabled for the given extruder number.
     */
    bool getExtruderPrimeBlobEnabled(const size_t extruder_nr) const;

    /*!
     * Gets the border of the usable print area for this machine.
     *
     * \param extruder_nr The extruder for which to return the allowed areas. -1 if the areas allowed for all extruders should be returned.
     * \return the Polygons representing the usable area of the print bed.
     */
    Polygons getMachineBorder(int extruder_nr = -1) const;

private:
    /*!
     * Construct the retraction_wipe_config_per_extruder
     */
    std::vector<RetractionAndWipeConfig> initializeRetractionAndWipeConfigs();
};

} // namespace cura

#endif // SLICE_DATA_STORAGE_H
