//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SLICE_DATA_STORAGE_H
#define SLICE_DATA_STORAGE_H

#include <map>
#include "PrimeTower.h"
#include "RetractionConfig.h"
#include "SupportInfillPart.h"
#include "TopSurface.h"
#include "settings/Settings.h" //For MAX_EXTRUDERS.
#include "settings/types/AngleDegrees.h" //Infill angles.
#include "settings/types/LayerIndex.h"
#include "utils/AABB.h"
#include "utils/AABB3D.h"
#include "utils/IntPoint.h"
#include "utils/NoCopy.h"
#include "utils/optional.h"
#include "utils/polygon.h"
#include "WipeScriptConfig.h"

namespace cura 
{

class Mesh;
class SierpinskiFillProvider;

/*!
 * A SkinPart is a connected area designated as top and/or bottom skin. 
 * Surrounding each non-bridged skin area with an outline may result in better top skins.
 * It's filled during FffProcessor.processSliceData(.) and used in FffProcessor.writeGCode(.) to generate the final gcode.
 */
class SkinPart
{
public:
    PolygonsPart outline;           //!< The skinOutline is the area which needs to be 100% filled to generate a proper top&bottom filling. It's filled by the "skin" module.
    std::vector<Polygons> insets;   //!< The skin can have perimeters so that the skin lines always start at a perimeter instead of in the middle of an infill cell.
    Polygons perimeter_gaps; //!< The gaps between the extra skin walls and gaps between the outer skin wall and the inner part inset
    Polygons inner_infill; //!< The inner infill of the skin with which the area within the innermost inset is filled
    Polygons roofing_fill; //!< The inner infill which has air directly above
};


/*!
    The SliceLayerPart is a single enclosed printable area for a single layer. (Also known as islands)
    It's filled during the FffProcessor.processSliceData(.), where each step uses data from the previous steps.
    Finally it's used in the FffProcessor.writeGCode(.) to generate the final gcode.
 */
class SliceLayerPart
{
public:
    AABB boundaryBox;       //!< The boundaryBox is an axis-aligned bounardy box which is used to quickly check for possible collision between different parts on different layers. It's an optimalization used during skin calculations.
    PolygonsPart outline;       //!< The outline is the first member that is filled, and it's filled with polygons that match a cross section of the 3D model. The first polygon is the outer boundary polygon and the rest are holes.
    Polygons print_outline; //!< An approximation to the outline of what's actually printed, based on the outer wall. Too small parts will be omitted compared to the outline.
    std::vector<Polygons> insets;         //!< The insets are generated with. The insets are also known as perimeters or the walls.
    Polygons perimeter_gaps; //!< The gaps between consecutive walls and between the inner wall and outer skin inset
    Polygons outline_gaps; //!< The gaps between the outline of the mesh and the first wall. a.k.a. thin walls.
    std::vector<SkinPart> skin_parts;     //!< The skin parts which are filled for 100% with lines and/or insets.

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

    std::vector<std::pair<Polygons, double>> spaghetti_infill_volumes; //!< For each filling volume on this layer, the area within which to fill and the total volume (in mm3) to fill over the area
};

/*!
    The SlicerLayer contains all the data for a single cross section of the 3D model.
 */
class SliceLayer
{
public:
    coord_t printZ;     //!< The height at which this layer needs to be printed. Can differ from sliceZ due to the raft.
    coord_t thickness;  //!< The thickness of this layer. Can be different when using variable layer heights.
    std::vector<SliceLayerPart> parts;  //!< An array of LayerParts which contain the actual data. The parts are printed one at a time to minimize travel outside of the 3D model.
    Polygons openPolyLines; //!< A list of lines which were never hooked up into a 2D polygon. (Currently unused in normal operation)
    mutable std::map<size_t, Polygons> innermost_walls_cache; //!< Cache for the in some cases computationaly expensive calculations in 'getInnermostWalls'.
        // ^^^^ NOTE: Caching function-results like this, when they don't change but are expensive to calculate, is generally considered one of the few 'acceptable uses' of the 'mutable' keyword.

    /*!
     * \brief The parts of the model that are exposed at the very top of the
     * model.
     *
     * This is filled only when the top surface is needed.
     */
    TopSurface top_surface;

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

    /*!
     * Collects the second wall of every part, or the outer wall if it has no second, or the outline, if it has no outer wall.
     * \result The collection of all polygons thus obtained.
     * \param max_inset If <= 1, use (up to) the 1st inner wall, if >= 2, use the 2nd inner wall.
     * \param mesh Pass mesh to let the function have access to wall-line-width settings.
     */
    Polygons& getInnermostWalls(const size_t max_inset, const SliceMeshStorage& mesh) const;

    ~SliceLayer();
};

/******************/




class SupportLayer
{
public:
    std::vector<SupportInfillPart> support_infill_parts;  //!< a list of support infill parts
    Polygons support_bottom; //!< Piece of support below the support and above the model. This must not overlap with any of the support_infill_parts or support_roof.
    Polygons support_roof; //!< Piece of support above the support and below the model. This must not overlap with any of the support_infill_parts or support_bottom.
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
    SierpinskiFillProvider* cross_fill_provider; //!< the fractal pattern for the cross (3d) filling pattern

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
    std::vector<Polygons> full_overhang_areas; //!< For each layer the full overhang without the tangent of the overhang angle removed, such that the overhang area adjoins the areas of the next layers.
    std::vector<std::vector<Polygons>> overhang_points; //!< For each layer a list of points where point-overhang is detected. This is overhang that hasn't got any surface area, such as a corner pointing downwards.
    AABB3D bounding_box; //!< the mesh's bounding box

    SubDivCube* base_subdiv_cube;
    SierpinskiFillProvider* cross_fill_provider; //!< the fractal pattern for the cross (3d) filling pattern

    /*!
     * \brief Creates a storage space for slice results of a mesh.
     * \param mesh The mesh that the storage space belongs to.
     * \param slice_layer_count How many layers are needed to store the slice
     * results of the mesh. This needs to be at least as high as the highest
     * layer that contains a part of the mesh.
     */
    SliceMeshStorage(Mesh* mesh, const size_t slice_layer_count);

    virtual ~SliceMeshStorage();

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

class SliceDataStorage : public NoCopy
{
public:
    size_t print_layer_count; //!< The total number of layers (except the raft and filler layers)

    Point3 model_size, model_min, model_max;
    AABB3D machine_size; //!< The bounding box with the width, height and depth of the printer.
    std::vector<SliceMeshStorage> meshes;

    std::vector<WipeScriptConfig> wipe_config_per_extruder; //!< Wipe configs per extruder.

    std::vector<RetractionConfig> retraction_config_per_extruder; //!< Retraction config per extruder.
    std::vector<RetractionConfig> extruder_switch_retraction_config_per_extruder; //!< Retraction config per extruder for when performing an extruder switch

    SupportStorage support;

    Polygons skirt_brim[MAX_EXTRUDERS]; //!< Skirt and brim polygons per extruder, ordered from inner to outer polygons.
    Polygons raftOutline;               //Storage for the outline of the raft. Will be filled with lines when the GCode is generated.

    int max_print_height_second_to_last_extruder; //!< Used in multi-extrusion: the layer number beyond which all models are printed with the same extruder
    std::vector<int> max_print_height_per_extruder; //!< For each extruder the highest layer number at which it is used.
    std::vector<size_t> max_print_height_order; //!< Ordered indices into max_print_height_per_extruder: back() will return the extruder number with the highest print height.

    std::vector<int> spiralize_seam_vertex_indices; //!< the index of the seam vertex for each layer
    std::vector<Polygons* > spiralize_wall_outlines; //!< the wall outline polygons for each layer

    PrimeTower primeTower;

    std::vector<Polygons> oozeShield;        //oozeShield per layer
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
     */
    Polygons getLayerOutlines(const LayerIndex layer_nr, const bool include_support, const bool include_prime_tower, const bool external_polys_only = false) const;

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
     * \param adhesion_offset whether to offset the border by the adhesion width to account for brims, skirts and
     * rafts, if present.
     * \return a Polygon representing the usable area of the print bed.
     */
    Polygon getMachineBorder(bool adhesion_offset = false) const;

private:
    /*!
     * Construct the retraction_config_per_extruder
     */
    std::vector<RetractionConfig> initializeRetractionConfigs();

    /*!
     * Construct the wipe_config_per_extruder
     */
    std::vector<WipeScriptConfig> initializeWipeConfigs();
};

}//namespace cura

#endif//SLICE_DATA_STORAGE_H
