/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SLICE_DATA_STORAGE_H
#define SLICE_DATA_STORAGE_H

#include "utils/intpoint.h"
#include "utils/optional.h"
#include "utils/polygon.h"
#include "utils/NoCopy.h"
#include "utils/AABB.h"
#include "mesh.h"
#include "gcodePlanner.h"
#include "MeshGroup.h"
#include "PrimeTower.h"
#include "GCodePathConfig.h"

namespace cura 
{
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
    std::vector<Polygons> insets;         //!< The insets are generated with: an offset of (index * line_width + line_width/2) compared to the outline. The insets are also known as perimeters, and printed inside out.
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
};

/*!
    The SlicerLayer contains all the data for a single cross section of the 3D model.
 */
class SliceLayer
{
public:
    int sliceZ;     //!< The height at which the 3D model was cut. 
    // TODO: remove this /\ unused member!
    int printZ;     //!< The height at which this layer needs to be printed. Can differ from sliceZ due to the raft.
    std::vector<SliceLayerPart> parts;  //!< An array of LayerParts which contain the actual data. The parts are printed one at a time to minimize travel outside of the 3D model.
    Polygons openPolyLines; //!< A list of lines which were never hooked up into a 2D polygon. (Currently unused in normal operation)

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
     * \return The collection of all polygons thus obtained
     */
    Polygons getSecondOrInnermostWalls() const;

    /*!
     * Collects the second wall of every part, or the outer wall if it has no second, or the outline, if it has no outer wall.
     * Add those polygons to @p result.
     * \param result The result: the collection of all polygons thus obtained
     */
    void getSecondOrInnermostWalls(Polygons& result) const;
};

/******************/
class SupportLayer
{
public:
    Polygons supportAreas; //!< normal support areas
    Polygons skin; //!< the support areas which are to be printed as denser roofs and/or bottoms. Note that the roof/bottom areas and support areas should be mutually exclusive.
};

class SupportStorage
{
public:
    bool generated; //!< whether generateSupportGrid(.) has completed (successfully)

    int layer_nr_max_filled_layer; //!< the layer number of the uppermost layer with content

    std::vector<SupportLayer> supportLayers;

    SupportStorage() : generated(false), layer_nr_max_filled_layer(-1) { }
    ~SupportStorage(){ supportLayers.clear(); }
};
/******************/

class SliceMeshStorage : public SettingsMessenger // passes on settings from a Mesh object
{
public:
    std::vector<SliceLayer> layers;

    int layer_nr_max_filled_layer; //!< the layer number of the uppermost layer with content (modified while infill meshes are processed)

    GCodePathConfig inset0_config;
    GCodePathConfig insetX_config;
    GCodePathConfig skin_config;
    std::vector<GCodePathConfig> infill_config;

    SliceMeshStorage(SettingsBaseVirtual* settings, unsigned int slice_layer_count)
    : SettingsMessenger(settings)
    , layer_nr_max_filled_layer(0)
    , inset0_config(PrintFeatureType::OuterWall)
    , insetX_config(PrintFeatureType::InnerWall)
    , skin_config(PrintFeatureType::Skin)
    {
        layers.reserve(slice_layer_count);
        infill_config.reserve(MAX_INFILL_COMBINE);
        for(int n=0; n<MAX_INFILL_COMBINE; n++)
            infill_config.emplace_back(PrintFeatureType::Infill);
    }
};

class SliceDataStorage : public SettingsMessenger, NoCopy
{
public:
    MeshGroup* meshgroup; // needed to pass on the per extruder settings.. (TODO: put this somewhere else? Put the per object settings here directly, or a pointer only to the per object settings.)

    Point3 model_size, model_min, model_max;
    std::vector<SliceMeshStorage> meshes;

    std::vector<RetractionConfig> retraction_config_per_extruder; //!< Retraction config per extruder.
    std::vector<RetractionConfig> extruder_switch_retraction_config_per_extruder; //!< Retraction config per extruder for when performing an extruder switch

    std::vector<GCodePathConfig> travel_config_per_extruder; //!< The config used for travel moves (only speed is set!)

    std::vector<GCodePathConfig> skirt_brim_config; //!< Configuration for skirt and brim per extruder.
    std::vector<CoastingConfig> coasting_config; //!< coasting config per extruder

    GCodePathConfig raft_base_config;
    GCodePathConfig raft_interface_config;
    GCodePathConfig raft_surface_config;

    GCodePathConfig support_config;
    GCodePathConfig support_skin_config; //!< The config to use to print the dense roofs and bottoms of support

    SupportStorage support;

    Polygons skirt_brim[MAX_EXTRUDERS]; //!< Skirt and brim polygons per extruder, ordered from inner to outer polygons.
    Polygons raftOutline;               //Storage for the outline of the raft. Will be filled with lines when the GCode is generated.

    int max_object_height_second_to_last_extruder; //!< Used in multi-extrusion: the layer number beyond which all models are printed with the same extruder
    PrimeTower primeTower;

    std::vector<Polygons> oozeShield;        //oozeShield per layer
    Polygons draft_protection_shield; //!< The polygons for a heightened skirt which protects from warping by gusts of wind and acts as a heated chamber.

    /*!
     * Construct the initial retraction_config_per_extruder
     */
    std::vector<RetractionConfig> initializeRetractionConfigs();

    /*!
     * Construct the initial travel_config_per_extruder
     */
    std::vector<GCodePathConfig> initializeTravelConfigs();

    /*!
     * Construct the initial skirt & brim configurations for each extruder.
     */
    std::vector<GCodePathConfig> initializeSkirtBrimConfigs();

    /*!
     * \brief Creates a new slice data storage that stores the slice data of the
     * specified mesh group.
     * 
     * It will obtain the settings from the mesh group too. The mesh group is
     * not yet sliced in this constructor. If no mesh group is provided, an
     * empty one will be created.
     * 
     * \param meshgroup The mesh group to load into this data storage, if any.
     */
    SliceDataStorage(MeshGroup* meshgroup);

    ~SliceDataStorage()
    {
    }

    /*!
     * Get all outlines within a given layer.
     * 
     * \param layer_nr the index of the layer for which to get the outlines (negative layer numbers indicate the raft)
     * \param include_helper_parts whether to include support and prime tower
     * \param external_polys_only whether to disregard all hole polygons
     */
    Polygons getLayerOutlines(int layer_nr, bool include_helper_parts, bool external_polys_only = false) const;

    /*!
     * Collects the second wall of every part, or the outer wall if it has no second, or the outline, if it has no outer wall.
     * 
     * For helper parts the outlines are used.
     * 
     * \param layer_nr the index of the layer for which to get the outlines (negative layer numbers indicate the raft)
     * \param include_helper_parts whether to include support and prime tower
     */
    Polygons getLayerSecondOrInnermostWalls(int layer_nr, bool include_helper_parts) const;

    /*!
     * Get the extruders used.
     * 
     * \return a vector of bools indicating whether the extruder with corresponding index is used in this layer.
     */
    std::vector<bool> getExtrudersUsed();
};

}//namespace cura

#endif//SLICE_DATA_STORAGE_H
