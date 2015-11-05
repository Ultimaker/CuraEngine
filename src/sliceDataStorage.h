/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SLICE_DATA_STORAGE_H
#define SLICE_DATA_STORAGE_H

#include "utils/intpoint.h"
#include "utils/polygon.h"
#include "mesh.h"
#include "gcodePlanner.h"
#include "MeshGroup.h"
#include "PrimeTower.h"

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
    PolygonsPart outline;               //!< The skinOutline is the area which needs to be 100% filled to generate a proper top&bottom filling. It's filled by the "skin" module.
    std::vector<Polygons> insets;   //!< The skin can have perimeters so that the skin lines always start at a perimeter instead of in the middle of an infill cell.
    Polygons perimeterGaps;         //!< The gaps introduced by avoidOverlappingPerimeters which would otherwise be overlapping perimeters.
};

/*!
 * A ReinforcementWall is like an insulated wall behind the outer walls.
 * It consists of an area with (generally more dense) infill and perimeters on the inside.
 * On the outside it has the outer walls, or the inner walls of another ReinforcementWall.
 */
class ReinforcementWall
{
public:
    Polygons wall_reinforcement_area; //!< The infill of the reinforced wall
    std::vector<Polygons> wall_reinforcement_axtra_walls; //!< The extra walls on the inside of the reinforcement infill
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
    std::vector<Polygons> insets;         //!< The insets are generated with: an offset of (index * line_width + line_width/2) compared to the outline. The insets are also known as perimeters, and printed inside out.
    std::vector<SkinPart> skin_parts;     //!< The skin parts which are filled for 100% with lines and/or insets.
    std::vector<Polygons> infill_area; //!< The infill_area are the areas which need to be filled with sparse (0-99%) infill. The infill_area is an array to support thicker layers of sparse infill. infill_area[n] is infill_area of (n+1) layers thick. 
    std::vector<ReinforcementWall> reinforcement_walls; //!< The reinforcement walls for this part. Order: from outter to inner reinforcement wall.
    Polygons perimeterGaps; //!< The gaps introduced by avoidOverlappingPerimeters which would otherwise be overlapping perimeters.
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
    
    Polygons getOutlines(bool external_polys_only = false);
    void getOutlines(Polygons& result, bool external_polys_only = false);
};

/******************/
class SupportLayer
{
public:
    Polygons supportAreas; //!< normal support areas
    Polygons roofs; //!< the support areas which are to be printed as denser roofs. Note that the roof areas and support areas are mutually exclusive.
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

    int layer_nr_max_filled_layer; //!< the layer number of the uppermost layer with content
    
    RetractionConfig retraction_config;
    GCodePathConfig inset0_config;
    GCodePathConfig insetX_config;
    GCodePathConfig skin_config;
    std::vector<GCodePathConfig> infill_config;
    GCodePathConfig wall_reinforcement_config;
    
    SliceMeshStorage(SettingsBaseVirtual* settings)
    : SettingsMessenger(settings), layer_nr_max_filled_layer(0), inset0_config(&retraction_config, "WALL-OUTER"), insetX_config(&retraction_config, "WALL-INNER"), skin_config(&retraction_config, "SKIN"), wall_reinforcement_config(&retraction_config, "SUPPORT")
    {
        infill_config.reserve(MAX_INFILL_COMBINE);
        for(int n=0; n<MAX_INFILL_COMBINE; n++)
            infill_config.emplace_back(&retraction_config, "FILL");
    }
};

class SliceDataStorage : public SettingsMessenger
{
public:
    MeshGroup* meshgroup; // needed to pass on the per extruder settings.. (TODO: put this somewhere else? Put the per object settings here directly, or a pointer only to the per object settings.)
    
    Point3 model_size, model_min, model_max;
    std::vector<SliceMeshStorage> meshes;
    
    std::vector<RetractionConfig> retraction_config_per_extruder; //!< used for support, skirt, etc.
    RetractionConfig retraction_config; //!< The retraction config used as fallback when getting the per_extruder_config or the mesh config was impossible (for travelConfig)
    
    GCodePathConfig travel_config; //!< The config used for travel moves (only the speed and retraction config are set!)
    std::vector<GCodePathConfig> skirt_config; //!< config for skirt per extruder
    std::vector<CoastingConfig> coasting_config; //!< coasting config per extruder
    
    GCodePathConfig raft_base_config;
    GCodePathConfig raft_interface_config;
    GCodePathConfig raft_surface_config;
    
    GCodePathConfig support_config;
    GCodePathConfig support_roof_config;
    
    SupportStorage support;
    
    Polygons skirt[MAX_EXTRUDERS]; //!< Skirt polygons per extruder, ordered from inner to outer polygons
    Polygons raftOutline;               //Storage for the outline of the raft. Will be filled with lines when the GCode is generated.
    
    int max_object_height_second_to_last_extruder; //!< Used in multi-extrusion: the layer number beyond which all models are printed with the same extruder
    PrimeTower primeTower;
    
    std::vector<Polygons> oozeShield;        //oozeShield per layer
    Polygons draft_protection_shield; //!< The polygons for a heightened skirt which protects from warping by gusts of wind and acts as a heated chamber.
    Point wipePoint;
    
    std::vector<RetractionConfig> initializeRetractionConfigs()
    {
        std::vector<RetractionConfig> ret;
        ret.resize(meshgroup->getExtruderCount()); // initializes with constructor RetractionConfig()
        return ret;
    }
    std::vector<GCodePathConfig> initializeSkirtConfigs()
    {
        std::vector<GCodePathConfig> ret;
        for (int extruder = 0; extruder < meshgroup->getExtruderCount(); extruder++)
        {
            RetractionConfig* extruder_retraction_config = &retraction_config_per_extruder[extruder];
            skirt_config.emplace_back(extruder_retraction_config, "SKIRT");
        }
        return ret;
    }
    SliceDataStorage(MeshGroup* meshgroup)
    : SettingsMessenger(meshgroup)
    , meshgroup(meshgroup)
    , retraction_config_per_extruder(initializeRetractionConfigs())
    , travel_config(&retraction_config, "MOVE")
    , skirt_config(initializeSkirtConfigs())
    , raft_base_config(&retraction_config_per_extruder[meshgroup->getSettingAsIndex("adhesion_extruder_nr")], "SUPPORT")
    , raft_interface_config(&retraction_config_per_extruder[meshgroup->getSettingAsIndex("adhesion_extruder_nr")], "SUPPORT")
    , raft_surface_config(&retraction_config_per_extruder[meshgroup->getSettingAsIndex("adhesion_extruder_nr")], "SUPPORT")
    , support_config(&retraction_config_per_extruder[meshgroup->getSettingAsIndex("support_extruder_nr")], "SUPPORT")
    , support_roof_config(&retraction_config_per_extruder[meshgroup->getSettingAsIndex("support_roof_extruder_nr")], "SKIN")
    , max_object_height_second_to_last_extruder(-1)
//     , primeTower()
    {
    }
    
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
    Polygons getLayerOutlines(int layer_nr, bool include_helper_parts, bool external_polys_only = false);
};

}//namespace cura

#endif//SLICE_DATA_STORAGE_H
