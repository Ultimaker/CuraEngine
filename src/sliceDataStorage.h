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
 * It's filled during fffProcessor.processSliceData(.) and used in fffProcessor.writeGCode(.) to generate the final gcode.
 */    
class SkinPart
{
public:
    PolygonsPart outline;               //!< The skinOutline is the area which needs to be 100% filled to generate a proper top&bottom filling. It's filled by the "skin" module.
    std::vector<Polygons> insets;   //!< The skin can have perimeters so that the skin lines always start at a perimeter instead of in the middle of an infill cell.
    Polygons perimeterGaps;         //!< The gaps introduced by avoidOverlappingPerimeters which would otherwise be overlapping perimeters.
};
/*!
    The SliceLayerPart is a single enclosed printable area for a single layer. (Also known as islands)
    It's filled during the fffProcessor.processSliceData(.), where each step uses data from the previous steps.
    Finally it's used in the fffProcessor.writeGCode(.) to generate the final gcode.
 */
class SliceLayerPart
{
public:
    AABB boundaryBox;       //!< The boundaryBox is an axis-aligned bounardy box which is used to quickly check for possible collision between different parts on different layers. It's an optimalization used during skin calculations.
    PolygonsPart outline;       //!< The outline is the first member that is filled, and it's filled with polygons that match a cross section of the 3D model. The first polygon is the outer boundary polygon and the rest are holes.
    std::vector<Polygons> insets;         //!< The insets are generated with: an offset of (index * line_width + line_width/2) compared to the outline. The insets are also known as perimeters, and printed inside out.
    std::vector<SkinPart> skin_parts;     //!< The skin parts which are filled for 100% with lines and/or insets.
    std::vector<Polygons> sparse_outline; //!< The sparse_outline are the areas which need to be filled with sparse (0-99%) infill. The sparse_outline is an array to support thicker layers of sparse infill. sparse_outline[n] is sparse outline of (n+1) layers thick. 
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
    Polygons openLines; //!< A list of lines which were never hooked up into a 2D polygon. (Currently unused in normal operation)
};

/******************/
class SupportLayer
{
public:
    Polygons supportAreas;
    Polygons roofs;
};

class SupportStorage
{
public:
    bool generated; //!< whether generateSupportGrid(.) has completed (successfully)
        
    int layer_nr_max_filled_layer; //!< the layer number of the uppermost layer with content
    
    std::vector<SupportLayer> supportLayers;

    SupportStorage() : layer_nr_max_filled_layer(-1) { }
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
    
    SliceMeshStorage(SettingsBaseVirtual* settings)
    : SettingsMessenger(settings), layer_nr_max_filled_layer(0), inset0_config(&retraction_config, "WALL-OUTER"), insetX_config(&retraction_config, "WALL-INNER"), skin_config(&retraction_config, "SKIN")
    {
        infill_config.reserve(MAX_SPARSE_COMBINE);
        for(int n=0; n<MAX_SPARSE_COMBINE; n++)
            infill_config.emplace_back(&retraction_config, "FILL");
    }
};

class SliceDataStorage : public SettingsMessenger
{
public:
    MeshGroup* meshgroup; // needed to pass on the per extruder settings.. (TODO: put this somewhere else? Put the per object settings here directly, or a pointer only to the per object settings.)
    
    Point3 model_size, model_min, model_max;
    int max_object_height_second_to_last_extruder; //!< Used in multi-extrusion: the layer number beyond which all models are printed with the same extruder
    Polygons skirt[MAX_EXTRUDERS]; //!< Skirt polygons per extruder, ordered from inner to outer polygons
    Polygons raftOutline;               //Storage for the outline of the raft. Will be filled with lines when the GCode is generated.
    std::vector<Polygons> oozeShield;        //oozeShield per layer
    std::vector<SliceMeshStorage> meshes;

    RetractionConfig retraction_config;
    GCodePathConfig* skirt_config[MAX_EXTRUDERS] = {nullptr}; //!< config for skirt per extruder
    GCodePathConfig support_config;
    GCodePathConfig support_roof_config;
    
    SupportStorage support;
    PrimeTower primeTower;
    
    Polygons draft_protection_shield; //!< The polygons for a heightened skirt which protects from warping by gusts of wind and acts as a heated chamber.
    Point wipePoint;
    
    SliceDataStorage(MeshGroup* meshgroup)
    : SettingsMessenger(meshgroup)
    , meshgroup(meshgroup)
    , max_object_height_second_to_last_extruder(-1)
    , support_config(&retraction_config, "SUPPORT")
    , support_roof_config(&retraction_config, "SKIN")
    , primeTower(meshgroup, &retraction_config)
    {
        for (int extruder = 0; extruder < meshgroup->getExtruderCount(); extruder++)
        {
            skirt_config[extruder] = new GCodePathConfig(&retraction_config, "SKIRT");
        }
    }
    
    ~SliceDataStorage()
    {
        for (int e =0; e < meshgroup->getExtruderCount(); e++)
        {
            delete skirt_config[e];
        }
    }
};

}//namespace cura

#endif//SLICE_DATA_STORAGE_H
