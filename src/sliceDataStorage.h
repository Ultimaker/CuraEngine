/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SLICE_DATA_STORAGE_H
#define SLICE_DATA_STORAGE_H

#include "utils/intpoint.h"
#include "utils/polygon.h"
#include "mesh.h"
#include "gcodePlanner.h"

namespace cura {
/*!
    The SliceLayerPart is a single enclosed printable area for a single layer. (Also known as islands)
    It's filled during the fffProcessor.processSliceData, where each step uses data from the previous steps.
    Finally it's used in the fffProcessor.writeGCode to generate the final gcode.
 */
class SliceLayerPart
{
public:
    AABB boundaryBox;       //!< The boundaryBox is an axis-aligned bounardy box which is used to quickly check for possible collision between different parts on different layers. It's an optimalization used during skin calculations.
    Polygons outline;       //!< The outline is the first member that is filled, and it's filled with polygons that match a cross section of the 3D model. The first polygon is the outer boundary polygon and the rest are holes.
    Polygons combBoundery;  //!< The combBoundery is generated from the online. It's the area in which the nozzle tries to stay during traveling.
    std::vector<Polygons> insets;       //!< The insets are generated with: an offset of (index * line_width + line_width/2) compared to the outline. The insets are also known as perimeters, and printed inside out.
    Polygons skinOutline;               //!< The skinOutline is the area which needs to be 100% filled to generate a proper top&bottom filling. It's filled by the "skin" module.
    std::vector<Polygons> sparse_outline; //!< The sparse_outline are the areas which need to be filled with sparse (0-99%) infill. The sparse_outline is an array to support thicker layers of sparse infill. sparse_outline[n] is sparse outline of (n+1) layers thick. 
};

/*!
    The SlicerLayer contains all the data for a single cross section of the 3D model.
 */
class SliceLayer
{
public:
    int sliceZ;     //!< The height at which the 3D model was cut.
    int printZ;     //!< The height at which this layer needs to be printed. Can differ from sliceZ due to the raft.
    std::vector<SliceLayerPart> parts;  //!< An array of LayerParts which contain the actual data. The parts are printed one at a time to minimize travel outside of the 3D model.
    Polygons openLines; //!< A list of lines which were never hooked up into a 2D polygon. (Currently unused in normal operation)
};

/******************/
class SupportStorage
{
public:
    bool generated; //!< whether generateSupportGrid(.) has completed (successfully)
        
    std::vector<Polygons> supportAreasPerLayer;

    SupportStorage(){}
    ~SupportStorage(){supportAreasPerLayer.clear(); }
};
/******************/

class SliceMeshStorage
{
public:
    SettingsBase* settings;
    std::vector<SliceLayer> layers;

    RetractionConfig retraction_config;
    GCodePathConfig inset0_config;
    GCodePathConfig insetX_config;
    GCodePathConfig skin_config;
    GCodePathConfig infill_config[MAX_SPARSE_COMBINE];
    
    SliceMeshStorage(SettingsBase* settings)
    : settings(settings), inset0_config(&retraction_config, "WALL-OUTER"), insetX_config(&retraction_config, "WALL-INNER"), skin_config(&retraction_config, "SKIN")
    {
        for(int n=0; n<MAX_SPARSE_COMBINE; n++)
            infill_config[n] = GCodePathConfig(&retraction_config, "FILL");
    }
};

class SliceDataStorage
{
public:
    Point3 model_size, model_min, model_max;
    Polygons skirt;
    Polygons raftOutline;               //Storage for the outline of the raft. Will be filled with lines when the GCode is generated.
    std::vector<Polygons> oozeShield;        //oozeShield per layer
    std::vector<SliceMeshStorage> meshes;

    RetractionConfig retraction_config;
    GCodePathConfig skirt_config;
    GCodePathConfig support_config;
    
    SupportStorage support;
    Polygons wipeTower;
    Point wipePoint;
    
    SliceDataStorage()
    : skirt_config(&retraction_config, "SKIRT"), support_config(&retraction_config, "SUPPORT")
    {
    }
};

}//namespace cura

#endif//SLICE_DATA_STORAGE_H
