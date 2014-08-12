/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SLICE_DATA_STORAGE_H
#define SLICE_DATA_STORAGE_H

#include "utils/intpoint.h"
#include "utils/polygon.h"
#include "mesh.h"
#include "GCodePlanner.h"

namespace cura {

class SliceLayerPart
{
public:
    AABB boundaryBox;
    Polygons outline;
    Polygons combBoundery;
    std::vector<Polygons> insets;    //insets[n] is the inset (n * line_width + line_width/2) offset
    Polygons skinOutline;
    std::vector<Polygons> sparse_outline; //sparse_outline[n] is sparse outline of (n+1) layer thick. 
};

class SliceLayer
{
public:
    int sliceZ;
    int printZ;
    std::vector<SliceLayerPart> parts;
    Polygons openLines;
};

/******************/
class SupportPoint
{
public:
    int32_t z;
    double cosAngle;
    
    SupportPoint(int32_t z, double cosAngle) : z(z), cosAngle(cosAngle) {}
};
class SupportStorage
{
public:
    bool generated;
    int angle;
    bool everywhere;
    int XYDistance;
    int ZDistance;
    
    Point gridOffset;
    int32_t gridScale;
    int32_t gridWidth, gridHeight;
    std::vector<SupportPoint>* grid;
    
   	SupportStorage(){grid = nullptr;}
    ~SupportStorage(){if(grid) delete [] grid;}
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
    GCodePathConfig fill_config[MAX_SPARSE_COMBINE];
    
    SliceMeshStorage(SettingsBase* settings)
    : settings(settings), inset0_config(&retraction_config, "WALL-OUTER"), insetX_config(&retraction_config, "WALL-INNER")
    {
        for(int n=0; n<MAX_SPARSE_COMBINE; n++)
            fill_config[n] = GCodePathConfig(&retraction_config, "FILL");
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
