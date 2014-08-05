/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SLICE_DATA_STORAGE_H
#define SLICE_DATA_STORAGE_H

#include "utils/intpoint.h"
#include "utils/polygon.h"
#include "mesh.h"

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
    
    SliceMeshStorage(SettingsBase* settings) : settings(settings) {}
};

class SliceDataStorage
{
public:
    Point3 model_size, model_min, model_max;
    Polygons skirt;
    Polygons raftOutline;               //Storage for the outline of the raft. Will be filled with lines when the GCode is generated.
    std::vector<Polygons> oozeShield;        //oozeShield per layer
    std::vector<SliceMeshStorage> meshes;
    
    SupportStorage support;
    Polygons wipeTower;
    Point wipePoint;
};

}//namespace cura

#endif//SLICE_DATA_STORAGE_H
