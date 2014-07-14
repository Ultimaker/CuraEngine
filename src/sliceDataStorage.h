/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SLICE_DATA_STORAGE_H
#define SLICE_DATA_STORAGE_H

#include "utils/intpoint.h"
#include "utils/polygon.h"

/*
SliceData
+ Layers[]
  + LayerParts[]
    + OutlinePolygons[]
    + Insets[]
      + Polygons[]
    + SkinPolygons[]
*/
namespace cura {

class SliceLayerPart
{
public:
    AABB boundaryBox;
    Polygons outline;
    Polygons combBoundery;
    vector<Polygons> insets;    //insets[n] is the inset (n * line_width + line_width/2) offset
    Polygons skinOutline;
    vector<Polygons> sparse_outline; //sparse_outline[n] is sparse outline of (n+1) layer thick. 
};

class SliceLayer
{
public:
    int sliceZ;
    int printZ;
    vector<SliceLayerPart> parts;
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
    vector<SupportPoint>* grid;
   	SupportStorage(){grid = nullptr;}
	  ~SupportStorage(){if(grid) delete [] grid;}
};
/******************/

class SliceVolumeStorage
{
public:
    vector<SliceLayer> layers;
};

class SliceDataStorage
{
public:
    Point3 modelSize, modelMin, modelMax;
    Polygons skirt;
    Polygons raftOutline;               //Storage for the outline of the raft. Will be filled with lines when the GCode is generated.
    vector<Polygons> oozeShield;        //oozeShield per layer
    vector<SliceVolumeStorage> volumes;
    
    SupportStorage support;
    Polygons wipeTower;
    Point wipePoint;
};

}//namespace cura

#endif//SLICE_DATA_STORAGE_H
