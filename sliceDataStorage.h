/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SLICE_DATA_STORAGE_H
#define SLICE_DATA_STORAGE_H

#include <vector>
using std::vector;
#include "utils/intpoint.h"
using ClipperLib::Polygons;

/*
SliceData
+ Layers[]
  + LayerParts[]
    + OutlinePolygons[]
    + Insets[]
      + Polygons[]
    + SkinPolygons[]
*/

class SliceLayerPart
{
public:
    AABB boundaryBox;
    Polygons outline;
    vector<Polygons> insets;
    Polygons skinOutline;
    Polygons sparseOutline;
    int bridgeAngle;
};

class SliceLayer
{
public:
    vector<SliceLayerPart> parts;
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
    Point gridOffset;
    int32_t gridScale;
    int32_t gridWidth, gridHeight;
    vector<SupportPoint>* grid;
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
    Polygons raftOutline;
    vector<SliceVolumeStorage> volumes;
    
    SupportStorage support;
};

#endif//SLICE_DATA_STORAGE_H
