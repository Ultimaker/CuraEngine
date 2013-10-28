/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef OPTIMIZED_MODEL_H
#define OPTIMIZED_MODEL_H

#include <map>
#include "modelFile/modelFile.h"

class OptimizedFace
{
public:
    int index[3];
    int touching[3];
};
class OptimizedPoint3
{
public:
    Point3 p;
    vector<uint32_t> faceIndexList;
    
    OptimizedPoint3(Point3 p): p(p) {}
};

class OptimizedModel;
class OptimizedVolume
{
public:
    OptimizedModel* model;
    vector<OptimizedPoint3> points;
    vector<OptimizedFace> faces;
    
    OptimizedVolume(SimpleVolume* volume, OptimizedModel* model);
    
    int getFaceIdxWithPoints(int idx0, int idx1, int notFaceIdx)
    {
        for(unsigned int i=0;i<points[idx0].faceIndexList.size();i++)
        {
            int f0 = points[idx0].faceIndexList[i];
            if (f0 == notFaceIdx) continue;
            for(unsigned int j=0;j<points[idx1].faceIndexList.size();j++)
            {
                int f1 = points[idx1].faceIndexList[j];
                if (f1 == notFaceIdx) continue;
                if (f0 == f1) return f0;
            }
        }
        return -1;
    }
};
class OptimizedModel
{
public:
    vector<OptimizedVolume> volumes;
    Point3 modelSize;
    Point3 vMin, vMax;
    
    OptimizedModel(SimpleModel* model, Point3 center)
    {
        for(unsigned int i=0; i<model->volumes.size(); i++)
            volumes.push_back(OptimizedVolume(&model->volumes[i], this));
        vMin = model->min();
        vMax = model->max();

        Point3 vOffset((vMin.x + vMax.x) / 2, (vMin.y + vMax.y) / 2, vMin.z);
        vOffset -= center;
        for(unsigned int i=0; i<volumes.size(); i++)
            for(unsigned int n=0; n<volumes[i].points.size(); n++)
                volumes[i].points[n].p -= vOffset;
        
        modelSize = vMax - vMin;
        vMin -= vOffset;
        vMax -= vOffset;
    }

    void saveDebugSTL(const char* filename);
};

#endif//OPTIMIZED_MODEL_H
