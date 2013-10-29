/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SUPPORT_H
#define SUPPORT_H

#include "sliceDataStorage.h"
#include "optimizedModel.h"

void generateSupportGrid(SupportStorage& storage, OptimizedModel* om, int supportAngle, bool supportEverywhere, int supportXYDistance, int supportZDistance);

class SupportPolyGenerator
{
public:
    Polygons polygons;

private:
    SupportStorage& storage;
    double cosAngle;
    int32_t z;
    int supportZDistance;
    bool everywhere;
    int* done;

    bool needSupportAt(Point p);
    void lazyFill(Point startPoint);
    
public:
    SupportPolyGenerator(SupportStorage& storage, int32_t z);
};

#endif//SUPPORT_H
