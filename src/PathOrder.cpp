//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PathOrder.h" //The definitions we're implementing here.
#include "sliceDataStorage.h" //For SliceLayerPart and related parts to reorder.

//Since the PathOrder is a template class, we will only implement the template specializations in this file.
//The templated segments need to go into the header.

namespace cura
{

template<>
ConstPolygonRef PathOrder<ConstPolygonRef>::getVertexData(ConstPolygonRef path)
{
    return path;
}

template<>
ConstPolygonRef PathOrder<PolygonRef>::getVertexData(PolygonRef path)
{
    return path;
}

template<>
ConstPolygonRef PathOrder<const SkinPart*>::getVertexData(const SkinPart* path)
{
    return path->outline.outerPolygon();
}

template<>
ConstPolygonRef PathOrder<const SliceLayerPart*>::getVertexData(const SliceLayerPart* path)
{
    return path->outline.outerPolygon();
}

template<>
ConstPolygonRef PathOrder<const SupportInfillPart*>::getVertexData(const SupportInfillPart* path)
{
    return path->outline.outerPolygon();
}

}