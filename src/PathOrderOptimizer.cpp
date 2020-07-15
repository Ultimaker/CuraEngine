//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PathOrderOptimizer.h" //The definitions we're implementing here.
#include "sliceDataStorage.h" //For SliceLayerPart.

//Since the PathOrderOptimizer is a template class, we will only implement the template specializations in this file.

namespace cura
{

template<>
ConstPolygonRef PathOrderOptimizer<ConstPolygonRef>::getVertexData(const ConstPolygonRef* path) const
{
    return (*path);
}

template<>
ConstPolygonRef PathOrderOptimizer<PolygonRef>::getVertexData(const PolygonRef* path) const
{
    return (*path);
}

template<>
ConstPolygonRef PathOrderOptimizer<SkinPart>::getVertexData(const SkinPart* path) const
{
    return path->outline.outerPolygon();
}

template<>
ConstPolygonRef PathOrderOptimizer<SliceLayerPart>::getVertexData(const SliceLayerPart* path) const
{
    if(!path->insets.empty())
    {
        return path->insets[0][0];
    }
    else
    {
        return path->outline.outerPolygon();
    }
}

template<>
ConstPolygonRef PathOrderOptimizer<SupportInfillPart>::getVertexData(const SupportInfillPart* path) const
{
    return path->outline.outerPolygon();
}

template<>
ConstPolygonRef PathOrderOptimizer<std::vector<ExtrusionJunction>>::getVertexData(const std::vector<ExtrusionJunction>* path) const
{
    Polygon poly;
    for(const ExtrusionJunction junction : *path)
    {
        poly.add(junction.p);
    }
    return ConstPolygonRef(poly); //TODO: The reference is lost here because poly goes out of scope?
}

}