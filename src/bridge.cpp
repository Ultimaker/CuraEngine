/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "bridge.h"

#include "sliceDataStorage.h"

namespace cura {

int bridgeAngle(Polygons outline, SliceLayer* prevLayer)
{
    AABB boundaryBox(outline);
    //To detect if we have a bridge, first calculate the intersection of the current layer with the previous layer.
    // This gives us the islands that the layer rests on.
    Polygons islands;
    for(auto prevLayerPart : prevLayer->parts)
    {
        if (!boundaryBox.hit(prevLayerPart.boundaryBox))
            continue;
        
        islands.add(outline.intersection(prevLayerPart.outline));
    }
    if (islands.size() > 5 || islands.size() < 1)
        return -1;
    
    //Next find the 2 largest islands that we rest on.
    double area1 = 0;
    double area2 = 0;
    int idx1 = -1;
    int idx2 = -1;
    for(unsigned int n=0; n<islands.size(); n++)
    {
        //Skip internal holes
        if (!islands[n].orientation())
            continue;
        double area = fabs(islands[n].area());
        if (area > area1)
        {
            if (area1 > area2)
            {
                area2 = area1;
                idx2 = idx1;
            }
            area1 = area;
            idx1 = n;
        }else if (area > area2)
        {
            area2 = area;
            idx2 = n;
        }
    }
    
    if (idx1 < 0 || idx2 < 0)
        return -1;
    
    Point center1 = islands[idx1].centerOfMass();
    Point center2 = islands[idx2].centerOfMass();

    return angle(center2 - center1);
}

}//namespace cura

