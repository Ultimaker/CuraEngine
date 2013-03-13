#ifndef BRIDGE_H
#define BRIDGE_H

int bridgeAngle(SliceLayerPart* part, SliceLayer* prevLayer)
{
    //To detect if we have a bridge, first calculate the intersection of the current layer with the previous layer.
    // This gives us the islands that the layer rests on.
    ClipperLib::Clipper bridgeClip;
    bridgeClip.AddPolygon(part->outline[0], ClipperLib::ptSubject);
    
    for(unsigned int n=0; n<prevLayer->parts.size(); n++)
    {
        if (!part->boundaryBox.hit(prevLayer->parts[n].boundaryBox)) continue;
        
        bridgeClip.AddPolygon(prevLayer->parts[n].outline[0], ClipperLib::ptClip);
    }
    
    Polygons islands;
    bridgeClip.Execute(ClipperLib::ctIntersection, islands);
    if (islands.size() > 5)
        return -1;
    
    //Next find the 2 largest islands that we rest on.
    double area1 = 0;
    double area2 = 0;
    int idx1 = -1;
    int idx2 = -1;
    for(unsigned int n=0; n<islands.size(); n++)
    {
        double area = abs(Area(islands[n]));
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
    
    Point center1 = centerOfMass(islands[idx1]);
    Point center2 = centerOfMass(islands[idx2]);
    
    double angle = atan2(center2.X - center1.X, center2.Y - center1.Y) / M_PI * 180;
    if (angle < 0) angle += 360;
    return angle;
}

#endif//BRIDGE_H
