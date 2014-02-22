/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "bridge.h"
#include "utils/polygondebug.h"
#include "utils/logoutput.h"

                              
// simple check if the edge is attached to polygons
// we get the center point of the edge and check if 2 micron square around this point makes intersection with the polygons 
bool Edge::isAttached(const Polygons &pol) const
{
    Polygons mes;
    Polygon me;
    Point c=center();
    me.add(c+Point(1,1));
    me.add(c+Point(-1,1));
    me.add(c+Point(-1,-1));
    me.add(c+Point(1,-1));
    me.add(c+Point(1,1));
    mes.add(me);
    return mes.intersection(pol).size()>0;
}
            
            

int bridgeAngle(SliceLayerPart* part, SliceLayer* prevLayer)
{
    //To detect if we have a bridge, first calculate the difference of the current layer with the previous layer.
    Polygons prevLayerPolygons;
    for(unsigned int n=0; n<prevLayer->parts.size(); n++)
    {
        if (!part->boundaryBox.hit(prevLayer->parts[n].boundaryBox)) continue;
        prevLayerPolygons.add(prevLayer->parts[n].outline);
    }

// this is the pure bridge:
    Polygons bridge=part->outline.difference(prevLayerPolygons);
    if (bridge.size()==0) return -1; // no bridges, using default angle

// find the largest polygon of the bridge, supposing that smaller polygons are artefacts
    int maxArea=0;
    EnhancedPolygonRef greatBridge=bridge[0];
    for (unsigned int n=0; n<bridge.size() && bridge.size()>1; n++)
    {
      int area=bridge[n].area();
      if (area>maxArea)
      {
        greatBridge=bridge[n];
        maxArea=area;
      }
    }

// now we must discover which edges of the polygon are attached to previous layer
    std::vector<bool> attached;
    for (unsigned int n=0; n<greatBridge.size(); n++)
    {
      Edge e=greatBridge.edge(n);
//      _log("edge %d: (%lld,%lld)->(%lld,%lld), center (%lld,%lld), attached=%s\n", n, e.A.X, e.A.Y, e.B.X, e.B.Y, e.center().X, e.center().Y, e.isAttached(prevLayerPolygons)?"yes":"no");
      attached.push_back(e.isAttached(prevLayerPolygons));
    }

// find the first edge that is attached
    int firstAttached=-1;
    for (unsigned int n=0; n<attached.size(); n++)
    {
      if (attached[n]) 
      {
        firstAttached=n;
        break;
      }
    }
    if (firstAttached<0) return -1; // no attached edge, using default angle

// find the longest NON attached chain of edges
    Edge longestEdge;
    float longestEdgeLength=0;
    
    unsigned int n=firstAttached;
    unsigned int lastTest=attached.size()+firstAttached; // check all edges starting from the first attached

    while (n<lastTest)
    {
      while (attached[n%attached.size()] && n<=lastTest) {n++;} // seek to begin of non attached chain
      Point begin=greatBridge[n%attached.size()];
//      _log("found begin at %d (%lld,%lld)\n", n, begin.X, begin.Y);
      while (!attached[n%attached.size()] && n<=lastTest) {n++;} // seek to next attached edge
      Point end=greatBridge[n%attached.size()];
//      _log("found end  at %d (%lld,%lld)\n", n, end.X, end.Y);
      Edge nonAttachedEdge(begin,end);
      float len=nonAttachedEdge.length2();
//      _log("length %f\n", len);
      if (len>longestEdgeLength)
      {
        longestEdgeLength=len;
        longestEdge=nonAttachedEdge;
      }
    }
//    _log("longest non attached edge is (%lld,%lld)->(%lld,%lld), angle is %d\n", longestEdge.A.X, longestEdge.A.Y, longestEdge.B.X, longestEdge.B.Y, longestEdge.angle());
    if (longestEdgeLength==0) return -1; // all edges are attached, using default angle

    return longestEdge.angle();
}

