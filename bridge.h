/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef BRIDGE_H
#define BRIDGE_H

#include "sliceDataStorage.h"

int bridgeAngle(SliceLayerPart* part, SliceLayer* prevLayer);

class Edge
{
public:
  Edge() {}

  Edge(Point pa, Point pb)
  {
    A=pa;
    B=pb;
  }

  Point center(void) const
  {
    return Point((A.X+B.X)/2,(A.Y+B.Y)/2);
  }

// simple check if the edge is attached to polygons
// we get the center point of the edge and check if 2 micron square around this point makes intersection with the polygons 
  bool isAttached(const Polygons &pol) const;

  int length(void) const
  {
    return round(sqrt(length2()));
  }

  int angle(void) const
  {
    double angle = atan2(A.X - B.X, A.Y - B.Y) / M_PI * 180;
    if (angle < 0) angle += 360;
    return round(angle);
  }
  
  float length2(void) const // returns square of the length (to avoid the square root function)
  {
    return pow((float)(A.X-B.X),2)+pow((float)(A.Y-B.Y),2);
  }


  Point A;
  Point B;
};


// this class enhances the PolygonRef
// adds method edge that returns Nth edge of the polygon.

class EnhancedPolygonRef:public PolygonRef
{
public:
  EnhancedPolygonRef(const PolygonRef &pr):PolygonRef(pr) {}

public:
  Edge edge(int index)
  {
    POLY_ASSERT(index < size());
    return Edge(operator[](index),operator[]((index+1)%size()));
  }
};


#endif//BRIDGE_H
