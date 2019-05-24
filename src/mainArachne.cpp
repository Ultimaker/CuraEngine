
#include <cstdio>
#include <string.h>
#include <strings.h>
#include <stdio.h> // for file output
#include <fstream>
#include <iostream>

#include <boost/version.hpp>

#include "utils/logoutput.h"
#include "utils/polygon.h"
#include "utils/gettime.h"
#include "utils/SVG.h"

#include "VoronoiQuadrilateralization.h"

using arachne::Point;

namespace arachne
{

Polygons generateTestPoly(size_t size, Point border)
{
    Polygons polys;
    PolygonRef poly = polys.newPoly();
    for (int i = 0; i < size; i++)
    {
        poly.emplace_back(rand() % border.X, rand() % border.Y);
    }
    
    polys = polys.unionPolygons();
//     polys = polys.offset(border.X*1.2, ClipperLib::jtRound);
    
//     polys = polys.offset(border.X*2, ClipperLib::jtRound);
//     polys = polys.offset(-border.X*1.8, ClipperLib::jtRound);
    
    polys = polys.offset(-10, ClipperLib::jtRound);
    polys = polys.offset(20, ClipperLib::jtRound);
    polys = polys.offset(-10, ClipperLib::jtRound);
//     polys = polys.offset(-border.X/200, ClipperLib::jtRound);
//     polys = polys.offset(border.X/100, ClipperLib::jtRound);
//     polys = polys.offset(-border.X/200, ClipperLib::jtRound);
    polys = polys.unionPolygons();
    return polys;
}


void test()
{
    
    // Preparing Input Geometries.
    int r;
    r = 1558617038;
    r = time(0);
//     r = 1558618076;
    r = 1558692831;
    srand(r);
    printf("random seed: %d\n", r);
    logError("boost version: %s\n", BOOST_LIB_VERSION);
    
    
    
    Polygons polys = generateTestPoly(20, Point(10000, 10000));
    {
        SVG svg("output/outline.svg", AABB(Point(0,0), Point(10000, 10000)));
        svg.writePolygons(polys);
    }
    
    
    TimeKeeper tk;
    
    VoronoiQuadrilateralization vq(polys);
    
    logError("Toal processing took %fs\n", tk.restart());
}


} // namespace arachne


int main() {
    arachne::test();
    return 0;
}
