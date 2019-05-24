
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
    
    polys = polys.offset(-5, ClipperLib::jtRound);
//     polys = polys.offset(10, ClipperLib::jtRound);
//     polys = polys.offset(-5, ClipperLib::jtRound);
//     polys = polys.offset(-border.X/200, ClipperLib::jtRound);
//     polys = polys.offset(border.X/100, ClipperLib::jtRound);
//     polys = polys.offset(-border.X/200, ClipperLib::jtRound);
    polys = polys.unionPolygons();
    return polys;
}
} // namespace arachne


int main() {
    // Preparing Input Geometries.
    int r;
    r = 1558617038;
    r = time(0);
//     r = 1558618076;
    r = 1558692831;
    srand(r);
    printf("random seed: %d\n", r);
    arachne::logError("boost version: %s\n", BOOST_LIB_VERSION);
    
    
    
    arachne::Polygons polys = arachne::generateTestPoly(20, Point(10000, 10000));
    
    arachne::TimeKeeper tk;
    
    arachne::VoronoiQuadrilateralization vq(polys);
    
    arachne::logError("Toal processing took %fs\n", tk.restart());
    return 0;
}
