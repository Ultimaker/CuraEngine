//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SierpinskiFillTest.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(SierpinskiFillTest);

void SierpinskiFillTest::setUp()
{
    // no dothing
}

void SierpinskiFillTest::tearDown()
{
    // no dothing
}

void SierpinskiFillTest::debugCheck()
{
    AABB aabb(Point(0,0), Point(1024, 1024)*8);
    //aabb.expand(512);
    SVG svg("output/sierpinski.html", aabb, Point(1000, 1000)*2);
    
    
//     srand(1);
    int max_depth = 18;
    {
        SierpinskiFill f(aabb, max_depth);
        SVG::Color color = SVG::Color::GREEN;
        switch (max_depth % 4)
        {
            default:
            case 0: color = SVG::Color::GREEN; break;
            case 1: color = SVG::Color::RED; break;
            case 2: color = SVG::Color::BLUE; break;
            case 3: color = SVG::Color::YELLOW; break;
        }
        color = SVG::Color::RAINBOW;
        color = SVG::Color::BLACK;
        svg.writePolygon(f.generateCross(5200, 16), color, 4);
//         svg.writePolygon(f.generateSierpinski(), color);
//         f.debugOutput(svg);
    }
    
}

void SierpinskiFillTest::boundsCheck()
{
    
}

}
