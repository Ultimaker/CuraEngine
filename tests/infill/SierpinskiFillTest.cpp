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
    AABB aabb(Point(0,0), Point (1024, 1024));
    //aabb.expand(512);
    SVG svg("output/sierpinski.html", aabb);
    
    
//     srand(1);
//     for (int i = 20; i < 21; i++)
    int i = 18;
    {
        SierpinskiFill f(aabb, i);
        SVG::Color color = SVG::Color::GREEN;
        switch (i % 4)
        {
            default:
            case 0: color = SVG::Color::GREEN; break;
            case 1: color = SVG::Color::RED; break;
            case 2: color = SVG::Color::BLUE; break;
            case 3: color = SVG::Color::YELLOW; break;
        }
        color = SVG::Color::GREEN;
        svg.writePolygon(f.generateCross(), color);
//         svg.writePolygon(f.generateSierpinski(), color);
//         f.debugOutput(svg);
    }
    
}

void SierpinskiFillTest::boundsCheck()
{
    
}

}
