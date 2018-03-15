//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "FractalInfillTest.h"
#include "../src/infill/SquareSubdivision.h"
#include "../src/infill/ImageBasedDensityProvider.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(FractalInfillTest);

void FractalInfillTest::setUp()
{
    // no dothing
}

void FractalInfillTest::tearDown()
{
    // no dothing
}

void FractalInfillTest::debugCheck()
{
    coord_t line_width = 400;
    AABB aabb(Point(0,0), Point(line_width, line_width)*512);
    
    DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/gradient.png", aabb);
    
    SquareSubdivision ss(*subdivider, aabb, line_width);
    ss.initialize();
    ss.testSubdivision();
    
    {
        Point canvas_size = Point(1000, 1000);
        SVG svg("output/subdiv_dither.svg", aabb, canvas_size);
        
        int drawing_line_width = line_width * canvas_size.X / aabb.max.X;
        ss.debugOutput(svg, drawing_line_width);
    }
}

}
