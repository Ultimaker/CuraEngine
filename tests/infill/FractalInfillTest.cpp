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
    int max_depth = 8;
    AABB aabb(Point(0,0), Point(line_width, line_width)*(1 << max_depth));
    
    DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/lena.png", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/gradient.png", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/slight_gradient.png", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/simple.png", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/gray.png", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/triangle.png", aabb);

//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/hitler.jpg", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/cheshire_cat.jpg", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/vader.jpg", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/sinterklaas.jpg", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/diamond.jpg", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/trex.jpeg", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/soulpilot.jpg", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/soulpilot_dark.jpg", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/cura-logo.jpg", aabb);
    
    SquareSubdivision ss(*subdivider, aabb, max_depth - 1, line_width);
    ss.initialize();
//     ss.createMinimalErrorPattern(false);
    ss.createDitheredPattern();
    
    {
        Point canvas_size = Point(1024, 1024);
        SVG svg("output/subdiv_dither.svg", aabb, canvas_size);
        
        int drawing_line_width = line_width * canvas_size.X / aabb.max.X;
        bool draw_arrows = false;
        ss.debugOutput(svg, drawing_line_width, draw_arrows);
    }
}

}
