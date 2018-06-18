//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SquareSubdivTest.h"
#include "../src/infill/SquareSubdiv.h"
#include "../src/infill/ImageBasedDensityProvider.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(SquareSubdivTest);

void SquareSubdivTest::setUp()
{
    // no dothing
}

void SquareSubdivTest::tearDown()
{
    // no dothing
}

void SquareSubdivTest::debugCheck()
{
    coord_t line_width = 400;
    int max_depth = 8;
    AABB3D aabb_3d(Point3(0, 0, 0), Point3(line_width, line_width, line_width) * 512);
    AABB aabb = aabb_3d.flatten();
    std::cerr << "AABB: " << aabb.max << "\n";
    
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/lena.png", aabb_3d);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/gradient.png", aabb_3d);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/slight_gradient.png", aabb_3d);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/simple.png", aabb_3d);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/gray.png", aabb_3d);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/triangle.png", aabb_3d);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/contrast.png", aabb_3d);

//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/hitler.jpg", aabb_3d);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/cheshire_cat.jpg", aabb_3d);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/vader.jpg", aabb_3d);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/sinterklaas.jpg", aabb_3d);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/diamond.jpg", aabb_3d);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/trex.jpeg", aabb_3d);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/bagelorb.jpg", aabb_3d);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/soulpilot.jpg", aabb_3d);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/soulpilot_dark.jpg", aabb_3d);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/cura-logo.jpg", aabb_3d);
    DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/T-shirt-printing/input/deer.png", aabb_3d);
    
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/nessy.jpg", aabb_3d);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/smize.png", aabb_3d);

    bool space_filling_curve = false;
    SquareSubdiv ss(*subdivider, aabb_3d, max_depth, line_width, space_filling_curve);
    ss.initialize();
//     ss.subdivideUpto(2);
//     ss.createMinimalDensityPattern();
    ss.createMinimalErrorPattern(false);
//     ss.createDitheredPattern();
//     ss.createMinimalDensityPattern();
//     ss.debugCheck();
    
    {
        Point canvas_size = Point(1024, 1024);
        SVG svg("output/subdiv_dither2.svg", aabb, canvas_size);
        
        bool draw_arrows = false;
        float drawing_line_width = static_cast<float>(line_width) * svg.getScale();
        if (draw_arrows) drawing_line_width *= .2;
        
        
        ss.debugOutput(svg, drawing_line_width, draw_arrows);
//         ss.outputHilbert(svg, drawing_line_width * 2);
//         ss.outputMoore(svg, drawing_line_width * 2);
    }
}

}
