//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SierpinskiFillTest.h"
#include "../src/infill/ImageBasedDensityProvider.h"

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
    coord_t line_width = 400;
    AABB aabb(Point(0,0), Point(line_width, line_width)*512);
    //aabb.expand(512);
    Point canvas_size = Point(1000, 1000) * 2;
    SVG svg("output/sierpinski.svg", aabb, canvas_size);
    
    
    int drawing_line_width = line_width * svg.getScale();
    
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/lena.png", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/gradient.png", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/slight_gradient.png", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/simple.png", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/gray.png", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/triangle.png", aabb);

//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/hitler.jpg", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/cheshire_cat.jpg", aabb);
    DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/vader.jpg", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/sinterklaas.jpg", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/diamond.jpg", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/trex.jpeg", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/soulpilot.jpg", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/soulpilot_dark.jpg", aabb);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/Cross Fractal/cura-logo.jpg", aabb);
//     subdivider = new UniformSubdivider();
//     srand(1);
    int max_depth = 1;
    for (coord_t size = line_width; size < aabb.max.X; size = size << 1)
    {
        max_depth += 2;
    }
    bool dithering = true;
    {
        SierpinskiFill f(*subdivider, aabb, max_depth, line_width, dithering);
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
//         svg.writePolygon(f.generateCross(1200, 16), color, 4);
//         svg.writePolygon(f.generateCross(), color, drawing_line_width);
        svg.writeAreas(f.generateCross(), SVG::Color::WHITE, color, drawing_line_width);
//         svg.writePoints(f.generateCross());
//         svg.writePolygon(f.generateSierpinski(), color);
//         f.debugOutput(svg);
    }
    
}

void SierpinskiFillTest::boundsCheck()
{
    
}

}
