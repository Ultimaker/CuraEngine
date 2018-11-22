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
    coord_t line_width = 350;
    AABB3D aabb_3d(Point3(0, 0, 0), Point3(line_width, line_width, line_width) * 128);
    AABB aabb = aabb_3d.flatten();
    std::cerr << "AABB: " << aabb.max << "\n";
    //aabb.expand(512);
    Point canvas_size = Point(512, 512);
    SVG svg("/home/t.kuipers/Development/CuraEngine/output/fractal_dithering_layers/gradient_2d.svg", aabb, canvas_size, SVG::Color::NONE, SVG::OMIT_BORDERS);
    
    
    float drawing_line_width = line_width * svg.getScale();
    
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/lena.png", aabb_3d,  0.0, 0.8, 0.4);
    DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/gradient.png", aabb_3d,  0.1, 0.4, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/slight_gradient.png", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/simple.png", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/gray.png", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/triangle.png", aabb_3d,  0.0, 0.8, 0.4);

//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/hitler.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/cheshire_cat.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/vader.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/einstein.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/kop.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/sinterklaas.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/diamond.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/trex.jpeg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/soulpilot.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/soulpilot_dark.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/cura-logo.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/deer.png", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/mexican_grey_wolf.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/hawk_skull.jpg", aabb_3d,  0.0, 0.8, 0.4);

//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/lena_inv.png", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/enlightenment_inv.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/enlightenment_2_inv.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/enlightenment_3_inv.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/bagelorb_inv.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/sinterklaas_inv.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/deer_inv.png", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/wolf_inv.png", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/hawk_skull_inv.jpg", aabb_3d,  0.0, 0.8, 0.4);
//     DensityProvider* subdivider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/womb_inv.png", aabb_3d,  0.0, 0.8, 0.4);
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
//         color = SVG::Color::RAINBOW;
        color = SVG::Color::BLACK;
//         svg.writePolygon(f.generateCross(1200, 16), color, 4);
        svg.writePolygon(f.generateCross(), color, drawing_line_width);
//         svg.writeAreas(f.generateCross(), SVG::Color::WHITE, color, drawing_line_width);
//         svg.writePoints(f.generateCross());
//         svg.writePolygon(f.generateSierpinski(), color);
//         f.debugOutput(svg);
    }
    
}

void SierpinskiFillTest::boundsCheck()
{
    
}

}
