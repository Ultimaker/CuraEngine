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
    int max_depth = 9;
    AABB3D aabb_3d(Point3(0, 0, 0), Point3(line_width, line_width, line_width) * (1 << max_depth));
    AABB aabb = aabb_3d.flatten();
    std::cerr << "AABB: " << aabb.max << "\n";
    
    std::string img_names[] = 
        { "lena.png"             // 0
        , "gradient.png"         // 1
        , "slight_gradient.png"  // 2
        , "simple.png"           // 3
        , "gray.png"             // 4
        , "triangle.png"         // 5
        , "contrast.png"         // 6
        , "hitler.jpg"           // 7
        , "cheshire_cat.jpg"     // 8
        , "vader.jpg"            // 9
        , "sinterklaas.jpg"      // 10
        , "diamond.jpg"          // 11
        , "trex.jpeg"            // 12
        , "bagelorb.jpg"         // 13
        , "soulpilot.jpg"        // 14
        , "soulpilot_dark.jpg"   // 15
        , "cura-logo.jpg"        // 16
        , "nessy.jpg"            // 17
        , "smize.png"};          // 18

//     std::string img_name = img_names[0];
    for (std::string img_name : img_names)
    {
        DensityProvider* subdivider = new ImageBasedDensityProvider(std::string("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/") + img_name, aabb_3d);

        bool space_filling_curve = false;
        SquareSubdiv ss(*subdivider, aabb_3d, max_depth, line_width, space_filling_curve);
        ss.initialize();
    //     ss.createMinimalDensityPattern();
    //     ss.createMinimalErrorPattern(false);
        ss.createDitheredPattern();
    //     ss.createMinimalDensityPattern();
    //     ss.debugCheck();

        {
            SVG svg(std::string("output/square_dither_") + img_name + ".svg", aabb);
            bool draw_arrows = false;
            float drawing_line_width = static_cast<float>(line_width) * svg.getScale();
            if (draw_arrows) drawing_line_width *= .2;
            ss.debugOutput(svg, drawing_line_width, draw_arrows);
        }
        if (false)
        {
            SVG svg(std::string("output/square_hilbert_") + img_name + ".svg", aabb);
            bool draw_arrows = false;
            float drawing_line_width = static_cast<float>(line_width) * svg.getScale();
            if (draw_arrows) drawing_line_width *= .2;
            Polygon moore_poly = ss.createMooreLine();
    //         ss.debugOutput(svg, drawing_line_width, draw_arrows);
            svg.writePolygon(moore_poly, SVG::Color::BLACK, drawing_line_width);
        }
    }
}
    

}
