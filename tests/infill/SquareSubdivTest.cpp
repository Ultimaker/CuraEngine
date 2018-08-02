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
    increaseVerboseLevel();
    increaseVerboseLevel();
    increaseVerboseLevel();

    coord_t line_width = 400;
    int max_depth = 10;
    AABB3D aabb_3d(Point3(0, 0, 0), Point3(line_width, line_width, line_width) * (2 << max_depth));
    AABB aabb = aabb_3d.flatten();
    std::cerr << "AABB: " << aabb.max << "\n";
    
    std::string img_names[] = 
        { "lena.png"             // 0
        , "gradient.png"         // 1
        , "slight_gradient.png"  // 2
        , "simple.png"           // 3
        , "simple_large.png"     // 4
        , "gray.png"             // 5
        , "triangle.png"         // 6
        , "contrast.png"         // 7
        , "hitler.jpg"           // 8
        , "cheshire_cat.jpg"     // 9
        , "vader.jpg"            // 10
        , "sinterklaas.jpg"      // 11
        , "diamond.jpg"          // 12
        , "trex.jpeg"            // 13
        , "bagelorb.jpg"         // 14
        , "soulpilot.jpg"        // 15
        , "soulpilot_dark.jpg"   // 16
        , "cura-logo.jpg"        // 17
        , "nessy.jpg"            // 18
        , "smize.png"            // 19
        , "ct.jpg"               // 20
        , "LDLTP.jpg"            // 21
        , "ct_pelvis.jpg"        // 22
        , "Empty-nose-after-80per-cent-partial-bilateral-turbinectomy.jpeg"      // 23
        , "rubber_duck.png"};    // 24

    std::string img_name = img_names[1];
    for (std::string img_name : img_names)
//     for (int do_dither = 0; do_dither < 3; do_dither++)
    {
        {
            DensityProvider* subdivider = new ImageBasedDensityProvider(std::string("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/") + img_name, aabb_3d);

            bool space_filling_curve = false;
            SquareSubdiv ss(*subdivider, aabb_3d, max_depth, line_width, space_filling_curve);
            ss.initialize();
        //     ss.createMinimalDensityPattern();
    //         ss.createMinimalErrorPattern(true);
            ss.createDitheredPattern();
//             ss.createBalancedPattern();
//             if (do_dither) ss.dither();
        //     ss.createMinimalDensityPattern();
        //     ss.debugCheck();

            {
                SVG svg(std::string("output/square_balance_dither_") + img_name + ".svg", aabb);
                bool draw_arrows = false;
                float drawing_line_width = static_cast<float>(line_width) * svg.getScale();
                if (draw_arrows) drawing_line_width *= .2;
                ss.debugOutput(svg, drawing_line_width, draw_arrows);
            }
        }
    }
}
    

}
