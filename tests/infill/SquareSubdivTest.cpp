//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SquareSubdivTest.h"
#include "../src/infill/SquareSubdiv.h"
#include "../src/infill/ImageBasedDensityProvider.h"
#include "../src/infill/CombinedDensityProvider.h"

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

    coord_t line_width = 350;
    int max_depth = 7;
    AABB3D aabb_3d(Point3(0, 0, 0), Point3(line_width, line_width, line_width) * (1 << max_depth));
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
        , "rubber_duck_v4.png"   // 24
        , "line.png"};           // 25

    std::string img_name = img_names[1];
//     for (std::string img_name : img_names)
//     for (int do_dither = 0; do_dither < 3; do_dither++)
    for (int method = 0; method < 3; method++)
    {
        {
//             DensityProvider* density = new ImageBasedDensityProvider(std::string("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/") + img_name, aabb_3d, 0.8, 0.05, 0.25);
//             DensityProvider* density = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/paper/image_sources/balance_vs_min_req/rubber_duck_input_v2.png", aabb_3d, 1.0, 0.0, 0.25);
            DensityProvider* density = new ImageBasedDensityProvider(std::string("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/") + img_name, aabb_3d, 1.0, 0.0, 0.25);
//             DensityProvider* density = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/line.png", aabb_3d, 1.0, 0.0, 0.25);
//             DensityProvider* subdivider = new ImageBasedDensityProvider(std::string("/home/t.kuipers/Documents/PhD/Fractal Dithering project/paper/image_sources/algorithm_overview_3d/try3/overview_gradient.svg.png"), aabb_3d, 1.0, 0.0, 0.25);
//             DensityProvider* min_density = new ImageBasedDensityProvider(std::string("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/rubber_duck_v4_min_density.png"), aabb_3d, 1.0, 0.0, 0.0);

//             DensityProvider* density = new CombinedDensityProvider(*subdivider, *min_density);
            
            bool space_filling_curve = false;
            SquareSubdiv ss(*density, aabb_3d, max_depth, line_width, space_filling_curve);
            ss.initialize();
            std::string method_str = "";
            switch (method)
            {
                case 0:
                    continue;
                    ss.createDitheredPattern(true); method_str = "_dit";
                break;
                case 1:
                    ss.createMinimalDensityPattern(/* one_step_less_dense = */ true, /* averaging_statistic = */ 0); method_str = "_min";
                break;
                case 2:
                    ss.createBalancedPattern(); method_str = "_bal";
                break;
                case 3:
                    ss.createMaximalDensityPattern(); method_str = "_max";
            }
//             if (false)
            {
                SVG svg(std::string("output/square_subdiv/") + img_name + method_str + ".svg", aabb, Point(512,512), SVG::NamedColor::NONE, SVG::OMIT_BORDERS);
                bool draw_arrows = false;
                float drawing_line_width = static_cast<float>(line_width) * svg.getScale();
                if (draw_arrows) drawing_line_width *= .2;
//                 ss.debugOutput(svg, drawing_line_width, draw_arrows);
                ss.debugOutputDensities(svg);
            }
            ss.dither();
//             if (false)
            {
                SVG svg(std::string("output/square_subdiv/") + img_name + method_str + "_dithered.svg", aabb, Point(512,512), SVG::NamedColor::NONE, SVG::OMIT_BORDERS);
                bool draw_arrows = false;
                float drawing_line_width = static_cast<float>(line_width) * svg.getScale();
                if (draw_arrows) drawing_line_width *= .2;
//                 ss.debugOutput(svg, drawing_line_width, draw_arrows);
                ss.debugOutputDensities(svg);
            }
            {
                SVG svg(std::string("output/square_subdiv/") + img_name + method_str + "_dithered_grid.svg", aabb, Point(512,512), SVG::NamedColor::NONE, SVG::OMIT_BORDERS);
                bool draw_arrows = false;
                float drawing_line_width = static_cast<float>(line_width) * svg.getScale();
                if (draw_arrows) drawing_line_width *= .2;
                ss.debugOutput(svg, drawing_line_width, draw_arrows);
            }
//             if (do_dither) ss.dither();
        //     ss.debugCheck();

            if (false)
            {
                ss.createMinimalDensityPattern();
    //             if (false)
                {
                    SVG svg(std::string("output/square_subdiv/") + img_name + "_subdiv_structure_top_density.svg", aabb, Point(512,512), SVG::NamedColor::NONE, SVG::OMIT_BORDERS);
                    bool draw_arrows = false;
                    float drawing_line_width = static_cast<float>(line_width) * svg.getScale() / 2;
                    if (draw_arrows) drawing_line_width *= .2;
                    ss.debugOutput(svg, drawing_line_width, draw_arrows);
                }
    //             if (false)
                {
                    SVG svg(std::string("output/square_subdiv/") + img_name + "_fractal.svg", aabb, Point(512,512), SVG::NamedColor::NONE, SVG::OMIT_BORDERS);
                    Polygon poly = ss.createMooreLine();
                    float drawing_line_width = static_cast<float>(line_width) * svg.getScale();
                    svg.writePolygon(poly, SVG::NamedColor::BLACK, drawing_line_width);
                }
            }
        }
    }
}
    

}
