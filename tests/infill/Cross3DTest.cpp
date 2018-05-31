//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Cross3DTest.h"
#include "../src/infill/ImageBasedDensityProvider.h"
#include "../src/infill/UniformDensityProvider.h"

namespace cura
{
CPPUNIT_TEST_SUITE_REGISTRATION(Cross3DTest);

void Cross3DTest::setUp()
{
    // no dothing
}

void Cross3DTest::tearDown()
{
    // no dothing
}

void Cross3DTest::debugCheck()
{
    coord_t line_width = 400;
    AABB3D aabb(Point3(0, 0, 0), Point3(line_width, line_width, line_width) * 8);
    AABB aabb2d = aabb.flatten();
    std::cerr << "AABB: " << aabb.max << "\n";
    //aabb.expand(512);
    SVG bottom_svg("output/bottom.svg", aabb2d);
    SVG tree_svg("output/tree.svg", aabb2d);
    SVG net_svg("output/net.svg", aabb2d);
    
    net_svg.writePoint(aabb2d.max, true);
    
    float drawing_line_width = 3; // line_width * svg.getScale() / 4;

    int max_depth = 1;
    for (coord_t size = line_width; size < aabb2d.max.X; size = size << 1)
    {
        max_depth += 2;
    }
    max_depth = 7;
    std::cerr << " max_depth = " << max_depth << '\n';

//     DensityProvider* density_provider = new ImageBasedDensityProvider("/home/t.kuipers/Documents/PhD/Fractal Dithering project/input images/slight_gradient.png", aabb2d);
    DensityProvider* density_provider = new UniformDensityProvider(1.0);
    
    

    Cross3D f(*density_provider, aabb, max_depth, line_width);
    f.initialize();
    f.debugOutputTree(tree_svg, drawing_line_width);
    f.createMinimalDensityPattern();
    f.debugOutputSequence(net_svg, drawing_line_width);
    Cross3D::SliceWalker seq = f.getSequence(0);
    std::cerr << "seq size = " << seq.layer_sequence.size() << '\n';
    f.debugOutput(seq, bottom_svg, drawing_line_width);

}

} // namespace cura
