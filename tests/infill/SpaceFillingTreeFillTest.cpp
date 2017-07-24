//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SpaceFillingTreeFillTest.h"

namespace cura
{
CPPUNIT_TEST_SUITE_REGISTRATION(SpaceFillingTreeFillTest);

void SpaceFillingTreeFillTest::setUp()
{
    //Do nothing.
}

void SpaceFillingTreeFillTest::tearDown()
{
    //Do nothing.
}

void SpaceFillingTreeFillTest::test()
{
    
}

/*
void SpaceFillingTreeFillTest::pointIsLeftOfLineSharpTest()
{
    short actual = -1;
    Point p(3896, 3975);
    Point a(1599, 3975);
    Point b(200, 3996);
    //looks like:         \        .
    int64_t supposed = LinearAlg2D::pointIsLeftOfLine(p, a, b);
    std::stringstream ss;
    ss << "Point " << p << " was computed as lying " << ((supposed == 0)? "on" : ((supposed < 0)? "left" : "right")) << " the line from " << a << " to " << b << ", instead of " << ((actual == 0)? "on" : ((actual < 0)? "left" : "right"));
    CPPUNIT_ASSERT_MESSAGE(ss.str(), actual * supposed > 0 || (actual == 0 && supposed == 0));

}
*/


}
