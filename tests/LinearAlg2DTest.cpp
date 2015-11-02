//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "LinearAlg2DTest.h"

#include <../src/utils/linearAlg2D.h>

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(LinearAlg2DTest);

void LinearAlg2DTest::setUp()
{
    //Do nothing.
}

void LinearAlg2DTest::tearDown()
{
    //Do nothing.
}

void LinearAlg2DTest::getDist2FromLineSegmentTest()
{
    Point line_start(0,0);
    Point line_end(10,0);
    Point point(3,3);
    CPPUNIT_ASSERT(LinearAlg2D::getDist2FromLineSegment(line_start,point,line_end) == 9);
    //CPPUNIT_ASSERT(LinearAlg2D::getDist2FromLineSegment(line_start,point,line_end) == 10);
}

}