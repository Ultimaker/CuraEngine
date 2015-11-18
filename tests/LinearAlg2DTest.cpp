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

void LinearAlg2DTest::getDist2FromLineSegmentHorizontalNearTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,0),Point(25,3),9,0);
}

void LinearAlg2DTest::getDist2FromLineSegmentHorizontalOnTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,0),Point(25,0),0,0);
}

void LinearAlg2DTest::getDist2FromLineSegmentHorizontalBeyondTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,0),Point(200,0),10000,1);
}

void LinearAlg2DTest::getDist2FromLineSegmentHorizontalBeforeTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,0),Point(-100,0),10000,-1);
}

void LinearAlg2DTest::getDist2FromLineSegmentHorizontalCornerTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,0),Point(-1,-1),2,-1);
}

void LinearAlg2DTest::getDist2FromLineSegmentHorizontalPerpendicularTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,0),Point(0,3),9,0);
}

void LinearAlg2DTest::getDist2FromLineSegmentVerticalNearTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(0,100),Point(5,25),25,0);
}

void LinearAlg2DTest::getDist2FromLineSegmentVerticalOnTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(0,100),Point(0,25),0,0);
}

void LinearAlg2DTest::getDist2FromLineSegmentVerticalBeyondTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(0,100),Point(0,200),10000,1);
}

void LinearAlg2DTest::getDist2FromLineSegmentVerticalBeforeTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(0,100),Point(0,-100),10000,-1);
}

void LinearAlg2DTest::getDist2FromLineSegmentVerticalCornerTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(0,100),Point(-1,-1),2,-1);
}

void LinearAlg2DTest::getDist2FromLineSegmentVerticalPerpendicularTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(0,100),Point(3,0),9,0);
}

void LinearAlg2DTest::getDist2FromLineSegmentDiagonalNearTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,100),Point(30,20),50,0);
}

void LinearAlg2DTest::getDist2FromLineSegmentDiagonalOnTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,100),Point(25,25),0,0);
}

void LinearAlg2DTest::getDist2FromLineSegmentDiagonalBeyondTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,100),Point(200,200),20000,1);
}

void LinearAlg2DTest::getDist2FromLineSegmentDiagonalBeforeTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,100),Point(-100,-100),20000,-1);
}

void LinearAlg2DTest::getDist2FromLineSegmentDiagonalCornerTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,100),Point(-3,0),9,-1);
}

void LinearAlg2DTest::getDist2FromLineSegmentDiagonalPerpendicularTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,100),Point(3,-3),9,0);
}

void LinearAlg2DTest::getDist2FromLineSegmentDiagonal2NearTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,50),Point(20,30),320,0);
}

void LinearAlg2DTest::getDist2FromLineSegmentDiagonal2OnTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,50),Point(40,20),0,0);
}

void LinearAlg2DTest::getDist2FromLineSegmentDiagonal2PointTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,50),Point(0,0),0,0);
}

void LinearAlg2DTest::getDist2FromLineSegmentDiagonal2BeyondTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,50),Point(200,100),12500,1);
}

void LinearAlg2DTest::getDist2FromLineSegmentDiagonal2BeforeTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,50),Point(-100,-50),12500,-1);
}

void LinearAlg2DTest::getDist2FromLineSegmentDiagonal2CornerTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,50),Point(-3,0),9,-1);
}

void LinearAlg2DTest::getDist2FromLineSegmentDiagonal2PerpendicularTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(100,50),Point(-2,4),20,0);
}

void LinearAlg2DTest::getDist2FromLineSegmentZeroNearTest()
{
    //Since the actual is_beyond may be either -1 or 1, we must make this test more specific.
    char supposed_is_beyond = 0;
    int64_t supposed_distance = LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(20,0),Point(0,0),&supposed_is_beyond);
    int64_t actual_distance = 40;
    std::stringstream ss;
    ss << "Line [0,0] -- [0,0], point [20,0], squared distance was ";
    ss << supposed_distance;
    ss << " rather than ";
    ss << actual_distance;
    ss << ".";
    CPPUNIT_ASSERT_MESSAGE(ss.str(),std::abs(supposed_distance - actual_distance) <= maximum_error);
    CPPUNIT_ASSERT_MESSAGE("Line [0,0] -- [0,0], point [20,0] was not said to be beyond the line, but it should be.",static_cast<int>(supposed_is_beyond) != 0);
}

void LinearAlg2DTest::getDist2FromLineSegmentZeroOnTest()
{
    getDist2FromLineSegmentAssert(Point(0,0),Point(0,0),Point(0,0),0,0);
}

void LinearAlg2DTest::getDist2FromLineSegmentAssert(Point line_start,Point line_end,Point point,int64_t actual_distance2,char actual_is_beyond)
{
    char supposed_is_beyond;
    int64_t supposed_distance = LinearAlg2D::getDist2FromLineSegment(line_start,point,line_end,&supposed_is_beyond);
    {
        std::stringstream ss;
        ss << "Line [";
        ss << line_start.X;
        ss << ",";
        ss << line_start.Y;
        ss << "] -- [";
        ss << line_end.X;
        ss << ",";
        ss << line_end.Y;
        ss << "], point [";
        ss << point.X;
        ss << ",";
        ss << point.Y;
        ss << "], squared distance was ";
        ss << supposed_distance;
        ss << " rather than ";
        ss << actual_distance2;
        ss << ".";
        CPPUNIT_ASSERT_MESSAGE(ss.str(),std::fabs(sqrt(double(supposed_distance)) - sqrt(double(actual_distance2))) <= maximum_error);
    }
    {
        std::stringstream ss;
        ss << "Line [";
        ss << line_start.X;
        ss << ",";
        ss << line_start.Y;
        ss << "] -- [";
        ss << line_end.X;
        ss << ",";
        ss << line_end.Y;
        ss << "], point [";
        ss << point.X;
        ss << ",";
        ss << point.Y;
        ss << "], check whether it is beyond was ";
        ss << static_cast<int>(supposed_is_beyond);
        ss << " rather than ";
        ss << static_cast<int>(actual_is_beyond);
        ss << ".";
        CPPUNIT_ASSERT_MESSAGE(ss.str(),supposed_is_beyond == actual_is_beyond);
    }
}

}