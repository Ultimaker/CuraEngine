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
    char is_beyond;
    
    //Horizontal line.
    CPPUNIT_ASSERT_EQUAL(int64_t(9),    LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(25,3),Point(100,0),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(0),       is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(0),    LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(25,0),Point(100,0),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(0),       is_beyond);
    //Points beyond the horizontal line.
    CPPUNIT_ASSERT_EQUAL(int64_t(10000),LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(200,0),Point(100,0),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(1),       is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(10000),LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(-100,0),Point(100,0),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(-1),      is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(2),    LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(-1,-1),Point(100,0),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(-1),      is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(9),    LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(0,3),Point(100,0),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(0),       is_beyond);
    
    //Vertical line.
    CPPUNIT_ASSERT_EQUAL(int64_t(25),   LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(5,25),Point(0,100),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(0),       is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(0),    LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(0,25),Point(0,100),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(0),       is_beyond);
    //Points beyond the vertical line.
    CPPUNIT_ASSERT_EQUAL(int64_t(10000),LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(0,200),Point(0,100),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(1),       is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(10000),LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(0,-100),Point(0,100),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(-1),      is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(2),    LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(-1,-1),Point(0,100),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(-1),      is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(9),    LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(3,0),Point(0,100),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(0),       is_beyond);
    
    //45 degrees diagonal line.
    CPPUNIT_ASSERT_EQUAL(int64_t(50),   LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(30,20),Point(100,100),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(0),       is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(0),    LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(25,25),Point(100,100),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(0),       is_beyond);
    //Points beyond the diagonal line.
    CPPUNIT_ASSERT_EQUAL(int64_t(20000),LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(200,200),Point(100,100),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(1),       is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(20000),LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(-100,-100),Point(100,100),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(-1),      is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(9),    LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(-3,0),Point(100,100),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(-1),      is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(18),   LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(3,-3),Point(100,100),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(0),       is_beyond);
    
    //Arbitrary degree diagonal line.
    CPPUNIT_ASSERT_EQUAL(int64_t(320),  LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(20,30),Point(100,50),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(0),       is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(0),    LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(40,20),Point(100,50),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(0),       is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(0),    LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(0,0),Point(100,50),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(0),       is_beyond);
    //Points beyond the diagonal line.
    CPPUNIT_ASSERT_EQUAL(int64_t(12500),LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(200,100),Point(100,50),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(1),       is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(12500),LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(-100,-50),Point(100,50),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(-1),      is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(9),    LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(-3,0),Point(100,50),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(-1),      is_beyond);
    CPPUNIT_ASSERT_EQUAL(int64_t(20),   LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(-2,4),Point(100,50),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(0),       is_beyond);
    
    //Zero-distance line.
    CPPUNIT_ASSERT_EQUAL(int64_t(100),  LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(0,0),Point(10,0),&is_beyond));
    CPPUNIT_ASSERT(is_beyond != 0); //It's beyond, but which direction it is beyond in doesn't matter.
    CPPUNIT_ASSERT_EQUAL(int64_t(0),    LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(0,0),Point(0,0),&is_beyond));
    CPPUNIT_ASSERT_EQUAL(char(0),       is_beyond);
}

}