//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PolygonUtilsTest.h"

#include <../src/utils/polygonUtils.h>

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(PolygonUtilsTest);

void PolygonUtilsTest::setUp()
{
    test_square.emplace_back(0, 0);
    test_square.emplace_back(100, 0);
    test_square.emplace_back(100, 100);
    test_square.emplace_back(0, 100);
}

void PolygonUtilsTest::tearDown()
{
    //Do nothing.
}

void PolygonUtilsTest::cornerInsideTest()
{
    moveInsideAssert(test_square, Point(110, 110), 28, Point(80, 80));
}

void PolygonUtilsTest::edgeInsideTest()
{
    moveInsideAssert(test_square, Point(50, 110), 20, Point(50, 80));
}

void PolygonUtilsTest::cornerOutsideTest()
{
    moveInsideAssert(test_square, Point(110, 110), -28, Point(120, 120));
}

void PolygonUtilsTest::edgeOutsideTest()
{
    moveInsideAssert(test_square, Point(50, 110), -20, Point(50, 120));
}

void PolygonUtilsTest::cornerInsideTest2()
{
    moveInside2Assert(test_square, Point(110, 110), 28, Point(80, 80));
}

void PolygonUtilsTest::edgeInsideTest2()
{
    moveInside2Assert(test_square, Point(50, 110), 20, Point(50, 80));
}

void PolygonUtilsTest::cornerOutsideTest2()
{
    moveInside2Assert(test_square, Point(110, 110), -28, Point(120, 120));
}

void PolygonUtilsTest::edgeOutsideTest2()
{
    moveInside2Assert(test_square, Point(50, 110), -20, Point(50, 120));
}


void PolygonUtilsTest::moveInsideAssert(const PolygonRef poly, Point close_to, const int distance, Point supposed)
{
    ClosestPolygonPoint cpp = PolygonUtils::findClosest(close_to, poly);
    Point result = PolygonUtils::moveInside(cpp, distance);
    {
        std::stringstream ss;
        ss << close_to << " moved with " << distance << " micron inside to " << result << " rather than " << supposed << ".\n";
        ss << "\tPS: dist to boundary computed = " << vSize(cpp.location - result) << "; idem supposed = " << vSize(cpp.location - result) << ".\n";
        ss << "\tclosest point = " << cpp.location << " at index " << cpp.pos << ".\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), vSize(result - supposed) <= maximum_error);
    }
}
void PolygonUtilsTest::moveInside2Assert(const PolygonRef poly, Point close_to, const int distance, Point supposed)
{
    Polygons polys;
    polys.add(poly);
    Point result = close_to;
    PolygonUtils::moveInside(polys, result, distance);
    {
        std::stringstream ss;
        ss << close_to << " moved with " << distance << " micron inside to " << result << " rather than " << supposed << ".\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), vSize(result - supposed) <= maximum_error);
    }
}

void PolygonUtilsTest::cornerFindCloseTest()
{
    findCloseAssert(test_square, Point(110,110), Point(100,100), 15);
}

void PolygonUtilsTest::edgeFindCloseTest()
{
    findCloseAssert(test_square, Point(50,110), Point(50,100), 15);
}



void PolygonUtilsTest::findCloseAssert(const PolygonRef poly, Point close_to, Point supposed, int cell_size)
{
    Polygons polys;
    polys.add(poly);
    BucketGrid2D<PolygonsPointIndex>* loc_to_line = PolygonUtils::createLocToLineGrid(polys, cell_size);
    
    ClosestPolygonPoint* cpp = PolygonUtils::findClose(close_to, polys, *loc_to_line);
    if (cpp)
    {
        std::stringstream ss;
        Point result = cpp->location;
        ss << "Close to " << close_to << " we found " << result << " rather than " << supposed << ".\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), vSize(result - supposed) <= maximum_error);
    }
    else 
    {
        std::stringstream ss;
        ss << "Couldn't find anything close to " << close_to << " ( should have been " << supposed << ").\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), false);
    }
    
    delete loc_to_line;
}

}