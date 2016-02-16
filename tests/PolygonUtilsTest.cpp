//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PolygonUtilsTest.h"

#include <../src/utils/polygonUtils.h>

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(PolygonUtilsTest);

void PolygonUtilsTest::setUp()
{
    //Do nothing.
}

void PolygonUtilsTest::tearDown()
{
    //Do nothing.
}

void PolygonUtilsTest::cornerInsideTest()
{
    Polygon poly;
    poly.emplace_back(0, 0);
    poly.emplace_back(100, 0);
    poly.emplace_back(100, 100);
    poly.emplace_back(0, 100);
    moveInsideAssert(poly, Point(110, 110), 28, Point(80, 80));
}

void PolygonUtilsTest::edgeInsideTest()
{
    Polygon poly;
    poly.emplace_back(0, 0);
    poly.emplace_back(100, 0);
    poly.emplace_back(100, 100);
    poly.emplace_back(0, 100);
    moveInsideAssert(poly, Point(50, 110), 20, Point(50, 80));
}

void PolygonUtilsTest::cornerOutsideTest()
{
    Polygon poly;
    poly.emplace_back(0, 0);
    poly.emplace_back(100, 0);
    poly.emplace_back(100, 100);
    poly.emplace_back(0, 100);
    moveInsideAssert(poly, Point(110, 110), -28, Point(120, 120));
}

void PolygonUtilsTest::edgeOutsideTest()
{
    Polygon poly;
    poly.emplace_back(0, 0);
    poly.emplace_back(100, 0);
    poly.emplace_back(100, 100);
    poly.emplace_back(0, 100);
    moveInsideAssert(poly, Point(50, 110), -20, Point(50, 120));
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

}