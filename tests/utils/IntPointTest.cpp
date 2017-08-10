//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "IntPointTest.h"

#include <../src/utils/intpoint.h>

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(IntPointTest);

void IntPointTest::setUp()
{
    //Do nothing.
}

void IntPointTest::tearDown()
{
    //Do nothing.
}

void IntPointTest::test()
{
    PointMatrix rot2d(90);
    Point3Matrix rot_homogeneous(rot2d);
    Point a(20, 10);
    Point b(30, 20);
    Point translated = Point3Matrix::translate(-a).apply(b);
    Point rotated = rot_homogeneous.apply(translated);
    Point rotated_in_place = Point3Matrix::translate(a).apply(rotated);

    Point3Matrix all_in_one = Point3Matrix::translate(a).compose(rot_homogeneous).compose(Point3Matrix::translate(-a));
    Point rotated_in_place_2 = all_in_one.apply(b);

    CPPUNIT_ASSERT_MESSAGE(std::string("Matrix composition with translate and rotate failed"), rotated_in_place == rotated_in_place_2);
}

/*
void LinearAlg2DTest::getDist2FromLineSegmentZeroNearTest()
{
    int64_t supposed_distance = LinearAlg2D::getDist2FromLineSegment(Point(0,0),Point(20,0),Point(0,0));
    int64_t actual_distance = 400;
    std::stringstream ss;
    ss << "Line [0,0] -- [0,0], point [20,0], squared distance was ";
    ss << supposed_distance;
    ss << " rather than ";
    ss << actual_distance;
    ss << ".";
    CPPUNIT_ASSERT_MESSAGE(ss.str(),std::abs(supposed_distance - actual_distance) <= maximum_error);
}
*/

}
