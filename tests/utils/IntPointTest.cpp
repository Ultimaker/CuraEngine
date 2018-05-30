//Copyright (c) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "IntPointTest.h"

#include <../src/utils/IntPoint.h>

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

void IntPointTest::testRotationMatrix()
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

}
