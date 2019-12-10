//Copyright (c) 2017 Tim Kuipers
//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>
#include <../src/utils/IntPoint.h>

namespace cura
{

TEST(IntPointTest, TestRotationMatrix)
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

    ASSERT_EQ(rotated_in_place, rotated_in_place_2) << "Matrix composition with translate and rotate failed.";
}

}