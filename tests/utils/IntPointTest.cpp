// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include "geometry/Point2LL.h"
#include "geometry/Point3Matrix.h"
#include "geometry/PointMatrix.h"

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{
TEST(IntPointTest, TestRotationMatrix)
{
    PointMatrix rot2d(90);
    Point3Matrix rot_homogeneous(rot2d);
    Point2LL a(20, 10);
    Point2LL b(30, 20);
    Point2LL translated = Point3Matrix::translate(-a).apply(b);
    Point2LL rotated = rot_homogeneous.apply(translated);
    Point2LL rotated_in_place = Point3Matrix::translate(a).apply(rotated);

    Point3Matrix all_in_one = Point3Matrix::translate(a).compose(rot_homogeneous).compose(Point3Matrix::translate(-a));
    Point2LL rotated_in_place_2 = all_in_one.apply(b);

    ASSERT_EQ(rotated_in_place, rotated_in_place_2) << "Matrix composition with translate and rotate failed.";
}

TEST(IntPointTest, TestSize)
{
    ASSERT_EQ(sizeof(Point2LL::X), sizeof(coord_t));
    ASSERT_LE(sizeof(coord_t), sizeof(int64_t));
}

} // namespace cura
// NOLINTEND(*-magic-numbers)
