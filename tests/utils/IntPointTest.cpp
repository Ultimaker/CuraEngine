// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include "geometry/Point2LL.h"
#include "geometry/Point3Matrix.h"
#include "geometry/PointMatrix.h"

// NOLINTBEGIN(*-magic-numbers)
namespace curaengine
{
TEST(IntPointTest, TestRotationMatrix)
{
    ::curaengine::geometry::PointMatrix rot2d(90);
    ::curaengine::geometry::Point3Matrix rot_homogeneous(rot2d);
    ::curaengine::geometry::Point2LL a(20, 10);
    ::curaengine::geometry::Point2LL b(30, 20);
    ::curaengine::geometry::Point2LL translated = ::curaengine::geometry::Point3Matrix::translate(-a).apply(b);
    ::curaengine::geometry::Point2LL rotated = rot_homogeneous.apply(translated);
    ::curaengine::geometry::Point2LL rotated_in_place = ::curaengine::geometry::Point3Matrix::translate(a).apply(rotated);

    ::curaengine::geometry::Point3Matrix all_in_one = ::curaengine::geometry::Point3Matrix::translate(a).compose(rot_homogeneous).compose(::curaengine::geometry::Point3Matrix::translate(-a));
    ::curaengine::geometry::Point2LL rotated_in_place_2 = all_in_one.apply(b);

    ASSERT_EQ(rotated_in_place, rotated_in_place_2) << "Matrix composition with translate and rotate failed.";
}

TEST(IntPointTest, TestSize)
{
    ASSERT_EQ(sizeof(::curaengine::geometry::Point2LL::X), sizeof(::curaengine::utils::coord_t));
    ASSERT_LE(sizeof(::curaengine::utils::coord_t), sizeof(int64_t));
}

} // namespace curaengine
// NOLINTEND(*-magic-numbers)
