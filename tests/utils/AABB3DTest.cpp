// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/AABB3D.h"
#include "utils/AABB.h"
#include "utils/Coord_t.h"
#include "geometry/Polygon.h"
#include <gtest/gtest.h>
#include <polyclipping/clipper.hpp>

// NOLINTBEGIN(*-magic-numbers)
namespace curaengine
{
inline ::curaengine::utils::AABB3D toBox(const ::curaengine::utils::coord_t& x, const ::curaengine::utils::coord_t& y, const ::curaengine::utils::coord_t& z)
{
    const ::curaengine::geometry::Point3LL pt(x, y, z);
    return { pt, pt };
}

TEST(AABB3DTest, TestConstructEmpty)
{
    ::curaengine::utils::AABB3D empty_box;

    EXPECT_FALSE(empty_box.hit(toBox(0, 0, 0))) << "Empty box shouldn't contain anything.";
    EXPECT_FALSE(empty_box.hit(empty_box)) << "Empty boxes shouldn't intersect.";

    empty_box.include(::curaengine::geometry::Point3LL(-10, -5, -2));
    empty_box.include(::curaengine::geometry::Point3LL(5, 10, 2));

    EXPECT_TRUE(empty_box.hit(toBox(0, 0, 0))) << "The previously empty box should now contain this point.";
    EXPECT_FALSE(empty_box.hit(toBox(11, 5, 0))) << "The previously empty box should now still not contain this point.";
}

TEST(AABB3DTest, TestConstructPoint)
{
    ::curaengine::utils::AABB3D point_box(::curaengine::geometry::Point3LL(-10, -5, -2), ::curaengine::geometry::Point3LL(5, 10, 2));

    EXPECT_TRUE(point_box.hit(toBox(0, 0, 0))) << "Box constructed from points around the origin should contain it.";
    EXPECT_FALSE(point_box.hit(toBox(11, 5, 0))) << "The box shouldn't contain a point outside of it.";
}

TEST(AABB3DTest, TestConstructInverse)
{
    ::curaengine::utils::AABB3D inverse_box(::curaengine::geometry::Point3LL(5, 10, 2), ::curaengine::geometry::Point3LL(-10, -5, -2));

    EXPECT_FALSE(inverse_box.hit(toBox(0, 0, 0))) << "'Inverse' box shouldn't contain anything.";
    EXPECT_FALSE(inverse_box.hit(inverse_box)) << "'Inverse' boxes shouldn't intersect.";

    inverse_box.include(::curaengine::geometry::Point3LL(-5, -2, -1));
    inverse_box.include(::curaengine::geometry::Point3LL(2, 5, 1));

    EXPECT_TRUE(inverse_box.hit(toBox(0, 0, 0))) << "The previously 'inverse' box should now contain this point.";
    EXPECT_FALSE(inverse_box.hit(toBox(4, 8, -1))) << "The previously 'inverse' box should now still not contain this point.";
}

TEST(AABB3DTest, TestHit)
{
    ::curaengine::utils::AABB3D box_a(::curaengine::geometry::Point3LL(-10, -5, -2), ::curaengine::geometry::Point3LL(5, 10, 2));
    ::curaengine::utils::AABB3D box_b(::curaengine::geometry::Point3LL(4, 9, 0), ::curaengine::geometry::Point3LL(12, 12, 12));
    ::curaengine::utils::AABB3D box_c(::curaengine::geometry::Point3LL(11, 11, 11), ::curaengine::geometry::Point3LL(14, 14, 14));

    EXPECT_TRUE(box_a.hit(box_a)) << "Box should overlap itself.";

    EXPECT_TRUE(box_a.hit(box_b)) << "These boxes should overlap (case AB).";
    EXPECT_TRUE(box_b.hit(box_a)) << "These boxes should overlap (case BA).";
    EXPECT_TRUE(box_b.hit(box_c)) << "These boxes should overlap (case BC).";
    EXPECT_TRUE(box_c.hit(box_b)) << "These boxes should overlap (case CB).";

    EXPECT_FALSE(box_a.hit(box_c)) << "These boxes should not overlap (case AC).";
    EXPECT_FALSE(box_c.hit(box_a)) << "These boxes should not overlap (case CA).";

    ::curaengine::utils::AABB3D box_d(::curaengine::geometry::Point3LL(3, 10, 2), ::curaengine::geometry::Point3LL(12, 12, 12));
    ::curaengine::utils::AABB3D box_e(::curaengine::geometry::Point3LL(5, 10, 2), ::curaengine::geometry::Point3LL(12, 12, 12));

    EXPECT_TRUE(box_a.hit(box_d)) << "Overlap-check is inclusive (case AD).";
    EXPECT_TRUE(box_d.hit(box_a)) << "Overlap-check is inclusive (case DA).";
    EXPECT_TRUE(box_a.hit(box_e)) << "Overlap-check is inclusive (case AE).";
    EXPECT_TRUE(box_e.hit(box_a)) << "Overlap-check is inclusive (case EA).";
}

TEST(AABB3DTest, TestGetMiddle)
{
    ::curaengine::utils::AABB3D box_a(::curaengine::geometry::Point3LL(-10, -6, -5), ::curaengine::geometry::Point3LL(6, 10, 3));
    ::curaengine::utils::AABB3D box_b(::curaengine::geometry::Point3LL(4, 10, 2), ::curaengine::geometry::Point3LL(12, 12, 12));

    EXPECT_EQ(box_a.getMiddle(), ::curaengine::geometry::Point3LL(-2, 2, -1)) << "The middle of the AABB should be this point (case A).";
    EXPECT_EQ(box_b.getMiddle(), ::curaengine::geometry::Point3LL(8, 11, 7)) << "The middle of the AABB should be this point (case B).";
}

TEST(AABB3DTest, TestInclude)
{
    ::curaengine::utils::AABB3D box(::curaengine::geometry::Point3LL(2, 2, 2), ::curaengine::geometry::Point3LL(5, 10, 3));

    EXPECT_FALSE(box.hit(toBox(1, 1, 1))) << "The unexpanded (via include/point) box should not contain a point in the (future) expanded area.";

    box.include(::curaengine::geometry::Point3LL(0, 0, 0));

    EXPECT_TRUE(box.hit(toBox(1, 1, 1))) << "The expanded (via include/point) box should contain a point in the expanded area.";
    EXPECT_FALSE(box.hit(toBox(6, 9, -1))) << "The unexpanded (via include/other) box should not contain a point in the (future) expanded area.";

    box.include(::curaengine::utils::AABB3D(::curaengine::geometry::Point3LL(7, 9, -2), ::curaengine::geometry::Point3LL(8, 10, 0)));

    EXPECT_TRUE(box.hit(toBox(6, 9, -1))) << "The expanded (via include/other) box should contain a point in the expanded area.";

    box.includeZ(-6);

    EXPECT_TRUE(box.hit(toBox(6, 9, -3))) << "The expanded (via includeZ/scalar) box should contain a point in the expanded area.";

    const ::curaengine::geometry::Point3LL a(2, 2, 2);
    const ::curaengine::geometry::Point3LL b(5, 10, 15);
    ::curaengine::utils::AABB3D box2(a, b);
    ::curaengine::utils::AABB3D empty;
    box2.include(empty);

    EXPECT_EQ(box2.min_, a) << "Inclusion of an 'empty' or negative box should not change the minimum of the original.";
    EXPECT_EQ(box2.max_, b) << "Inclusion of an 'empty' or negative box should not change the maximum of the original.";
}

TEST(AABB3DTest, TestTranslate)
{
    ::curaengine::utils::AABB3D box(::curaengine::geometry::Point3LL(2, 2, 2), ::curaengine::geometry::Point3LL(5, 10, 3));

    EXPECT_FALSE(box.hit(toBox(1, 1, 1))) << "The unexpanded (via offset-3D) box should not contain a point in the (future) expanded area.";

    box.translate(::curaengine::geometry::Point3LL(-2, -2, -2));

    EXPECT_TRUE(box.hit(toBox(1, 1, 1))) << "The expanded (via offset-3D) box should contain a point in the expanded area.";
    EXPECT_FALSE(box.hit(toBox(6, 9, -1))) << "The unexpanded (via offset-3D) box should not contain a point in the (future) expanded area.";

    box.translate(::curaengine::geometry::Point2LL(-2, -2));

    EXPECT_TRUE(box.hit(toBox(-1, -1, 0))) << "The expanded (via offset-2D) box should contain a point in the expanded area.";
}

TEST(AABB3DTest, TestExpand)
{
    ::curaengine::utils::AABB3D box(::curaengine::geometry::Point3LL(-10, -5, -2), ::curaengine::geometry::Point3LL(5, 10, 2));

    EXPECT_FALSE(box.hit(toBox(6, 11, 3))) << "Before expanding, the box shouldn't contain this point.";

    box.expandXY(2);

    EXPECT_TRUE(box.hit(toBox(6, 11, 1))) << "After expanding, the box should contain this point.";

    box.expandXY(-2);

    EXPECT_FALSE(box.hit(toBox(6, 11, 1))) << "After shrinking, the box shouldn't contain this point anymore.";
}

TEST(AABB3DTest, TestFlatten)
{
    ::curaengine::utils::AABB3D box(::curaengine::geometry::Point3LL(-10, -5, -2), ::curaengine::geometry::Point3LL(5, 10, 2));

    ::curaengine::utils::AABB flat = box.flatten();

    EXPECT_TRUE(flat.contains(::curaengine::geometry::Point2LL(1, 1))) << "The flattened box should contain this point.";
    EXPECT_FALSE(flat.contains(::curaengine::geometry::Point2LL(-11, 3))) << "The flattened box shouldn't contain this point.";
}
} // namespace curaengine
// NOLINTEND(*-magic-numbers)
