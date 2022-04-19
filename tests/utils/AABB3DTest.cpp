//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>
#include <polyclipping/clipper.hpp>

#include "../src/utils/AABB.h"
#include "../src/utils/AABB3D.h"
#include "../src/utils/polygon.h"

namespace cura
{
    inline AABB3D toBox(const coord_t& x, const coord_t& y, const coord_t& z)
    {
        const Point3 pt(x, y, z);
        return AABB3D(pt, pt);
    }

    TEST(AABB3DTest, TestConstructEmpty)
    {
        AABB3D empty_box;

        EXPECT_FALSE(empty_box.hit(toBox(0, 0, 0))) << "Empty box shouldn't contain anything.";
        EXPECT_FALSE(empty_box.hit(empty_box)) << "Empty boxes shouldn't intersect.";

        empty_box.include(Point3(-10, -5, -2));
        empty_box.include(Point3(5, 10, 2));

        EXPECT_TRUE(empty_box.hit(toBox(0, 0, 0))) << "The previously empty box should now contain this point.";
        EXPECT_FALSE(empty_box.hit(toBox(11, 5, 0))) << "The previously empty box should now still not contain this point.";
    }

    TEST(AABB3DTest, TestConstructPoint)
    {
        AABB3D point_box(Point3(-10, -5, -2), Point3(5, 10, 2));

        EXPECT_TRUE(point_box.hit(toBox(0, 0, 0))) << "Box constructed from points around the origin should contain it.";
        EXPECT_FALSE(point_box.hit(toBox(11, 5, 0))) << "The box shouldn't contain a point outside of it.";
    }

    TEST(AABB3DTest, TestConstructInverse)
    {
        AABB3D inverse_box(Point3(5, 10, 2), Point3(-10, -5, -2));

        EXPECT_FALSE(inverse_box.hit(toBox(0, 0, 0))) << "'Inverse' box shouldn't contain anything.";
        EXPECT_FALSE(inverse_box.hit(inverse_box)) << "'Inverse' boxes shouldn't intersect.";

        inverse_box.include(Point3(-5, -2, -1));
        inverse_box.include(Point3(2, 5, 1));

        EXPECT_TRUE(inverse_box.hit(toBox(0, 0, 0))) << "The previously 'inverse' box should now contain this point.";
        EXPECT_FALSE(inverse_box.hit(toBox(4, 8, -1))) << "The previously 'inverse' box should now still not contain this point.";
    }

    TEST(AABB3DTest, TestHit)
    {
        AABB3D box_a(Point3(-10, -5, -2), Point3(5, 10, 2));
        AABB3D box_b(Point3(4, 9, 0), Point3(12, 12, 12));
        AABB3D box_c(Point3(11, 11, 11), Point3(14, 14, 14));

        EXPECT_TRUE(box_a.hit(box_a)) << "Box should overlap itself.";

        EXPECT_TRUE(box_a.hit(box_b)) << "These boxes should overlap (case AB).";
        EXPECT_TRUE(box_b.hit(box_a)) << "These boxes should overlap (case BA).";
        EXPECT_TRUE(box_b.hit(box_c)) << "These boxes should overlap (case BC).";
        EXPECT_TRUE(box_c.hit(box_b)) << "These boxes should overlap (case CB).";

        EXPECT_FALSE(box_a.hit(box_c)) << "These boxes should not overlap (case AC).";
        EXPECT_FALSE(box_c.hit(box_a)) << "These boxes should not overlap (case CA).";

        AABB3D box_d(Point3(3, 10, 2), Point3(12, 12, 12));
        AABB3D box_e(Point3(5, 10, 2), Point3(12, 12, 12));

        EXPECT_TRUE(box_a.hit(box_d)) << "Overlap-check is inclusive (case AD).";
        EXPECT_TRUE(box_d.hit(box_a)) << "Overlap-check is inclusive (case DA).";
        EXPECT_TRUE(box_a.hit(box_e)) << "Overlap-check is inclusive (case AE).";
        EXPECT_TRUE(box_e.hit(box_a)) << "Overlap-check is inclusive (case EA).";
    }

    TEST(AABB3DTest, TestGetMiddle)
    {
        AABB3D box_a(Point3(-10, -6, -5), Point3(6, 10, 3));
        AABB3D box_b(Point3(4, 10, 2), Point3(12, 12, 12));

        EXPECT_EQ(box_a.getMiddle(), Point3(-2, 2, -1)) << "The middle of the AABB should be this point (case A).";
        EXPECT_EQ(box_b.getMiddle(), Point3(8, 11, 7)) << "The middle of the AABB should be this point (case B).";
    }

    TEST(AABB3DTest, TestInclude)
    {
        AABB3D box(Point3(2, 2, 2), Point3(5, 10, 3));

        EXPECT_FALSE(box.hit(toBox(1, 1, 1))) << "The unexpanded (via include/point) box should not contain a point in the (future) expanded area.";

        box.include(Point3(0, 0, 0));

        EXPECT_TRUE(box.hit(toBox(1, 1, 1))) << "The expanded (via include/point) box should contain a point in the expanded area.";
        EXPECT_FALSE(box.hit(toBox(6, 9, -1))) << "The unexpanded (via include/other) box should not contain a point in the (future) expanded area.";

        box.include(AABB3D(Point3(7, 9, -2), Point3(8, 10, 0)));

        EXPECT_TRUE(box.hit(toBox(6, 9, -1))) << "The expanded (via include/other) box should contain a point in the expanded area.";

        box.includeZ(-6);

        EXPECT_TRUE(box.hit(toBox(6, 9, -3))) << "The expanded (via includeZ/scalar) box should contain a point in the expanded area.";

        const Point3 a(2, 2, 2);
        const Point3 b(5, 10, 15);
        AABB3D box2(a, b);
        AABB3D empty;
        box2.include(empty);

        EXPECT_EQ(box2.min, a) << "Inclusion of an 'empty' or negative box should not change the minimum of the original.";
        EXPECT_EQ(box2.max, b) << "Inclusion of an 'empty' or negative box should not change the maximum of the original.";
    }

    TEST(AABB3DTest, TestOffset)
    {
        AABB3D box(Point3(2, 2, 2), Point3(5, 10, 3));

        EXPECT_FALSE(box.hit(toBox(1, 1, 1))) << "The unexpanded (via offset-3D) box should not contain a point in the (future) expanded area.";

        box.offset(Point3(-2, -2, -2));

        EXPECT_TRUE(box.hit(toBox(1, 1, 1))) << "The expanded (via offset-3D) box should contain a point in the expanded area.";
        EXPECT_FALSE(box.hit(toBox(6, 9, -1))) << "The unexpanded (via offset-3D) box should not contain a point in the (future) expanded area.";

        box.offset(Point(-2, -2));

        EXPECT_TRUE(box.hit(toBox(-1, -1, 0))) << "The expanded (via offset-2D) box should contain a point in the expanded area.";
    }

    TEST(AABB3DTest, TestExpand)
    {
        AABB3D box(Point3(-10, -5, -2), Point3(5, 10, 2));

        EXPECT_FALSE(box.hit(toBox(6, 11, 3))) << "Before expanding, the box shouldn't contain this point.";

        box.expandXY(2);

        EXPECT_TRUE(box.hit(toBox(6, 11, 1))) << "After expanding, the box should contain this point.";

        box.expandXY(-2);

        EXPECT_FALSE(box.hit(toBox(6, 11, 1))) << "After shrinking, the box shouldn't contain this point anymore.";
    }

    TEST(AABB3DTest, TestFlatten)
    {
        AABB3D box(Point3(-10, -5, -2), Point3(5, 10, 2));

        AABB flat = box.flatten();

        EXPECT_TRUE(flat.contains(Point(1, 1))) << "The flattened box should contain this point.";
        EXPECT_FALSE(flat.contains(Point(-11, 3))) << "The flattened box shouldn't contain this point.";
    }
}
