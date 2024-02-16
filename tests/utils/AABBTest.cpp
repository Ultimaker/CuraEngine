// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/AABB.h"
#include "utils/polygon.h"
#include <gtest/gtest.h>
#include <polyclipping/clipper.hpp>

namespace cura
{
// NOLINTBEGIN(*-magic-numbers)
TEST(AABBTest, TestConstructEmpty)
{
    AABB empty_box;

    EXPECT_FALSE(empty_box.contains(Point2LL(0, 0))) << "Empty box shouldn't contain anything.";
    EXPECT_FALSE(empty_box.contains(empty_box.getMiddle())) << "Empty box shouldn't contain anything, even it's own middle.";
    EXPECT_FALSE(empty_box.hit(empty_box)) << "Empty boxes shouldn't intersect.";

    empty_box.include(Point2LL(-10, -5));
    empty_box.include(Point2LL(5, 10));

    EXPECT_TRUE(empty_box.contains(Point2LL(0, 0))) << "The previously empty box should now contain this point.";
    EXPECT_FALSE(empty_box.contains(Point2LL(11, 5))) << "The previously empty box should now still not contain this point.";
}

TEST(AABBTest, TestConstructPoint)
{
    AABB point_box(Point2LL(-10, -5), Point2LL(5, 10));

    EXPECT_TRUE(point_box.contains(Point2LL(0, 0))) << "Box constructed from points around the origin should contain it.";
    EXPECT_FALSE(point_box.contains(Point2LL(11, 5))) << "The box shouldn't contain a point outside of it.";
}

TEST(AABBTest, TestConstructPolygons)
{
    Polygons empty_polygon;
    AABB polygons_box_a(empty_polygon);

    EXPECT_FALSE(polygons_box_a.contains(Point2LL(0, 0))) << "Box constructed from empty polygon shouldn't contain anything.";

    Polygons polygons;
    polygons.add(Polygon(ClipperLib::Path({ ClipperLib::IntPoint{ -10, -10 }, ClipperLib::IntPoint{ 10, -10 }, ClipperLib::IntPoint{ -5, -5 }, ClipperLib::IntPoint{ -10, 10 } })));
    polygons.add(Polygon(ClipperLib::Path({ ClipperLib::IntPoint{ 11, 11 }, ClipperLib::IntPoint{ -11, 11 }, ClipperLib::IntPoint{ 4, 4 }, ClipperLib::IntPoint{ 11, -11 } })));
    polygons.add(Polygon(ClipperLib::Path({ ClipperLib::IntPoint{ 2, 2 }, ClipperLib::IntPoint{ 2, 3 }, ClipperLib::IntPoint{ 3, 3 }, ClipperLib::IntPoint{ 3, 2 } })));

    AABB polygons_box_b(polygons);

    EXPECT_TRUE(polygons_box_b.contains(Point2LL(0, 0))) << "Polygon box should contain origin, even though origin is outside of the original polygons.";
    EXPECT_TRUE(polygons_box_b.contains(Point2LL(-7, -7))) << "Polygon box should contain point that was inside of the original polygons.";
    EXPECT_FALSE(polygons_box_b.contains(Point2LL(12, 12))) << "Polygon box should not contain point outside of the AABB of the polygon.";
}

TEST(AABBTest, TestConstructInverse)
{
    AABB inverse_box(Point2LL(5, 10), Point2LL(-10, -5));

    EXPECT_FALSE(inverse_box.contains(Point2LL(0, 0))) << "'Inverse' box shouldn't contain anything.";
    EXPECT_FALSE(inverse_box.contains(inverse_box.getMiddle())) << "'Inverse' box shouldn't contain anything, even it's own middle.";
    EXPECT_FALSE(inverse_box.hit(inverse_box)) << "'Inverse' boxes shouldn't intersect.";

    inverse_box.include(Point2LL(-5, -2));
    inverse_box.include(Point2LL(2, 5));

    EXPECT_TRUE(inverse_box.contains(Point2LL(0, 0))) << "The previously 'inverse' box should now contain this point.";
    EXPECT_FALSE(inverse_box.contains(Point2LL(4, 8))) << "The previously 'inverse' box should now still not contain this point.";
}

TEST(AABBTest, TestContains)
{
    AABB box(Point2LL(-10, -5), Point2LL(5, 10));

    EXPECT_FALSE(box.contains(Point2LL(-16, 16))) << "Box constructed from points shouldn't contain a point outside of the box.";
    EXPECT_TRUE(box.contains(Point2LL(3, 10))) << "Box constructed from points should contain a point on its edge.";
    EXPECT_TRUE(box.contains(Point2LL(5, 10))) << "Box constructed from points should contain its edge-points.";
    EXPECT_TRUE(box.contains(Point2LL(0, 0))) << "Box constructed from points should contain the origin.";
}

TEST(AABBTest, TestHit)
{
    AABB box_a(Point2LL(-10, -5), Point2LL(5, 10));
    AABB box_b(Point2LL(4, 9), Point2LL(12, 12));
    AABB box_c(Point2LL(11, 11), Point2LL(14, 14));

    EXPECT_TRUE(box_a.hit(box_a)) << "Box should overlap itself.";

    EXPECT_TRUE(box_a.hit(box_b)) << "These boxes should overlap (case AB).";
    EXPECT_TRUE(box_b.hit(box_a)) << "These boxes should overlap (case BA).";
    EXPECT_TRUE(box_b.hit(box_c)) << "These boxes should overlap (case BC).";
    EXPECT_TRUE(box_c.hit(box_b)) << "These boxes should overlap (case CB).";

    EXPECT_FALSE(box_a.hit(box_c)) << "These boxes should not overlap (case AC).";
    EXPECT_FALSE(box_c.hit(box_a)) << "These boxes should not overlap (case CA).";

    AABB box_d(Point2LL(3, 10), Point2LL(12, 12));
    AABB box_e(Point2LL(5, 10), Point2LL(12, 12));

    EXPECT_TRUE(box_a.hit(box_d)) << "Overlap-check is inclusive (case AD).";
    EXPECT_TRUE(box_d.hit(box_a)) << "Overlap-check is inclusive (case DA).";
    EXPECT_TRUE(box_a.hit(box_e)) << "Overlap-check is inclusive (case AE).";
    EXPECT_TRUE(box_e.hit(box_a)) << "Overlap-check is inclusive (case EA).";
}

TEST(AABBTest, TestGetMiddle)
{
    AABB box_a(Point2LL(-10, -6), Point2LL(6, 10));
    AABB box_b(Point2LL(4, 10), Point2LL(12, 12));

    EXPECT_EQ(box_a.getMiddle(), Point2LL(-2, 2)) << "The middle of the AABB should be this point (case A).";
    EXPECT_EQ(box_b.getMiddle(), Point2LL(8, 11)) << "The middle of the AABB should be this point (case B).";
}

TEST(AABBTest, TestInclude)
{
    AABB box(Point2LL(2, 2), Point2LL(5, 10));

    EXPECT_FALSE(box.contains(Point2LL(1, 1))) << "The unexpanded (via include/point) box should not contain a point in the (future) expanded area.";

    box.include(Point2LL(0, 0));

    EXPECT_TRUE(box.contains(Point2LL(1, 1))) << "The expanded (via include/point) box should contain a point in the expanded area.";
    EXPECT_FALSE(box.contains(Point2LL(6, 9))) << "The unexpanded (via include/other) box should not contain a point in the (future) expanded area.";

    box.include(AABB(Point2LL(7, 9), Point2LL(8, 10)));

    EXPECT_TRUE(box.contains(Point2LL(6, 9))) << "The expanded (via include/other) box should contain a point in the expanded area.";

    const Point2LL a(2, 2);
    const Point2LL b(5, 10);
    AABB box2(a, b);
    AABB empty;
    box2.include(empty);

    EXPECT_EQ(box2.min_, a) << "Inclusion of an 'empty' or negative box should not change the minimum of the original.";
    EXPECT_EQ(box2.max_, b) << "Inclusion of an 'empty' or negative box should not change the maximum of the original.";
}

TEST(AABBTest, TestExpand)
{
    AABB box(Point2LL(-10, -5), Point2LL(5, 10));

    EXPECT_FALSE(box.contains(Point2LL(6, 11))) << "Before expanding, the box shouldn't contain this point.";

    box.expand(2);

    EXPECT_TRUE(box.contains(Point2LL(6, 11))) << "After expanding, the box should contain this point.";

    box.expand(-2);

    EXPECT_FALSE(box.contains(Point2LL(6, 11))) << "After shrinking, the box shouldn't contain this point anymore.";
}

TEST(AABBTest, TestToPolygon)
{
    AABB box(Point2LL(-10, -5), Point2LL(5, 10));

    Polygon polygon = box.toPolygon();

    EXPECT_EQ(polygon.area(), (box.max_.X - box.min_.X) * (box.max_.Y - box.min_.Y)) << "The polygon from the bounding box should have the same area.";
    EXPECT_EQ(polygon.centerOfMass(), box.getMiddle()) << "The center of mass of an (AA) rectangle is its middle.";
}
// NOLINTEND(*-magic-numbers)

} // namespace cura