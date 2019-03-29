//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "../src/utils/MinimumSpanningTree.h"

namespace cura
{
    bool has(const Point& pt, const std::vector<Point>& list)
    {
        return std::find(list.begin(), list.end(), pt) != list.end();
    }

    TEST(MinimumSpanningTreeTest, TestConstructEmpty)
    {
        std::unordered_set<Point> vertices;
        MinimumSpanningTree tree(vertices);

        ASSERT_TRUE(tree.leaves().empty()) << "Empty tree should be empty.";
    }

    TEST(MinimumSpanningTreeTest, TestConstructOne)
    {
        const Point pt_a(1, 1);
        std::unordered_set<Point> vertices = { pt_a };
        MinimumSpanningTree tree(vertices);

        ASSERT_FALSE(tree.leaves().empty()) << "Tree with one point shouldn't have no vertices.";
        std::vector<Point> points = tree.vertices();
        EXPECT_EQ(points.size(), 1) << "Tree with one point should have exactly one vertex.";
        EXPECT_EQ(points[0], pt_a) << "Tree with one point should have that point among its vertices.";
    }

    TEST(MinimumSpanningTreeTest, TestSimpleAdjacent)
    {
        const Point pt_a(1, 1);
        const Point pt_b(2, 2);
        const Point pt_c(3, 3);
        const Point pt_d(4, 4);
        std::unordered_set<Point> vertices = { pt_a, pt_b, pt_c, pt_d };
        MinimumSpanningTree tree(vertices);

        std::vector<Point> adjacent;

        adjacent = tree.adjacentNodes(pt_b);
        EXPECT_EQ(adjacent.size(), 2) << "2 points should be adjacent to point B (simple case).";
        EXPECT_TRUE(has(pt_a, adjacent)) << "Point A should be adjacent to Point B (simple case).";
        EXPECT_FALSE(has(pt_b, adjacent)) << "Point B should not be adjacent to itself (simple case).";
        EXPECT_TRUE(has(pt_c, adjacent)) << "Point C should be adjacent to Point B (simple case).";
        EXPECT_FALSE(has(pt_d, adjacent)) << "Point D should not be adjacent to Point B (simple case).";

        adjacent = tree.adjacentNodes(pt_d);
        EXPECT_EQ(adjacent.size(), 1) << "1 point should be adjacent to point D (simple case).";
        EXPECT_TRUE(has(pt_c, adjacent)) << "Point C should be adjacent to Point D (simple case).";
        EXPECT_FALSE(has(pt_d, adjacent)) << "Point D should not be adjacent to itself (simple case).";

        adjacent = tree.adjacentNodes(Point(5, 5)); // point E, a non-existant node
        EXPECT_EQ(adjacent.size(), 0) << "No points should be adjacent to point E.";;
    }

    TEST(MinimumSpanningTreeTest, TestSimpleLeaves)
    {
        Point pt_a(1, 1);
        Point pt_b(5, 2);
        Point pt_c(2, 5);
        Point pt_d(3, 3);
        std::unordered_set<Point> vertices = { pt_a, pt_b, pt_c, pt_d };
        MinimumSpanningTree tree(vertices);

        std::vector<Point> leaves = tree.leaves();
        EXPECT_EQ(leaves.size(), 3) << "Three out of four points should be leaves (simple case).";
        EXPECT_TRUE(has(pt_a, leaves)) << "Point A should be one of the leaves (simple case).";
        EXPECT_TRUE(has(pt_b, leaves)) << "Point B should be one of the leaves  (simple case).";
        EXPECT_TRUE(has(pt_c, leaves)) << "Point C should be one of the leaves  (simple case).";
        EXPECT_FALSE(has(pt_d, leaves)) << "Point D should not be a leave (simple case).";
    }
}
