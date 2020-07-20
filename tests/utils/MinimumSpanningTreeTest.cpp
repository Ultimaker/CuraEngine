//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>
#include <unordered_map>

#include "../src/utils/MinimumSpanningTree.h"

namespace cura
{
    bool has(const Point& pt, const std::vector<Point>& list)
    {
        return std::find(list.begin(), list.end(), pt) != list.end();
    }

    class MinimumSpanningTreeTest : public ::testing::Test
    {
    public:
        void SetUp() override
        {
            pts =
            {
                Point(-3, -4),
                Point( 3, -4),
                Point( 0, -3),
                Point( 0,  0),
                Point( 1,  1),
                Point( 5,  1),
                Point(-1,  6),
                Point( 0,  5),
                Point( 5,  7),
                Point(12, 12),
                Point(12, 13),
            };
            std::vector<Point> pts_set(pts.begin(), pts.end());
            p_mst = new MinimumSpanningTree(pts_set);
        }

        void TearDown() override
        {
            delete p_mst;
            p_mst = nullptr;
        }

        std::vector<Point> pts;
        MinimumSpanningTree* p_mst; // Needs to be a pointer, beacuse SetUp isn't a constructor and the copy function is deleted.
    };

    TEST(SimpleMinimumSpanningTreeTest, TestConstructEmpty)
    {
        std::vector<Point> vertices;
        MinimumSpanningTree tree(vertices);

        ASSERT_TRUE(tree.leaves().empty()) << "Empty tree should be empty.";
    }

    TEST(SimpleMinimumSpanningTreeTest, TestConstructOne)
    {
        const Point pt_a(1, 1);
        std::vector<Point> vertices = { pt_a };
        MinimumSpanningTree tree(vertices);

        ASSERT_FALSE(tree.leaves().empty()) << "Tree with one point shouldn't have no vertices.";
        std::vector<Point> points = tree.vertices();
        EXPECT_EQ(points.size(), 1) << "Tree with one point should have exactly one vertex.";
        EXPECT_EQ(points[0], pt_a) << "Tree with one point should have that point among its vertices.";
    }

    TEST(SimpleMinimumSpanningTreeTest, TestSimpleAdjacent)
    {
        const Point pt_a(1, 1);
        const Point pt_b(2, 2);
        const Point pt_c(3, 3);
        const Point pt_d(4, 4);
        std::vector<Point> vertices = { pt_a, pt_b, pt_c, pt_d };
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

    TEST(SimpleMinimumSpanningTreeTest, TestSimpleLeaves)
    {
        const Point pt_a(1, 1);
        const Point pt_b(5, 2);
        const Point pt_c(2, 5);
        const Point pt_d(3, 3);
        std::vector<Point> vertices = { pt_a, pt_b, pt_c, pt_d };
        MinimumSpanningTree tree(vertices);

        std::vector<Point> leaves = tree.leaves();
        EXPECT_EQ(leaves.size(), 3) << "Three out of four points should be leaves (simple case).";
        EXPECT_TRUE(has(pt_a, leaves)) << "Point A should be one of the leaves (simple case).";
        EXPECT_TRUE(has(pt_b, leaves)) << "Point B should be one of the leaves  (simple case).";
        EXPECT_TRUE(has(pt_c, leaves)) << "Point C should be one of the leaves  (simple case).";
        EXPECT_FALSE(has(pt_d, leaves)) << "Point D should not be a leave (simple case).";
    }

    TEST_F(MinimumSpanningTreeTest, TestAdjacent)
    {
        static const std::vector<size_t> expected_node_degree = { 1, 1, 3, 2, 3, 1, 1, 3, 2, 2, 1 };
        // constexpr wont work here (yet) on Win.

        MinimumSpanningTree& mst = *p_mst;

        const size_t len = pts.size();
        for (size_t i_pt = 0; i_pt < len; ++i_pt)
        {
            EXPECT_EQ(expected_node_degree[i_pt], mst.adjacentNodes(pts[i_pt]).size()) << "Degree of node #" << i_pt << " (start @0) should be the expected one.";
        }
    }

    TEST_F(MinimumSpanningTreeTest, TestLeaves)
    {
        static const std::vector<bool> should_be_leave({ true, true, false, false, false, true, true, false, false, false, true });
        // constexpr wont work here (yet) on Win.
        
        MinimumSpanningTree& mst = *p_mst;
        const std::vector<Point> leaves = mst.leaves();

        const size_t len = pts.size();
        for (size_t i_pt = 0; i_pt < len; ++i_pt)
        {
            EXPECT_EQ(should_be_leave[i_pt], has(pts[i_pt], leaves)) << "Leaf-'status' of point #" << i_pt << " (start @0) should be the expected one.";
        }
    }
}
