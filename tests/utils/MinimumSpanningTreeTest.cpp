// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/MinimumSpanningTree.h"
#include <algorithm>
#include <gtest/gtest.h>
#include <vector>

// NOLINTBEGIN(*-magic-numbers)
namespace curaengine
{
bool has(const ::curaengine::geometry::Point2LL& pt, const std::vector<::curaengine::geometry::Point2LL>& list)
{
    return std::find(list.begin(), list.end(), pt) != list.end();
}

class MinimumSpanningTreeTest : public ::testing::Test
{
public:
    void SetUp() override
    {
        pts = {
            ::curaengine::geometry::Point2LL(-3, -4), ::curaengine::geometry::Point2LL(3, -4), ::curaengine::geometry::Point2LL(0, -3), ::curaengine::geometry::Point2LL(0, 0), ::curaengine::geometry::Point2LL(1, 1), ::curaengine::geometry::Point2LL(5, 1), ::curaengine::geometry::Point2LL(-1, 6), ::curaengine::geometry::Point2LL(0, 5), ::curaengine::geometry::Point2LL(5, 7), ::curaengine::geometry::Point2LL(12, 12), ::curaengine::geometry::Point2LL(12, 13),
        };
        std::vector<::curaengine::geometry::Point2LL> pts_set(pts.begin(), pts.end());
        p_mst = new ::curaengine::utils::MinimumSpanningTree(pts_set);
    }

    void TearDown() override
    {
        delete p_mst;
        p_mst = nullptr;
    }

    std::vector<::curaengine::geometry::Point2LL> pts;
    ::curaengine::utils::MinimumSpanningTree* p_mst; // Needs to be a pointer, because SetUp isn't a constructor and the copy function is deleted.
};

TEST(SimpleMinimumSpanningTreeTest, TestConstructEmpty)
{
    std::vector<::curaengine::geometry::Point2LL> vertices;
    ::curaengine::utils::MinimumSpanningTree tree(vertices);

    ASSERT_TRUE(tree.leaves().empty()) << "Empty tree should be empty.";
}

TEST(SimpleMinimumSpanningTreeTest, TestConstructOne)
{
    const ::curaengine::geometry::Point2LL pt_a(1, 1);
    std::vector<::curaengine::geometry::Point2LL> vertices = { pt_a };
    ::curaengine::utils::MinimumSpanningTree tree(vertices);

    ASSERT_FALSE(tree.leaves().empty()) << "Tree with one point shouldn't have no vertices.";
    std::vector<::curaengine::geometry::Point2LL> points = tree.vertices();
    EXPECT_EQ(points.size(), 1) << "Tree with one point should have exactly one vertex.";
    EXPECT_EQ(points[0], pt_a) << "Tree with one point should have that point among its vertices.";
}

TEST(SimpleMinimumSpanningTreeTest, TestSimpleAdjacent)
{
    const ::curaengine::geometry::Point2LL pt_a(1, 1);
    const ::curaengine::geometry::Point2LL pt_b(2, 2);
    const ::curaengine::geometry::Point2LL pt_c(3, 3);
    const ::curaengine::geometry::Point2LL pt_d(4, 4);
    std::vector<::curaengine::geometry::Point2LL> vertices = { pt_a, pt_b, pt_c, pt_d };
    ::curaengine::utils::MinimumSpanningTree tree(vertices);

    std::vector<::curaengine::geometry::Point2LL> adjacent;

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

    adjacent = tree.adjacentNodes(::curaengine::geometry::Point2LL(5, 5)); // point E, a non-existent node
    EXPECT_EQ(adjacent.size(), 0) << "No points should be adjacent to point E.";
}

TEST(SimpleMinimumSpanningTreeTest, TestSimpleLeaves)
{
    const ::curaengine::geometry::Point2LL pt_a(1, 1);
    const ::curaengine::geometry::Point2LL pt_b(5, 2);
    const ::curaengine::geometry::Point2LL pt_c(2, 5);
    const ::curaengine::geometry::Point2LL pt_d(3, 3);
    std::vector<::curaengine::geometry::Point2LL> vertices = { pt_a, pt_b, pt_c, pt_d };
    ::curaengine::utils::MinimumSpanningTree tree(vertices);

    std::vector<::curaengine::geometry::Point2LL> leaves = tree.leaves();
    EXPECT_EQ(leaves.size(), 3) << "Three out of four points should be leaves (simple case).";
    EXPECT_TRUE(has(pt_a, leaves)) << "Point A should be one of the leaves (simple case).";
    EXPECT_TRUE(has(pt_b, leaves)) << "Point B should be one of the leaves (simple case).";
    EXPECT_TRUE(has(pt_c, leaves)) << "Point C should be one of the leaves (simple case).";
    EXPECT_FALSE(has(pt_d, leaves)) << "Point D should not be a leave (simple case).";
}

TEST_F(MinimumSpanningTreeTest, TestAdjacent)
{
    static const std::vector<size_t> expected_node_degree = { 1, 1, 3, 2, 3, 1, 1, 3, 2, 2, 1 };
    // constexpr won't work here (yet) on Win.

    ::curaengine::utils::MinimumSpanningTree& mst = *p_mst;

    const size_t len = pts.size();
    for (size_t i_pt = 0; i_pt < len; ++i_pt)
    {
        EXPECT_EQ(expected_node_degree[i_pt], mst.adjacentNodes(pts[i_pt]).size()) << "Degree of node #" << i_pt << " (start @0) should be the expected one.";
    }
}

TEST_F(MinimumSpanningTreeTest, TestLeaves)
{
    static const std::vector<bool> should_be_leave({ true, true, false, false, false, true, true, false, false, false, true });
    // constexpr won't work here (yet) on Win.

    ::curaengine::utils::MinimumSpanningTree& mst = *p_mst;
    const std::vector<::curaengine::geometry::Point2LL> leaves = mst.leaves();

    const size_t len = pts.size();
    for (size_t i_pt = 0; i_pt < len; ++i_pt)
    {
        EXPECT_EQ(should_be_leave[i_pt], has(pts[i_pt], leaves)) << "Leaf-'status' of point #" << i_pt << " (start @0) should be the expected one.";
    }
}
} // namespace curaengine
// NOLINTEND(*-magic-numbers)
